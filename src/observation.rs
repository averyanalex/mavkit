use crate::error::VehicleError;
use crate::telemetry::{TelemetryMessageKind, VehicleTimestamp};
use std::pin::Pin;
use std::sync::{Arc, Mutex};
use std::task::{Context, Poll};
use std::time::{Duration, Instant};
use tokio::sync::{broadcast, watch};
use tokio_stream::wrappers::errors::BroadcastStreamRecvError;
use tokio_stream::wrappers::{BroadcastStream, WatchStream};
use tokio_stream::{Stream, StreamExt};

pub use crate::types::SupportState;

/// A decoded metric value together with the provenance needed to reason about freshness.
///
/// Multiple MAVLink message types can carry the same logical metric (e.g. altitude
/// appears in both `GLOBAL_POSITION_INT` and `VFR_HUD`). `source` records which
/// message produced this sample so callers can filter or prefer a specific source.
///
/// `vehicle_time` is `None` when the autopilot's message did not carry a usable
/// timestamp (e.g. the field was zero or outside the plausible boot-time range).
/// Use `received_at` as a fallback for staleness checks in that case.
#[derive(Clone, Debug, PartialEq)]
pub struct MetricSample<T> {
    pub value: T,
    /// Which MAVLink message type produced this sample.
    pub source: TelemetryMessageKind,
    /// Vehicle boot-relative timestamp extracted from the MAVLink message, if available.
    pub vehicle_time: Option<VehicleTimestamp>,
    /// Wall-clock time at which the event loop received this message.
    pub received_at: Instant,
}

/// A raw MAVLink message value together with receipt timestamps.
///
/// Unlike [`MetricSample`], `MessageSample` does not carry a `source` field because
/// each `MessageHandle` is bound to exactly one MAVLink message type.
///
/// `vehicle_time` is `None` when the message's timestamp field was absent or zero.
#[derive(Clone, Debug, PartialEq)]
pub struct MessageSample<M> {
    pub value: M,
    /// Wall-clock time at which the event loop received this message.
    pub received_at: Instant,
    /// Vehicle boot-relative timestamp extracted from the MAVLink message, if available.
    pub vehicle_time: Option<VehicleTimestamp>,
}

struct WatchStore<T: Clone + Send + Sync + 'static> {
    tx: watch::Sender<Option<T>>,
}

struct BroadcastStore<T: Clone + Send + Sync + 'static> {
    tx: broadcast::Sender<T>,
    latest: Arc<Mutex<Option<T>>>,
}

enum ObservationBacking<T: Clone + Send + Sync + 'static> {
    Watch(WatchStore<T>),
    Broadcast(BroadcastStore<T>),
}

/// Read side of a reactive observation channel.
///
/// Observations are the primary mechanism for telemetry, vehicle state, and domain
/// updates. Each handle wraps one of two backing strategies:
///
/// - **Watch** — coalesces updates; [`latest()`] always returns the most recent value
///   (which may be stale if the vehicle is silent). Callers that need every change
///   should use [`subscribe()`] or [`wait()`] rather than polling [`latest()`].
///
/// - **Broadcast** — preserves every event; no coalescing occurs. [`latest()`]
///   returns the value of the last published event, but [`subscribe()`] only receives
///   events published *after* the subscription was created. Use this for status-text
///   messages and other discrete events where history matters.
///
/// [`wait()`] blocks until the first value is available (or returns immediately if one
/// already exists). It returns [`VehicleError::Disconnected`] when the writer is
/// dropped, signalling vehicle disconnection.
///
/// [`subscribe()`] returns an [`ObservationSubscription`] that implements `Stream<Item = T>`.
/// The stream terminates (yields `None`) when the writer is dropped.
///
/// [`latest()`]: ObservationHandle::latest
/// [`wait()`]: ObservationHandle::wait
/// [`subscribe()`]: ObservationHandle::subscribe
#[derive(Clone)]
pub struct ObservationHandle<T: Clone + Send + Sync + 'static> {
    backing: Arc<ObservationBacking<T>>,
    close_rx: watch::Receiver<bool>,
}

/// Generic observation writer paired with an [`ObservationHandle`].
#[derive(Clone)]
pub struct ObservationWriter<T: Clone + Send + Sync + 'static> {
    backing: Arc<ObservationBacking<T>>,
    close_notifier: Arc<CloseNotifier>,
}

struct CloseNotifier {
    close_tx: watch::Sender<bool>,
}

impl CloseNotifier {
    fn close(&self) {
        let _ = self.close_tx.send(true);
    }
}

impl Drop for CloseNotifier {
    fn drop(&mut self) {
        let _ = self.close_tx.send(true);
    }
}

enum SubscriptionBacking<T: Clone + Send + Sync + 'static> {
    Watch { stream: WatchStream<Option<T>> },
    Broadcast { stream: BroadcastStream<T> },
}

/// A `Stream<Item = T>` created from an [`ObservationHandle`].
///
/// The stream terminates (yields `None`) when the originating [`ObservationWriter`] is
/// dropped. This signals vehicle disconnection and lets callers break out of
/// `while let Some(value) = sub.recv().await` loops cleanly.
///
/// For watch-backed observations the stream replays the current value on creation and
/// then emits each subsequent change. For broadcast-backed observations only events
/// published *after* subscription creation are delivered; earlier events are not
/// replayed. If the channel is lagged (the receiver falls too far behind the sender)
/// intermediate events are silently dropped and the stream continues from the next
/// available item.
pub struct ObservationSubscription<T: Clone + Send + Sync + 'static> {
    backing: SubscriptionBacking<T>,
    close_stream: WatchStream<bool>,
    disconnect_emitted: bool,
}

impl<T: Clone + Send + Sync + 'static> ObservationHandle<T> {
    /// Create a coalescing (watch-backed) observation pair.
    ///
    /// Use for metrics that should always expose the most recent value — telemetry
    /// readings, vehicle state, etc. Only the latest published value is retained;
    /// rapid updates between reads are collapsed into the last one.
    pub fn watch() -> (ObservationWriter<T>, Self) {
        let (tx, _) = watch::channel(None::<T>);
        let (close_tx, close_rx) = watch::channel(false);
        let backing = Arc::new(ObservationBacking::Watch(WatchStore { tx }));
        let close_notifier = Arc::new(CloseNotifier { close_tx });

        (
            ObservationWriter {
                backing: backing.clone(),
                close_notifier,
            },
            Self { backing, close_rx },
        )
    }

    /// Create a broadcast-backed observation pair.
    ///
    /// Use for discrete events where every occurrence matters — status-text messages,
    /// calibration reports, etc. All active subscriptions receive every event
    /// independently. `capacity` is the broadcast channel buffer size; slow receivers
    /// that fall behind by more than `capacity` items will skip the intermediate events.
    pub fn broadcast(capacity: usize) -> (ObservationWriter<T>, Self) {
        let (tx, _) = broadcast::channel(capacity.max(1));
        let (close_tx, close_rx) = watch::channel(false);
        let backing = Arc::new(ObservationBacking::Broadcast(BroadcastStore {
            tx,
            latest: Arc::new(Mutex::new(None)),
        }));
        let close_notifier = Arc::new(CloseNotifier { close_tx });

        (
            ObservationWriter {
                backing: backing.clone(),
                close_notifier,
            },
            Self { backing, close_rx },
        )
    }

    pub fn latest(&self) -> Option<T> {
        match self.backing.as_ref() {
            ObservationBacking::Watch(store) => store.tx.subscribe().borrow().clone(),
            ObservationBacking::Broadcast(store) => {
                store.latest.lock().ok().and_then(|v| v.clone())
            }
        }
    }

    pub async fn wait(&self) -> Result<T, VehicleError> {
        if *self.close_rx.borrow() {
            return Err(VehicleError::Disconnected);
        }

        if let Some(value) = self.latest() {
            return Ok(value);
        }

        match self.backing.as_ref() {
            ObservationBacking::Watch(store) => {
                let mut value_rx = store.tx.subscribe();
                let mut close_rx = self.close_rx.clone();

                if let Some(value) = value_rx.borrow().clone() {
                    return Ok(value);
                }

                loop {
                    tokio::select! {
                        changed = value_rx.changed() => {
                            changed.map_err(|_| VehicleError::Disconnected)?;
                            if let Some(value) = value_rx.borrow_and_update().clone() {
                                return Ok(value);
                            }
                        }
                        changed = close_rx.changed() => {
                            if changed.is_err() || *close_rx.borrow_and_update() {
                                return Err(VehicleError::Disconnected);
                            }
                        }
                    }
                }
            }
            ObservationBacking::Broadcast(store) => {
                let mut events_rx = store.tx.subscribe();
                let mut close_rx = self.close_rx.clone();

                if let Some(value) = self.latest() {
                    return Ok(value);
                }

                loop {
                    tokio::select! {
                        recv = events_rx.recv() => {
                            match recv {
                                Ok(value) => return Ok(value),
                                Err(broadcast::error::RecvError::Lagged(_)) => continue,
                                Err(broadcast::error::RecvError::Closed) => return Err(VehicleError::Disconnected),
                            }
                        }
                        changed = close_rx.changed() => {
                            if changed.is_err() || *close_rx.borrow_and_update() {
                                return Err(VehicleError::Disconnected);
                            }
                        }
                    }
                }
            }
        }
    }

    pub async fn wait_timeout(&self, timeout: Duration) -> Result<T, VehicleError> {
        match tokio::time::timeout(timeout, self.wait()).await {
            Ok(result) => result,
            Err(_) => Err(VehicleError::Timeout("observation wait".into())),
        }
    }

    pub fn subscribe(&self) -> ObservationSubscription<T> {
        match self.backing.as_ref() {
            ObservationBacking::Watch(store) => {
                let value_rx = store.tx.subscribe();
                let close_stream = WatchStream::new(self.close_rx.clone());
                ObservationSubscription {
                    backing: SubscriptionBacking::Watch {
                        stream: WatchStream::new(value_rx),
                    },
                    close_stream,
                    disconnect_emitted: false,
                }
            }
            ObservationBacking::Broadcast(store) => ObservationSubscription {
                backing: SubscriptionBacking::Broadcast {
                    stream: BroadcastStream::new(store.tx.subscribe()),
                },
                close_stream: WatchStream::new(self.close_rx.clone()),
                disconnect_emitted: false,
            },
        }
    }
}

impl<T: Clone + Send + Sync + 'static> ObservationWriter<T> {
    pub fn publish(&self, value: T) -> Result<(), VehicleError> {
        match self.backing.as_ref() {
            ObservationBacking::Watch(store) => {
                store.tx.send_replace(Some(value));
                Ok(())
            }
            ObservationBacking::Broadcast(store) => {
                if let Ok(mut latest) = store.latest.lock() {
                    *latest = Some(value.clone());
                }
                let _ = store.tx.send(value);
                Ok(())
            }
        }
    }

    pub(crate) fn clear(&self) {
        match self.backing.as_ref() {
            ObservationBacking::Watch(store) => {
                store.tx.send_replace(None);
            }
            ObservationBacking::Broadcast(store) => {
                if let Ok(mut latest) = store.latest.lock() {
                    *latest = None;
                }
            }
        }
    }

    pub(crate) fn close(&self) {
        self.close_notifier.close();
    }
}

impl<T: Clone + Send + Sync + 'static> ObservationSubscription<T> {
    pub async fn recv(&mut self) -> Option<T> {
        self.next().await
    }
}

impl<T: Clone + Send + Sync + 'static> Stream for ObservationSubscription<T> {
    type Item = T;

    fn poll_next(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Option<Self::Item>> {
        let this = self.get_mut();

        if this.disconnect_emitted {
            return Poll::Ready(None);
        }

        match Pin::new(&mut this.close_stream).poll_next(cx) {
            Poll::Ready(None) => {
                this.disconnect_emitted = true;
                return Poll::Ready(None);
            }
            Poll::Ready(Some(closed)) => {
                if closed {
                    this.disconnect_emitted = true;
                    return Poll::Ready(None);
                }
            }
            Poll::Pending => {}
        }

        match &mut this.backing {
            SubscriptionBacking::Watch { stream } => loop {
                match Pin::new(&mut *stream).poll_next(cx) {
                    Poll::Ready(Some(Some(value))) => return Poll::Ready(Some(value)),
                    Poll::Ready(Some(None)) => continue,
                    Poll::Ready(None) => {
                        this.disconnect_emitted = true;
                        return Poll::Ready(None);
                    }
                    Poll::Pending => return Poll::Pending,
                }
            },
            SubscriptionBacking::Broadcast { stream } => loop {
                match Pin::new(&mut *stream).poll_next(cx) {
                    Poll::Ready(Some(Ok(value))) => return Poll::Ready(Some(value)),
                    Poll::Ready(Some(Err(BroadcastStreamRecvError::Lagged(_)))) => {
                        continue;
                    }
                    Poll::Ready(None) => {
                        this.disconnect_emitted = true;
                        return Poll::Ready(None);
                    }
                    Poll::Pending => return Poll::Pending,
                }
            },
        }
    }
}

/// A typed [`ObservationHandle`] for a coalesced telemetry metric, paired with a
/// support-state observation.
///
/// Use [`support()`] to check whether the connected vehicle is currently sending this
/// metric at all: a value of [`SupportState::Unsupported`] means the autopilot has not
/// produced the relevant MAVLink messages since connection, so [`wait()`] would block
/// indefinitely.
///
/// [`support()`]: MetricHandle::support
/// [`wait()`]: MetricHandle::wait
#[derive(Clone)]
pub struct MetricHandle<T: Clone + Send + Sync + 'static> {
    observation: ObservationHandle<MetricSample<T>>,
    support: ObservationHandle<SupportState>,
}

impl<T: Clone + Send + Sync + 'static> MetricHandle<T> {
    pub fn new(
        observation: ObservationHandle<MetricSample<T>>,
        support: ObservationHandle<SupportState>,
    ) -> Self {
        Self {
            observation,
            support,
        }
    }

    pub fn latest(&self) -> Option<MetricSample<T>> {
        self.observation.latest()
    }

    pub async fn wait(&self) -> Result<MetricSample<T>, VehicleError> {
        self.observation.wait().await
    }

    pub async fn wait_timeout(&self, timeout: Duration) -> Result<MetricSample<T>, VehicleError> {
        self.observation.wait_timeout(timeout).await
    }

    pub fn subscribe(&self) -> ObservationSubscription<MetricSample<T>> {
        self.observation.subscribe()
    }

    pub fn support(&self) -> ObservationHandle<SupportState> {
        self.support.clone()
    }
}

/// A typed [`ObservationHandle`] for a specific MAVLink message type, paired with a
/// support-state observation.
///
/// Unlike [`MetricHandle`], which coalesces values from multiple message sources,
/// `MessageHandle` is bound to a single MAVLink message type and preserves every
/// received event via broadcast backing. Use [`subscribe()`] when you need an
/// uncoalesced stream of individual messages.
///
/// [`support()`] indicates whether the vehicle has sent this message type since
/// connection. A value of [`SupportState::Unsupported`] means [`wait()`] would block
/// indefinitely.
///
/// [`support()`]: MessageHandle::support
/// [`wait()`]: MessageHandle::wait
/// [`subscribe()`]: MessageHandle::subscribe
#[derive(Clone)]
pub struct MessageHandle<M: Clone + Send + Sync + 'static> {
    observation: ObservationHandle<MessageSample<M>>,
    support: ObservationHandle<SupportState>,
}

impl<M: Clone + Send + Sync + 'static> MessageHandle<M> {
    pub fn new(
        observation: ObservationHandle<MessageSample<M>>,
        support: ObservationHandle<SupportState>,
    ) -> Self {
        Self {
            observation,
            support,
        }
    }

    pub fn latest(&self) -> Option<MessageSample<M>> {
        self.observation.latest()
    }

    pub async fn wait(&self) -> Result<MessageSample<M>, VehicleError> {
        self.observation.wait().await
    }

    pub async fn wait_timeout(&self, timeout: Duration) -> Result<MessageSample<M>, VehicleError> {
        self.observation.wait_timeout(timeout).await
    }

    pub fn subscribe(&self) -> ObservationSubscription<MessageSample<M>> {
        self.observation.subscribe()
    }

    pub fn support(&self) -> ObservationHandle<SupportState> {
        self.support.clone()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use tokio_stream::Stream;

    fn watch_fixture() -> (ObservationWriter<u32>, ObservationHandle<u32>) {
        ObservationHandle::<u32>::watch()
    }

    fn broadcast_fixture(
        capacity: usize,
    ) -> (ObservationWriter<String>, ObservationHandle<String>) {
        ObservationHandle::<String>::broadcast(capacity)
    }

    #[tokio::test]
    async fn watch_lifecycle() {
        let (writer, handle) = watch_fixture();
        assert_eq!(handle.latest(), None);

        writer.publish(42).unwrap();
        assert_eq!(handle.latest(), Some(42));

        writer.publish(99).unwrap();
        assert_eq!(handle.latest(), Some(99));
    }

    #[tokio::test]
    async fn wait_disconnect() {
        let (writer, handle) = watch_fixture();
        let task = tokio::spawn(async move { handle.wait().await });

        tokio::time::sleep(Duration::from_millis(20)).await;
        drop(writer);

        let result = task.await.unwrap();
        assert!(matches!(result, Err(VehicleError::Disconnected)));
    }

    #[tokio::test]
    async fn subscribe_terminates_when_writer_closed() {
        let (writer, handle) = watch_fixture();
        let mut sub = handle.subscribe();

        writer.close();

        assert_eq!(sub.recv().await, None);
    }

    #[tokio::test]
    async fn wait_timeout() {
        let (_writer, handle) = watch_fixture();
        let result = handle.wait_timeout(Duration::from_millis(20)).await;
        assert!(matches!(result, Err(VehicleError::Timeout(_))));
    }

    #[tokio::test]
    async fn wait_timeout_returns_disconnected_over_cached_latest() {
        let (writer, handle) = watch_fixture();
        writer.publish(7).unwrap();
        writer.close();

        let result = handle.wait_timeout(Duration::from_millis(20)).await;
        assert!(matches!(result, Err(VehicleError::Disconnected)));
    }

    #[tokio::test]
    async fn wait_timeout_returns_disconnected_over_cached_latest_after_last_writer_drop() {
        let (writer, handle) = watch_fixture();
        writer.publish(7).unwrap();
        drop(writer);

        let result = handle.wait_timeout(Duration::from_millis(20)).await;
        assert!(matches!(result, Err(VehicleError::Disconnected)));
    }

    #[tokio::test]
    async fn dropping_non_last_writer_clone_does_not_close_observation() {
        let (writer, handle) = watch_fixture();
        let writer_clone = writer.clone();

        drop(writer);
        writer_clone.publish(7).unwrap();

        assert_eq!(handle.wait().await.unwrap(), 7);
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn wait_timeout_returns_disconnected_after_concurrent_final_writer_drop() {
        for _ in 0..64 {
            let (writer, handle) = watch_fixture();
            writer.publish(7).unwrap();

            let writer_a = writer.clone();
            let writer_b = writer;
            let barrier = Arc::new(std::sync::Barrier::new(3));

            let barrier_a = barrier.clone();
            let thread_a = std::thread::spawn(move || {
                barrier_a.wait();
                drop(writer_a);
            });

            let barrier_b = barrier.clone();
            let thread_b = std::thread::spawn(move || {
                barrier_b.wait();
                drop(writer_b);
            });

            barrier.wait();
            thread_a.join().unwrap();
            thread_b.join().unwrap();

            let result = handle.wait_timeout(Duration::from_millis(20)).await;
            assert!(matches!(result, Err(VehicleError::Disconnected)));
        }
    }

    #[tokio::test]
    async fn wait_returns_current_value_immediately() {
        let (writer, handle) = watch_fixture();
        writer.publish(7).unwrap();
        assert_eq!(handle.wait().await.unwrap(), 7);
    }

    #[tokio::test]
    async fn subscribe_watch_emits_current_then_updates() {
        let (writer, handle) = watch_fixture();
        writer.publish(10).unwrap();

        let mut sub = handle.subscribe();
        assert_eq!(sub.recv().await.unwrap(), 10);

        writer.publish(11).unwrap();
        assert_eq!(sub.recv().await.unwrap(), 11);
    }

    #[test]
    fn subscribe_exposes_stream_item_t() {
        fn assert_stream<S: Stream<Item = u32>>(_stream: S) {}

        let (_writer, handle) = watch_fixture();
        assert_stream(handle.subscribe());
    }

    #[tokio::test]
    async fn shared_store() {
        let (writer, handle_a) = watch_fixture();
        let handle_b = handle_a.clone();

        writer.publish(123).unwrap();

        assert_eq!(handle_a.latest(), Some(123));
        assert_eq!(handle_b.latest(), Some(123));
    }

    #[tokio::test]
    async fn broadcast_no_coalesce() {
        let (writer, handle) = broadcast_fixture(16);
        let mut sub = handle.subscribe();

        writer.publish("msg1".to_string()).unwrap();
        writer.publish("msg2".to_string()).unwrap();
        writer.publish("msg3".to_string()).unwrap();

        assert_eq!(sub.recv().await.unwrap(), "msg1");
        assert_eq!(sub.recv().await.unwrap(), "msg2");
        assert_eq!(sub.recv().await.unwrap(), "msg3");
    }

    #[tokio::test]
    async fn broadcast_subscribe_only_future_events() {
        let (writer, handle) = broadcast_fixture(16);

        writer.publish("before".to_string()).unwrap();
        let mut sub = handle.subscribe();
        writer.publish("after".to_string()).unwrap();

        assert_eq!(sub.recv().await.unwrap(), "after");
    }

    #[tokio::test]
    async fn broadcast_wait_returns_latest_immediately() {
        let (writer, handle) = broadcast_fixture(16);

        writer.publish("latest".to_string()).unwrap();
        assert_eq!(handle.wait().await.unwrap(), "latest");
    }

    #[tokio::test]
    async fn broadcast_wait_timeout_returns_disconnected_over_cached_latest_after_last_writer_drop()
    {
        let (writer, handle) = broadcast_fixture(16);

        writer.publish("latest".to_string()).unwrap();
        drop(writer);

        let result = handle.wait_timeout(Duration::from_millis(20)).await;
        assert!(matches!(result, Err(VehicleError::Disconnected)));
    }

    #[tokio::test]
    async fn broadcast_lag_skips_to_retained_values() {
        let (writer, handle) = broadcast_fixture(2);
        let mut sub = handle.subscribe();

        writer.publish("m1".to_string()).unwrap();
        writer.publish("m2".to_string()).unwrap();
        writer.publish("m3".to_string()).unwrap();
        writer.publish("m4".to_string()).unwrap();

        assert_eq!(sub.recv().await.unwrap(), "m3");
        assert_eq!(sub.recv().await.unwrap(), "m4");
    }
}
