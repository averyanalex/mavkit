use super::RawMessage;
use crate::dialect;
use crate::observation::ObservationSubscription;
use crate::state::LinkState;
use mavlink::MavHeader;
use std::pin::Pin;
use std::task::{Context, Poll};
use tokio_stream::Stream;
use tokio_stream::wrappers::BroadcastStream;
use tokio_stream::wrappers::errors::BroadcastStreamRecvError;

pub(super) struct RawSubscription {
    messages: BroadcastStream<(MavHeader, dialect::MavMessage)>,
    link_state: ObservationSubscription<LinkState>,
    message_id: Option<u32>,
    disconnected: bool,
}

impl RawSubscription {
    pub(super) fn new(
        messages: tokio::sync::broadcast::Receiver<(MavHeader, dialect::MavMessage)>,
        link_state: ObservationSubscription<LinkState>,
        message_id: Option<u32>,
    ) -> Self {
        Self {
            messages: BroadcastStream::new(messages),
            link_state,
            message_id,
            disconnected: false,
        }
    }
}

impl Stream for RawSubscription {
    type Item = RawMessage;

    fn poll_next(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Option<Self::Item>> {
        let this = self.get_mut();

        if this.disconnected {
            return Poll::Ready(None);
        }

        // Keep disconnect termination local to the raw subscription seam so callers can reason
        // about link-state closure separately from message filtering and lag handling.
        loop {
            match Pin::new(&mut this.link_state).poll_next(cx) {
                Poll::Ready(Some(LinkState::Disconnected | LinkState::Error(_)))
                | Poll::Ready(None) => {
                    this.disconnected = true;
                    return Poll::Ready(None);
                }
                Poll::Ready(Some(LinkState::Connecting | LinkState::Connected)) => continue,
                Poll::Pending => break,
            }
        }

        loop {
            match Pin::new(&mut this.messages).poll_next(cx) {
                Poll::Ready(Some(Ok((header, message)))) => {
                    let raw = RawMessage::from_mavlink(header, message);
                    if this
                        .message_id
                        .is_none_or(|expected| expected == raw.message_id)
                    {
                        return Poll::Ready(Some(raw));
                    }
                }
                Poll::Ready(Some(Err(BroadcastStreamRecvError::Lagged(_)))) => continue,
                Poll::Ready(None) => {
                    this.disconnected = true;
                    return Poll::Ready(None);
                }
                Poll::Pending => return Poll::Pending,
            }
        }
    }
}
