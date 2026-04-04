use crate::VehicleError;
use crate::command::{Command, CommandIntPayload, send_command_int_ack};
use crate::dialect::{self, MavCmd};
use crate::observation::ObservationSubscription;
use crate::state::LinkState;
use crate::vehicle::Vehicle;
use mavlink::{MavHeader, MavlinkVersion, Message};
use num_traits::FromPrimitive;
use std::pin::Pin;
use std::task::{Context, Poll};
use std::time::{Duration, Instant};
use tokio_stream::Stream;
use tokio_stream::StreamExt;
use tokio_stream::wrappers::BroadcastStream;
use tokio_stream::wrappers::errors::BroadcastStreamRecvError;

/// Decoded `COMMAND_ACK` payload returned from raw command helpers.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CommandAck {
    pub command: u16,
    pub result: u8,
    pub progress: Option<u8>,
    pub result_param2: Option<i32>,
}

impl CommandAck {
    #[cfg(test)]
    pub(crate) fn from_wire(ack: dialect::COMMAND_ACK_DATA) -> Self {
        Self::from_wire_values(
            ack.command as u16,
            ack.result as u8,
            ack.progress,
            ack.result_param2,
        )
    }

    pub(crate) fn from_wire_values(
        command: u16,
        result: u8,
        progress: u8,
        result_param2: i32,
    ) -> Self {
        Self {
            command,
            result,
            progress: Some(progress),
            result_param2: Some(result_param2),
        }
    }
}

/// Raw MAVLink frame payload with sender identity and receive timestamp.
#[derive(Clone, Debug)]
pub struct RawMessage {
    pub message_id: u32,
    pub system_id: u8,
    pub component_id: u8,
    pub payload: Vec<u8>,
    pub received_at: Instant,
}

impl PartialEq for RawMessage {
    fn eq(&self, other: &Self) -> bool {
        self.message_id == other.message_id
            && self.system_id == other.system_id
            && self.component_id == other.component_id
            && self.payload == other.payload
    }
}

impl RawMessage {
    pub(crate) fn from_mavlink(header: MavHeader, message: dialect::MavMessage) -> Self {
        let mut payload = [0_u8; 255];
        let payload_len = message.ser(MavlinkVersion::V2, &mut payload);

        Self {
            message_id: message.message_id(),
            system_id: header.system_id,
            component_id: header.component_id,
            payload: payload[..payload_len].to_vec(),
            received_at: Instant::now(),
        }
    }

    fn to_mavlink(&self) -> Result<dialect::MavMessage, VehicleError> {
        dialect::MavMessage::parse(MavlinkVersion::V2, self.message_id, &self.payload).map_err(
            |err| {
                VehicleError::InvalidParameter(format!(
                    "invalid raw MAVLink payload for message_id {}: {err}",
                    self.message_id
                ))
            },
        )
    }
}

/// Low-level MAVLink escape hatch for commands and raw message streaming.
///
/// Obtained from [`Vehicle::raw`](crate::Vehicle::raw). All high-level vehicle actions use the
/// typed `Vehicle` API; this handle is intended for one-off protocol exploration, debugging, and
/// capabilities not yet exposed through the typed API.
///
/// # Conflict model
///
/// `command_long` and `command_int` share the same command-ack scope as the typed vehicle
/// commands (e.g., `Vehicle::arm`). Sending a raw command while any typed command is awaiting
/// its ack returns [`VehicleError::OperationConflict`] immediately.
pub struct RawHandle<'a> {
    vehicle: &'a Vehicle,
}

struct RawSubscription {
    messages: BroadcastStream<(MavHeader, dialect::MavMessage)>,
    link_state: ObservationSubscription<LinkState>,
    message_id: Option<u32>,
    disconnected: bool,
}

impl<'a> RawHandle<'a> {
    pub(crate) fn new(vehicle: &'a Vehicle) -> Self {
        Self { vehicle }
    }

    /// Sends a `COMMAND_LONG` and waits for a `COMMAND_ACK`.
    ///
    /// `command` must be a valid `MAV_CMD` numeric value; an unknown value returns
    /// [`VehicleError::InvalidParameter`] immediately without sending anything.
    ///
    /// Returns [`VehicleError::OperationConflict`] if a typed command is already awaiting an ack.
    /// Returns [`VehicleError::Timeout`] if no ack arrives within the configured command timeout.
    pub async fn command_long(
        &self,
        command: u16,
        params: [f32; 7],
    ) -> Result<CommandAck, VehicleError> {
        let command = parse_command(command)?;
        self.vehicle
            .send_command(|reply| Command::RawCommandLongAck {
                command,
                params,
                reply,
            })
            .await
    }

    /// Sends a `COMMAND_INT` and waits for a `COMMAND_ACK`.
    ///
    /// `command` must be a valid `MAV_CMD` numeric value; `frame` must be a valid `MAV_FRAME`
    /// numeric value. Unknown values return [`VehicleError::InvalidParameter`] immediately.
    ///
    /// Returns [`VehicleError::OperationConflict`] if a typed command is already awaiting an ack.
    /// Returns [`VehicleError::Timeout`] if no ack arrives within the configured command timeout.
    #[allow(
        clippy::too_many_arguments,
        reason = "public RawHandle::command_int signature is required by the Task 21 design contract"
    )]
    pub async fn command_int(
        &self,
        command: u16,
        frame: u8,
        current: u8,
        autocontinue: u8,
        params: [f32; 4],
        x: i32,
        y: i32,
        z: f32,
    ) -> Result<CommandAck, VehicleError> {
        let command = parse_command(command)?;
        let frame = parse_frame(frame)?;

        send_command_int_ack(
            self.vehicle.inner.command_tx.clone(),
            CommandIntPayload {
                command,
                frame,
                current,
                autocontinue,
                params,
                x,
                y,
                z,
            },
        )
        .await
    }

    /// Sends `MAV_CMD_REQUEST_MESSAGE` for `message_id` and returns the first matching message.
    ///
    /// Subscribes to the raw message stream before issuing the request to avoid missing the
    /// response. Returns [`VehicleError::Timeout`] if no matching message arrives within
    /// `timeout`.
    pub async fn request_message(
        &self,
        message_id: u32,
        timeout: Duration,
    ) -> Result<RawMessage, VehicleError> {
        let stream = self.subscribe_filtered(message_id);
        tokio::pin!(stream);

        self.vehicle
            .send_command(|reply| Command::RawCommandLong {
                command: MavCmd::MAV_CMD_REQUEST_MESSAGE,
                params: [message_id as f32, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                reply,
            })
            .await?;

        tokio::time::timeout(timeout, async {
            stream.next().await.ok_or(VehicleError::Disconnected)
        })
        .await
        .map_err(|_| VehicleError::Timeout("raw command".into()))?
    }

    /// Requests the vehicle to stream `message_id` at the specified interval.
    ///
    /// `interval_us` is microseconds between messages; 0 disables the stream, -1 requests the
    /// default rate. Sends `MAV_CMD_SET_MESSAGE_INTERVAL` without waiting for an ack.
    pub async fn set_message_interval(
        &self,
        message_id: u32,
        interval_us: i32,
    ) -> Result<(), VehicleError> {
        self.vehicle
            .send_command(|reply| Command::RawCommandLong {
                command: MavCmd::MAV_CMD_SET_MESSAGE_INTERVAL,
                params: [
                    message_id as f32,
                    interval_us as f32,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                ],
                reply,
            })
            .await
    }

    /// Sends a raw MAVLink message to the vehicle without waiting for any ack.
    ///
    /// The `RawMessage` payload is re-parsed before sending to ensure it is a valid MAVLink
    /// message for the active dialect. Returns [`VehicleError::InvalidParameter`] if the payload
    /// cannot be re-parsed.
    pub async fn send(&self, message: RawMessage) -> Result<(), VehicleError> {
        let message = message.to_mavlink()?;
        self.vehicle
            .send_command(|reply| Command::RawSend {
                message: Box::new(message),
                reply,
            })
            .await
    }

    /// Returns a stream of all incoming MAVLink messages as raw frames.
    ///
    /// Messages may be lagged if the consumer is slow — lagged items are silently dropped
    /// by the underlying broadcast channel.
    pub fn subscribe(&self) -> impl Stream<Item = RawMessage> + Send + 'static {
        self.subscription_stream(None)
    }

    /// Returns a stream of incoming MAVLink messages filtered to the given `message_id`.
    ///
    /// Equivalent to [`subscribe`](Self::subscribe) with a client-side filter. Useful when
    /// polling for a specific response without allocating a full message fanout.
    pub fn subscribe_filtered(
        &self,
        message_id: u32,
    ) -> impl Stream<Item = RawMessage> + Send + 'static {
        self.subscription_stream(Some(message_id))
    }

    fn subscription_stream(
        &self,
        message_id: Option<u32>,
    ) -> impl Stream<Item = RawMessage> + Send + 'static {
        RawSubscription {
            messages: BroadcastStream::new(self.vehicle.inner.stores.raw_message_tx.subscribe()),
            link_state: self.vehicle.inner.stores.link_state_observation.subscribe(),
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

fn parse_command(command: u16) -> Result<MavCmd, VehicleError> {
    MavCmd::from_u16(command)
        .ok_or_else(|| VehicleError::InvalidParameter(format!("unknown MAV_CMD value {command}")))
}

fn parse_frame(frame: u8) -> Result<dialect::MavFrame, VehicleError> {
    dialect::MavFrame::from_u8(frame)
        .ok_or_else(|| VehicleError::InvalidParameter(format!("unknown MAV_FRAME value {frame}")))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dialect;
    use crate::test_support::{
        ConnectedVehicleHarness, ConnectedVehicleOptions, command_ack_with, default_header,
    };
    use tokio::time::timeout;

    fn global_position_int_msg() -> dialect::MavMessage {
        dialect::MavMessage::GLOBAL_POSITION_INT(dialect::GLOBAL_POSITION_INT_DATA {
            time_boot_ms: 42,
            lat: 473_977_420,
            lon: 85_455_940,
            alt: 500_000,
            relative_alt: 15_000,
            vx: 0,
            vy: 0,
            vz: 0,
            hdg: 9_000,
        })
    }

    #[tokio::test]
    async fn command_long_ack() {
        let harness = ConnectedVehicleHarness::connect(ConnectedVehicleOptions::default()).await;
        let vehicle = harness.vehicle;
        let msg_tx = harness.msg_tx;
        let sent = harness.sent;

        let command_task = {
            let vehicle = vehicle.clone();
            tokio::spawn(async move {
                vehicle
                    .raw()
                    .command_long(
                        dialect::MavCmd::MAV_CMD_COMPONENT_ARM_DISARM as u16,
                        [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    )
                    .await
            })
        };

        tokio::time::sleep(Duration::from_millis(10)).await;
        msg_tx
            .send((
                default_header(),
                command_ack_with(
                    dialect::MavCmd::MAV_CMD_COMPONENT_ARM_DISARM,
                    dialect::MavResult::MAV_RESULT_ACCEPTED,
                    77,
                    13,
                ),
            ))
            .await
            .expect("command ack should be delivered");

        let ack = command_task
            .await
            .expect("command task should join")
            .expect("raw command_long should succeed");
        assert_eq!(
            ack.command,
            dialect::MavCmd::MAV_CMD_COMPONENT_ARM_DISARM as u16
        );
        assert_eq!(ack.result, dialect::MavResult::MAV_RESULT_ACCEPTED as u8);
        assert_eq!(ack.progress, Some(77));
        assert_eq!(ack.result_param2, Some(13));

        assert!(sent.lock().unwrap().iter().any(|(_, msg)| matches!(
            msg,
        dialect::MavMessage::COMMAND_LONG(data)
            if data.command == dialect::MavCmd::MAV_CMD_COMPONENT_ARM_DISARM
        )));

        vehicle
            .disconnect()
            .await
            .expect("disconnect should succeed");
    }

    #[test]
    fn command_ack_preserves_zero_values() {
        let ack = CommandAck::from_wire(dialect::COMMAND_ACK_DATA {
            command: dialect::MavCmd::MAV_CMD_REQUEST_MESSAGE,
            result: dialect::MavResult::MAV_RESULT_ACCEPTED,
            progress: 0,
            result_param2: 0,
            target_system: 0,
            target_component: 0,
        });

        assert_eq!(ack.progress, Some(0));
        assert_eq!(ack.result_param2, Some(0));
    }

    #[tokio::test]
    async fn send_bypasses_ack() {
        let harness = ConnectedVehicleHarness::connect(ConnectedVehicleOptions::default()).await;
        let vehicle = harness.vehicle;
        let sent = harness.sent;
        let raw_message = RawMessage::from_mavlink(
            default_header(),
            dialect::MavMessage::COMMAND_LONG(dialect::COMMAND_LONG_DATA {
                target_system: 1,
                target_component: 1,
                command: dialect::MavCmd::MAV_CMD_REQUEST_MESSAGE,
                confirmation: 0,
                param1: 33.0,
                param2: 0.0,
                param3: 0.0,
                param4: 0.0,
                param5: 0.0,
                param6: 0.0,
                param7: 0.0,
            }),
        );

        timeout(Duration::from_millis(100), vehicle.raw().send(raw_message))
            .await
            .expect("raw send should not wait for any COMMAND_ACK")
            .expect("raw send should succeed");

        assert!(sent.lock().unwrap().iter().any(|(_, msg)| matches!(
            msg,
        dialect::MavMessage::COMMAND_LONG(data)
            if data.command == dialect::MavCmd::MAV_CMD_REQUEST_MESSAGE && data.param1 == 33.0
        )));

        vehicle
            .disconnect()
            .await
            .expect("disconnect should succeed");
    }

    #[tokio::test]
    async fn subscribe_filtered() {
        let harness = ConnectedVehicleHarness::connect(ConnectedVehicleOptions::default()).await;
        let vehicle = harness.vehicle;
        let msg_tx = harness.msg_tx;
        let stream = vehicle.raw().subscribe_filtered(33);
        tokio::pin!(stream);

        msg_tx
            .send((
                default_header(),
                dialect::MavMessage::HEARTBEAT(dialect::HEARTBEAT_DATA::default()),
            ))
            .await
            .expect("heartbeat should be delivered");
        msg_tx
            .send((default_header(), global_position_int_msg()))
            .await
            .expect("global position int should be delivered");

        let message = timeout(Duration::from_millis(250), stream.next())
            .await
            .expect("filtered subscription should yield a matching message")
            .expect("filtered subscription should stay open");
        assert_eq!(message.message_id, 33);

        let decoded =
            dialect::MavMessage::parse(MavlinkVersion::V2, message.message_id, &message.payload)
                .expect("filtered raw payload should parse back into a typed MAVLink message");
        assert!(matches!(
            decoded,
            dialect::MavMessage::GLOBAL_POSITION_INT(_)
        ));

        vehicle
            .disconnect()
            .await
            .expect("disconnect should succeed");
    }

    #[tokio::test]
    async fn raw_subscriptions_close_on_disconnect() {
        let harness = ConnectedVehicleHarness::connect(ConnectedVehicleOptions::default()).await;
        let vehicle = harness.vehicle;
        let stream = vehicle.raw().subscribe();
        tokio::pin!(stream);

        vehicle
            .disconnect()
            .await
            .expect("disconnect should succeed");

        let next = timeout(Duration::from_millis(250), stream.next())
            .await
            .expect("raw subscription should resolve after disconnect");
        assert!(
            next.is_none(),
            "raw subscription should close on disconnect"
        );
    }

    #[tokio::test]
    async fn command_int_shares_ack_scope_with_typed_commands() {
        let harness = ConnectedVehicleHarness::connect(ConnectedVehicleOptions::default()).await;
        let vehicle = harness.vehicle;
        let msg_tx = harness.msg_tx;

        let arm_task = {
            let vehicle = vehicle.clone();
            tokio::spawn(async move { vehicle.arm().await })
        };

        tokio::time::sleep(Duration::from_millis(10)).await;
        let raw_conflict = vehicle
            .raw()
            .command_int(
                dialect::MavCmd::MAV_CMD_COMPONENT_ARM_DISARM as u16,
                dialect::MavFrame::MAV_FRAME_GLOBAL as u8,
                0,
                0,
                [1.0, 0.0, 0.0, 0.0],
                0,
                0,
                0.0,
            )
            .await;

        assert!(matches!(
            raw_conflict,
            Err(VehicleError::OperationConflict {
                conflicting_domain,
                conflicting_op,
            }) if conflicting_domain == "command" && conflicting_op == "ack_key_in_flight"
        ));

        msg_tx
            .send((
                default_header(),
                command_ack_with(
                    dialect::MavCmd::MAV_CMD_COMPONENT_ARM_DISARM,
                    dialect::MavResult::MAV_RESULT_ACCEPTED,
                    0,
                    0,
                ),
            ))
            .await
            .expect("arm ack should be delivered");

        arm_task
            .await
            .expect("arm task should join")
            .expect("arm should succeed");

        vehicle
            .disconnect()
            .await
            .expect("disconnect should succeed");
    }
}
