use super::*;
use crate::VehicleError;
use crate::dialect;
use crate::test_support::{
    ConnectedVehicleHarness, ConnectedVehicleOptions, command_ack_with, default_header,
};
use mavlink::{MavlinkVersion, Message};
use tokio::time::{Duration, timeout};
use tokio_stream::StreamExt;

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
