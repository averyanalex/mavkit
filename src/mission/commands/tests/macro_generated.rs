use super::super::wire_support::{
    bool_from_param, bool_to_param, mission_command_to_wire, position_command_to_wire,
    position_from_wire, u16_from_param, unit_command_to_wire,
};
use super::super::*;
use crate::geo::{GeoPoint3d, GeoPoint3dRelHome};
use mavkit_macros::mavkit_command;

fn test_speed_type_to_param(speed_type: SpeedType) -> f32 {
    match speed_type {
        SpeedType::Airspeed => 0.0,
        SpeedType::Groundspeed => 1.0,
    }
}

fn test_speed_type_from_param(value: f32) -> SpeedType {
    match value.round() as i32 {
        0 => SpeedType::Airspeed,
        _ => SpeedType::Groundspeed,
    }
}

fn test_wire_x_encode(value: i16) -> i32 {
    i32::from(value) * 10
}

fn test_wire_x_decode(value: i32) -> i16 {
    (value / 10) as i16
}

fn test_wire_y_encode(value: bool) -> i32 {
    if value { 123 } else { -123 }
}

fn test_wire_y_decode(value: i32) -> bool {
    value > 0
}

fn test_wire_z_encode(value: u8) -> f32 {
    f32::from(value) + 0.5
}

fn test_wire_z_decode(value: f32) -> u8 {
    (value - 0.5).round() as u8
}

// --- Position command: struct with #[position] + #[param] fields ---

#[mavkit_command(id = 16, category = Nav)]
pub struct TestNavWaypoint {
    #[position]
    pub position: GeoPoint3d,
    #[param(1)]
    pub hold_time_s: f32,
    #[param(2)]
    pub acceptance_radius_m: f32,
}

#[test]
fn macro_position_command_roundtrip() {
    let cmd = TestNavWaypoint {
        position: GeoPoint3d::RelHome(GeoPoint3dRelHome {
            latitude_deg: 47.3977419,
            longitude_deg: 8.5455939,
            relative_alt_m: 100.0,
        }),
        hold_time_s: 5.0,
        acceptance_radius_m: 10.0,
    };

    let (frame, params, x, y, z) = test_nav_waypoint_to_wire(cmd);

    assert_eq!(frame, MissionFrame::GlobalRelativeAlt);
    assert_eq!(params[0], 5.0);
    assert_eq!(params[1], 10.0);
    assert_eq!(params[2], 0.0);
    assert_eq!(params[3], 0.0);

    let result = test_nav_waypoint_from_wire(frame, params, x, y, z);
    assert_eq!(result.hold_time_s, 5.0);
    assert_eq!(result.acceptance_radius_m, 10.0);
}

// --- Non-position command: params only ---

#[mavkit_command(id = 178, category = Do)]
pub struct TestDoChangeSpeed {
    #[param(1, via = test_speed_type_to_param, from = test_speed_type_from_param)]
    pub speed_type: SpeedType,
    #[param(2)]
    pub speed_mps: f32,
    #[param(3)]
    pub throttle_pct: f32,
}

#[test]
fn macro_non_position_command_with_custom_fn() {
    let cmd = TestDoChangeSpeed {
        speed_type: SpeedType::Groundspeed,
        speed_mps: 15.0,
        throttle_pct: 50.0,
    };

    let (frame, params, x, y, z) = test_do_change_speed_to_wire(cmd);

    assert_eq!(frame, MissionFrame::Mission);
    assert_eq!(params[0], 1.0); // Groundspeed
    assert_eq!(params[1], 15.0);
    assert_eq!(params[2], 50.0);
    assert_eq!(params[3], 0.0);
    assert_eq!(x, 0);
    assert_eq!(y, 0);
    assert_eq!(z, 0.0);

    let result = test_do_change_speed_from_wire(frame, params, x, y, z);
    assert_eq!(result.speed_type, SpeedType::Groundspeed);
    assert_eq!(result.speed_mps, 15.0);
    assert_eq!(result.throttle_pct, 50.0);
}

// --- Non-position command with wire_x and wire_z ---

#[mavkit_command(id = 1000, category = Do)]
pub struct TestDoGimbalPitchYaw {
    #[param(1)]
    pub pitch_deg: f32,
    #[param(2)]
    pub yaw_deg: f32,
    #[wire_x]
    pub flags: u32,
    #[wire_z]
    pub gimbal_id: u8,
}

#[test]
fn macro_non_position_with_wire_x_z() {
    let cmd = TestDoGimbalPitchYaw {
        pitch_deg: -30.0,
        yaw_deg: 45.0,
        flags: 0x1234,
        gimbal_id: 2,
    };

    let (frame, params, x, y, z) = test_do_gimbal_pitch_yaw_to_wire(cmd);

    assert_eq!(frame, MissionFrame::Mission);
    assert_eq!(params[0], -30.0);
    assert_eq!(params[1], 45.0);
    assert_eq!(params[2], 0.0);
    assert_eq!(params[3], 0.0);
    assert_eq!(x, 0x1234);
    assert_eq!(y, 0);
    assert_eq!(z, 2.0);

    let result = test_do_gimbal_pitch_yaw_from_wire(frame, params, x, y, z);
    assert_eq!(result.pitch_deg, -30.0);
    assert_eq!(result.yaw_deg, 45.0);
    assert_eq!(result.flags, 0x1234);
    assert_eq!(result.gimbal_id, 2);
}

#[mavkit_command(id = 1001, category = Do)]
pub struct TestDoCustomWireHooks {
    #[wire_x(via = test_wire_x_encode, from = test_wire_x_decode)]
    pub x_value: i16,
    #[wire_y(via = test_wire_y_encode, from = test_wire_y_decode)]
    pub y_value: bool,
    #[wire_z(via = test_wire_z_encode, from = test_wire_z_decode)]
    pub z_value: u8,
}

#[test]
fn macro_non_position_with_custom_wire_hooks() {
    let cmd = TestDoCustomWireHooks {
        x_value: 7,
        y_value: true,
        z_value: 9,
    };

    let (frame, params, x, y, z) = test_do_custom_wire_hooks_to_wire(cmd);
    assert_eq!(frame, MissionFrame::Mission);
    assert_eq!(params, [0.0, 0.0, 0.0, 0.0]);
    assert_eq!(x, 70);
    assert_eq!(y, 123);
    assert_eq!(z, 9.5);

    let roundtrip = test_do_custom_wire_hooks_from_wire(frame, params, x, y, z);
    assert_eq!(roundtrip.x_value, 7);
    assert!(roundtrip.y_value);
    assert_eq!(roundtrip.z_value, 9);
}

// --- Unit command: no fields ---

#[mavkit_command(id = 20, category = Nav)]
pub struct TestReturnToLaunch;

#[test]
fn macro_unit_command() {
    let (frame, params, x, y, z) = test_return_to_launch_to_wire(TestReturnToLaunch);

    assert_eq!(frame, MissionFrame::Mission);
    assert_eq!(params, [0.0, 0.0, 0.0, 0.0]);
    assert_eq!(x, 0);
    assert_eq!(y, 0);
    assert_eq!(z, 0.0);

    let _result = test_return_to_launch_from_wire(frame, params, x, y, z);
}

// --- Bool param ---

#[mavkit_command(id = 194, category = Do)]
pub struct TestDoSetReverse {
    #[param(1)]
    pub reverse: bool,
}

#[test]
fn macro_bool_param_roundtrip() {
    let cmd = TestDoSetReverse { reverse: true };

    let (frame, params, x, y, z) = test_do_set_reverse_to_wire(cmd);

    assert_eq!(frame, MissionFrame::Mission);
    assert_eq!(params[0], 1.0);

    let result = test_do_set_reverse_from_wire(frame, params, x, y, z);
    assert!(result.reverse);

    // Test false
    let cmd = TestDoSetReverse { reverse: false };
    let (frame, params, x, y, z) = test_do_set_reverse_to_wire(cmd);
    assert_eq!(params[0], 0.0);

    let result = test_do_set_reverse_from_wire(frame, params, x, y, z);
    assert!(!result.reverse);
}

// --- u16 param ---

#[mavkit_command(id = 177, category = Do)]
pub struct TestDoJump {
    #[param(1)]
    pub target_index: u16,
    #[param(2)]
    pub repeat_count: u16,
}

#[test]
fn macro_u16_param_roundtrip() {
    let cmd = TestDoJump {
        target_index: 42,
        repeat_count: 3,
    };

    let (frame, params, x, y, z) = test_do_jump_to_wire(cmd);

    assert_eq!(frame, MissionFrame::Mission);
    assert_eq!(params[0], 42.0);
    assert_eq!(params[1], 3.0);

    let result = test_do_jump_from_wire(frame, params, x, y, z);
    assert_eq!(result.target_index, 42);
    assert_eq!(result.repeat_count, 3);
}

// --- Struct with existing derives preserved ---

#[mavkit_command(id = 178, category = Do)]
#[derive(Copy)]
pub struct TestWithCopy {
    #[param(1)]
    pub value: f32,
}

#[test]
fn macro_preserves_existing_derives() {
    let cmd = TestWithCopy { value: 1.0 };
    let cmd2 = cmd; // Copy works
    let _ = cmd; // Original still usable
    assert_eq!(cmd2.value, 1.0);
}

#[test]
fn macro_generates_command_id_and_inherent_wire_methods() {
    assert_eq!(TestDoJump::COMMAND_ID, 177);

    let cmd = TestDoJump {
        target_index: 7,
        repeat_count: 2,
    };
    let (frame, params, x, y, z) = cmd.into_wire();
    assert_eq!(frame, MissionFrame::Mission);
    assert_eq!(params, [7.0, 2.0, 0.0, 0.0]);
    assert_eq!((x, y, z), (0, 0, 0.0));

    let roundtrip = TestDoJump::from_wire(frame, params, x, y, z);
    assert_eq!(roundtrip.target_index, 7);
    assert_eq!(roundtrip.repeat_count, 2);
}
