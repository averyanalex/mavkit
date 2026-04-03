use super::super::*;
use super::core::{assert_roundtrip, geo_msl, geo_rel_home, geo_terrain};

#[test]
fn nav_waypoint_roundtrip() {
    let original = MissionCommand::from(NavWaypoint {
        position: geo_rel_home(473_977_420, 85_455_970, 42.0),
        hold_time_s: 5.0,
        acceptance_radius_m: 2.0,
        pass_radius_m: 0.5,
        yaw_deg: 180.0,
    });

    assert_roundtrip(
        original,
        (
            NavWaypoint::COMMAND_ID,
            MissionFrame::GlobalRelativeAlt,
            [5.0, 2.0, 0.5, 180.0],
            473_977_420,
            85_455_970,
            42.0,
        ),
    );
}

#[test]
fn nav_script_time_roundtrip() {
    let original = MissionCommand::from(NavScriptTime {
        command: 9,
        timeout_s: 11.0,
        arg1: 1.5,
        arg2: -2.5,
        arg3: 17,
        arg4: -29,
    });

    assert_roundtrip(
        original,
        (
            NavScriptTime::COMMAND_ID,
            MissionFrame::Mission,
            [9.0, 11.0, 1.5, -2.5],
            17,
            -29,
            0.0,
        ),
    );
}

#[test]
fn all_nav_roundtrip() {
    let cases = [
        (
            MissionCommand::from(NavWaypoint {
                position: geo_rel_home(473_977_420, 85_455_970, 42.0),
                hold_time_s: 5.0,
                acceptance_radius_m: 2.0,
                pass_radius_m: 0.5,
                yaw_deg: 180.0,
            }),
            (
                NavWaypoint::COMMAND_ID,
                MissionFrame::GlobalRelativeAlt,
                [5.0, 2.0, 0.5, 180.0],
                473_977_420,
                85_455_970,
                42.0,
            ),
        ),
        (
            MissionCommand::from(NavSplineWaypoint {
                position: geo_msl(473_977_421, 85_455_971, 120.0),
                hold_time_s: 7.0,
            }),
            (
                NavSplineWaypoint::COMMAND_ID,
                MissionFrame::Global,
                [7.0, 0.0, 0.0, 0.0],
                473_977_421,
                85_455_971,
                120.0,
            ),
        ),
        (
            MissionCommand::from(NavArcWaypoint {
                position: geo_terrain(473_977_422, 85_455_972, 18.0),
                arc_angle_deg: 90.0,
                direction: LoiterDirection::CounterClockwise,
            }),
            (
                NavArcWaypoint::COMMAND_ID,
                MissionFrame::GlobalTerrainAlt,
                [-90.0, 0.0, 0.0, 0.0],
                473_977_422,
                85_455_972,
                18.0,
            ),
        ),
        (
            MissionCommand::from(NavTakeoff {
                position: geo_terrain(473_977_423, 85_455_973, 60.0),
                pitch_deg: 12.0,
            }),
            (
                NavTakeoff::COMMAND_ID,
                MissionFrame::GlobalTerrainAlt,
                [12.0, 0.0, 0.0, 0.0],
                473_977_423,
                85_455_973,
                60.0,
            ),
        ),
        (
            MissionCommand::from(NavLand {
                position: geo_msl(473_977_424, 85_455_974, 10.0),
                abort_alt_m: 25.0,
            }),
            (
                NavLand::COMMAND_ID,
                MissionFrame::Global,
                [25.0, 0.0, 0.0, 0.0],
                473_977_424,
                85_455_974,
                10.0,
            ),
        ),
        (
            MissionCommand::from(NavCommand::ReturnToLaunch),
            (20, MissionFrame::Mission, [0.0, 0.0, 0.0, 0.0], 0, 0, 0.0),
        ),
        (
            MissionCommand::from(NavLoiterUnlimited {
                position: geo_rel_home(473_977_425, 85_455_975, 50.0),
                radius_m: 35.0,
                direction: LoiterDirection::CounterClockwise,
            }),
            (
                NavLoiterUnlimited::COMMAND_ID,
                MissionFrame::GlobalRelativeAlt,
                [0.0, 0.0, -35.0, 0.0],
                473_977_425,
                85_455_975,
                50.0,
            ),
        ),
        (
            MissionCommand::from(NavLoiterTurns {
                position: geo_rel_home(473_977_426, 85_455_976, 55.0),
                turns: 2.5,
                radius_m: 20.0,
                direction: LoiterDirection::Clockwise,
                exit_xtrack: true,
            }),
            (
                NavLoiterTurns::COMMAND_ID,
                MissionFrame::GlobalRelativeAlt,
                [2.5, 0.0, 20.0, 1.0],
                473_977_426,
                85_455_976,
                55.0,
            ),
        ),
        (
            MissionCommand::from(NavLoiterTime {
                position: geo_msl(473_977_427, 85_455_977, 65.0),
                time_s: 15.0,
                direction: LoiterDirection::Clockwise,
                exit_xtrack: false,
            }),
            (
                NavLoiterTime::COMMAND_ID,
                MissionFrame::Global,
                [15.0, 0.0, 0.0, 0.0],
                473_977_427,
                85_455_977,
                65.0,
            ),
        ),
        (
            MissionCommand::from(NavLoiterToAlt {
                position: geo_terrain(473_977_428, 85_455_978, 75.0),
                radius_m: 40.0,
                direction: LoiterDirection::CounterClockwise,
                exit_xtrack: true,
            }),
            (
                NavLoiterToAlt::COMMAND_ID,
                MissionFrame::GlobalTerrainAlt,
                [0.0, -40.0, 0.0, 1.0],
                473_977_428,
                85_455_978,
                75.0,
            ),
        ),
        (
            MissionCommand::from(NavContinueAndChangeAlt {
                position: geo_terrain(473_977_429, 85_455_979, 80.0),
                action: AltChangeAction::Descend,
            }),
            (
                NavContinueAndChangeAlt::COMMAND_ID,
                MissionFrame::GlobalTerrainAlt,
                [2.0, 0.0, 0.0, 0.0],
                473_977_429,
                85_455_979,
                80.0,
            ),
        ),
        (
            MissionCommand::from(NavDelay {
                seconds: -1.0,
                hour_utc: 6.0,
                min_utc: 7.0,
                sec_utc: 8.0,
            }),
            (
                NavDelay::COMMAND_ID,
                MissionFrame::Mission,
                [-1.0, 6.0, 7.0, 8.0],
                0,
                0,
                0.0,
            ),
        ),
        (
            MissionCommand::from(NavGuidedEnable { enabled: true }),
            (
                NavGuidedEnable::COMMAND_ID,
                MissionFrame::Mission,
                [1.0, 0.0, 0.0, 0.0],
                0,
                0,
                0.0,
            ),
        ),
        (
            MissionCommand::from(NavAltitudeWait {
                altitude_m: 1500.0,
                descent_rate_mps: -4.0,
                wiggle_time_s: 3.0,
            }),
            (
                NavAltitudeWait::COMMAND_ID,
                MissionFrame::Mission,
                [1500.0, -4.0, 3.0, 0.0],
                0,
                0,
                0.0,
            ),
        ),
        (
            MissionCommand::from(NavVtolTakeoff {
                position: geo_rel_home(473_977_430, 85_455_980, 90.0),
            }),
            (
                NavVtolTakeoff::COMMAND_ID,
                MissionFrame::GlobalRelativeAlt,
                [0.0, 0.0, 0.0, 0.0],
                473_977_430,
                85_455_980,
                90.0,
            ),
        ),
        (
            MissionCommand::from(NavVtolLand {
                position: geo_msl(473_977_431, 85_455_981, 8.0),
                options: 2,
            }),
            (
                NavVtolLand::COMMAND_ID,
                MissionFrame::Global,
                [2.0, 0.0, 0.0, 0.0],
                473_977_431,
                85_455_981,
                8.0,
            ),
        ),
        (
            MissionCommand::from(NavPayloadPlace {
                position: geo_terrain(473_977_432, 85_455_982, 12.0),
                max_descent_m: 6.0,
            }),
            (
                NavPayloadPlace::COMMAND_ID,
                MissionFrame::GlobalTerrainAlt,
                [6.0, 0.0, 0.0, 0.0],
                473_977_432,
                85_455_982,
                12.0,
            ),
        ),
        (
            MissionCommand::from(NavSetYawSpeed {
                angle_deg: 45.0,
                speed_mps: 3.5,
                relative: true,
            }),
            (
                NavSetYawSpeed::COMMAND_ID,
                MissionFrame::Mission,
                [45.0, 3.5, 1.0, 0.0],
                0,
                0,
                0.0,
            ),
        ),
        (
            MissionCommand::from(NavScriptTime {
                command: 9,
                timeout_s: 11.0,
                arg1: 1.5,
                arg2: -2.5,
                arg3: 17,
                arg4: -29,
            }),
            (
                NavScriptTime::COMMAND_ID,
                MissionFrame::Mission,
                [9.0, 11.0, 1.5, -2.5],
                17,
                -29,
                0.0,
            ),
        ),
        (
            MissionCommand::from(NavAttitudeTime {
                time_s: 12.0,
                roll_deg: 15.0,
                pitch_deg: -5.0,
                yaw_deg: 270.0,
                climb_rate_mps: 3.0,
            }),
            (
                NavAttitudeTime::COMMAND_ID,
                MissionFrame::Mission,
                [12.0, 15.0, -5.0, 270.0],
                3,
                0,
                0.0,
            ),
        ),
    ];

    for (original, expected_wire) in cases {
        assert_roundtrip(original, expected_wire);
    }
}

#[test]
fn nav_script_time_wire_xy_decode_saturates_to_i16() {
    let decoded = NavScriptTime::from_wire(
        MissionFrame::Mission,
        [9.0, 11.0, 1.5, -2.5],
        i32::MAX,
        i32::MIN,
        0.0,
    );

    assert_eq!(decoded.arg3, i16::MAX);
    assert_eq!(decoded.arg4, i16::MIN);
}

#[test]
fn nav_script_time_command_decode_uses_saturating_cast_semantics() {
    let decoded = NavScriptTime::from_wire(MissionFrame::Mission, [9.9, 0.0, 0.0, 0.0], 0, 0, 0.0);

    // Regression: keep historical float->u16 cast behavior (truncate toward zero),
    // not the macro's default rounded u16 decode.
    assert_eq!(decoded.command, 9);
}

#[test]
fn mavkit_command_accepts_path_style_id() {
    use mavkit_macros::mavkit_command;
    use serde::{Deserialize, Serialize};

    mod compat {
        pub const ID: u16 = 42_702;
    }

    fn unit_command_to_wire() -> (MissionFrame, [f32; 4], i32, i32, f32) {
        (MissionFrame::Mission, [0.0, 0.0, 0.0, 0.0], 0, 0, 0.0)
    }

    #[mavkit_command(id = compat::ID, category = Nav)]
    struct PathStyleIdCompat;

    assert_eq!(PathStyleIdCompat::COMMAND_ID, compat::ID);
    let _ = PathStyleIdCompat.into_wire();
}

#[test]
fn nav_attitude_time_climb_rate_encode_rounds_to_nearest_i32() {
    let up = NavAttitudeTime {
        time_s: 0.0,
        roll_deg: 0.0,
        pitch_deg: 0.0,
        yaw_deg: 0.0,
        climb_rate_mps: 2.6,
    }
    .into_wire();
    assert_eq!(up.2, 3);

    let down = NavAttitudeTime {
        time_s: 0.0,
        roll_deg: 0.0,
        pitch_deg: 0.0,
        yaw_deg: 0.0,
        climb_rate_mps: -2.6,
    }
    .into_wire();
    assert_eq!(down.2, -3);
}
