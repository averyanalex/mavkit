use super::super::*;
use super::core::{assert_roundtrip, geo_msl, geo_rel_home, geo_terrain};

#[test]
fn do_change_speed_roundtrip() {
    let original = MissionCommand::from(DoChangeSpeed {
        speed_type: SpeedType::Groundspeed,
        speed_mps: 5.5,
        throttle_pct: 66.0,
    });

    assert_roundtrip(
        original,
        (
            DoChangeSpeed::COMMAND_ID,
            MissionFrame::Mission,
            [1.0, 5.5, 66.0, 0.0],
            0,
            0,
            0.0,
        ),
    );
}

#[test]
fn do_set_roi_none_roundtrip() {
    assert_roundtrip(
        MissionCommand::from(DoCommand::SetRoiNone),
        (197, MissionFrame::Mission, [0.0, 0.0, 0.0, 0.0], 0, 0, 0.0),
    );
}

#[test]
fn do_set_home_roundtrip_uses_geopoint_frame() {
    let original = MissionCommand::from(DoSetHome {
        position: geo_terrain(473_977_520, 85_456_070, 21.5),
        use_current: false,
    });

    assert_roundtrip(
        original,
        (
            DoSetHome::COMMAND_ID,
            MissionFrame::GlobalTerrainAlt,
            [0.0, 0.0, 0.0, 0.0],
            473_977_520,
            85_456_070,
            21.5,
        ),
    );
}

#[test]
fn all_do_roundtrip() {
    let cases = [
        (
            MissionCommand::from(DoJump {
                target_index: 12,
                repeat_count: 3,
            }),
            (
                DoJump::COMMAND_ID,
                MissionFrame::Mission,
                [12.0, 3.0, 0.0, 0.0],
                0,
                0,
                0.0,
            ),
        ),
        (
            MissionCommand::from(DoJumpTag {
                tag: 44,
                repeat_count: 5,
            }),
            (
                DoJumpTag::COMMAND_ID,
                MissionFrame::Mission,
                [44.0, 5.0, 0.0, 0.0],
                0,
                0,
                0.0,
            ),
        ),
        (
            MissionCommand::from(DoTag { tag: 44 }),
            (
                DoTag::COMMAND_ID,
                MissionFrame::Mission,
                [44.0, 0.0, 0.0, 0.0],
                0,
                0,
                0.0,
            ),
        ),
        (
            MissionCommand::from(DoPauseContinue { pause: true }),
            (
                DoPauseContinue::COMMAND_ID,
                MissionFrame::Mission,
                [0.0, 0.0, 0.0, 0.0],
                0,
                0,
                0.0,
            ),
        ),
        (
            MissionCommand::from(DoChangeSpeed {
                speed_type: SpeedType::Groundspeed,
                speed_mps: 5.5,
                throttle_pct: 66.0,
            }),
            (
                DoChangeSpeed::COMMAND_ID,
                MissionFrame::Mission,
                [1.0, 5.5, 66.0, 0.0],
                0,
                0,
                0.0,
            ),
        ),
        (
            MissionCommand::from(DoSetReverse { reverse: true }),
            (
                DoSetReverse::COMMAND_ID,
                MissionFrame::Mission,
                [1.0, 0.0, 0.0, 0.0],
                0,
                0,
                0.0,
            ),
        ),
        (
            MissionCommand::from(DoSetHome {
                position: geo_msl(473_977_501, 85_456_001, 110.0),
                use_current: false,
            }),
            (
                DoSetHome::COMMAND_ID,
                MissionFrame::Global,
                [0.0, 0.0, 0.0, 0.0],
                473_977_501,
                85_456_001,
                110.0,
            ),
        ),
        (
            MissionCommand::from(DoLandStart {
                position: geo_rel_home(473_977_502, 85_456_002, 25.0),
            }),
            (
                DoLandStart::COMMAND_ID,
                MissionFrame::GlobalRelativeAlt,
                [0.0, 0.0, 0.0, 0.0],
                473_977_502,
                85_456_002,
                25.0,
            ),
        ),
        (
            MissionCommand::from(DoReturnPathStart {
                position: geo_terrain(473_977_503, 85_456_003, 15.0),
            }),
            (
                DoReturnPathStart::COMMAND_ID,
                MissionFrame::GlobalTerrainAlt,
                [0.0, 0.0, 0.0, 0.0],
                473_977_503,
                85_456_003,
                15.0,
            ),
        ),
        (
            MissionCommand::from(DoGoAround {
                position: geo_rel_home(473_977_504, 85_456_004, 35.0),
            }),
            (
                DoGoAround::COMMAND_ID,
                MissionFrame::GlobalRelativeAlt,
                [0.0, 0.0, 0.0, 0.0],
                473_977_504,
                85_456_004,
                35.0,
            ),
        ),
        (
            MissionCommand::from(DoSetRoiLocation {
                position: geo_msl(473_977_505, 85_456_005, 45.0),
            }),
            (
                DoSetRoiLocation::COMMAND_ID,
                MissionFrame::Global,
                [0.0, 0.0, 0.0, 0.0],
                473_977_505,
                85_456_005,
                45.0,
            ),
        ),
        (
            MissionCommand::from(DoCommand::SetRoiNone),
            (197, MissionFrame::Mission, [0.0, 0.0, 0.0, 0.0], 0, 0, 0.0),
        ),
        (
            MissionCommand::from(DoSetRoi {
                mode: 3,
                position: geo_terrain(473_977_506, 85_456_006, 55.0),
            }),
            (
                DoSetRoi::COMMAND_ID,
                MissionFrame::GlobalTerrainAlt,
                [3.0, 0.0, 0.0, 0.0],
                473_977_506,
                85_456_006,
                55.0,
            ),
        ),
        (
            MissionCommand::from(DoMountControl {
                pitch_deg: -10.0,
                roll_deg: 2.5,
                yaw_deg: 180.0,
            }),
            (
                DoMountControl::COMMAND_ID,
                MissionFrame::Mission,
                [-10.0, 2.5, 180.0, 0.0],
                0,
                0,
                0.0,
            ),
        ),
        (
            MissionCommand::from(DoGimbalManagerPitchYaw {
                pitch_deg: -5.0,
                yaw_deg: 90.0,
                pitch_rate_dps: 3.0,
                yaw_rate_dps: 4.0,
                flags: 7,
                gimbal_id: 5,
            }),
            (
                DoGimbalManagerPitchYaw::COMMAND_ID,
                MissionFrame::Mission,
                [-5.0, 90.0, 3.0, 4.0],
                7,
                0,
                5.0,
            ),
        ),
        (
            MissionCommand::from(DoCamTriggerDistance {
                meters: 12.5,
                trigger_now: true,
            }),
            (
                DoCamTriggerDistance::COMMAND_ID,
                MissionFrame::Mission,
                [12.5, 0.0, 1.0, 0.0],
                0,
                0,
                0.0,
            ),
        ),
        (
            MissionCommand::from(DoImageStartCapture {
                instance: 2,
                interval_s: 4.0,
                total_images: 6,
                start_number: 8,
            }),
            (
                DoImageStartCapture::COMMAND_ID,
                MissionFrame::Mission,
                [2.0, 4.0, 6.0, 8.0],
                0,
                0,
                0.0,
            ),
        ),
        (
            MissionCommand::from(DoImageStopCapture { instance: 2 }),
            (
                DoImageStopCapture::COMMAND_ID,
                MissionFrame::Mission,
                [2.0, 0.0, 0.0, 0.0],
                0,
                0,
                0.0,
            ),
        ),
        (
            MissionCommand::from(DoVideoStartCapture { stream_id: 3 }),
            (
                DoVideoStartCapture::COMMAND_ID,
                MissionFrame::Mission,
                [3.0, 0.0, 0.0, 0.0],
                0,
                0,
                0.0,
            ),
        ),
        (
            MissionCommand::from(DoVideoStopCapture { stream_id: 4 }),
            (
                DoVideoStopCapture::COMMAND_ID,
                MissionFrame::Mission,
                [4.0, 0.0, 0.0, 0.0],
                0,
                0,
                0.0,
            ),
        ),
        (
            MissionCommand::from(DoSetCameraZoom {
                zoom_type: 2,
                zoom_value: 55.0,
            }),
            (
                DoSetCameraZoom::COMMAND_ID,
                MissionFrame::Mission,
                [2.0, 55.0, 0.0, 0.0],
                0,
                0,
                0.0,
            ),
        ),
        (
            MissionCommand::from(DoSetCameraFocus {
                focus_type: 3,
                focus_value: 9.0,
            }),
            (
                DoSetCameraFocus::COMMAND_ID,
                MissionFrame::Mission,
                [3.0, 9.0, 0.0, 0.0],
                0,
                0,
                0.0,
            ),
        ),
        (
            MissionCommand::from(DoSetCameraSource {
                instance: 1,
                primary: 2,
                secondary: 3,
            }),
            (
                DoSetCameraSource::COMMAND_ID,
                MissionFrame::Mission,
                [1.0, 2.0, 3.0, 0.0],
                0,
                0,
                0.0,
            ),
        ),
        (
            MissionCommand::from(DoDigicamConfigure {
                shooting_mode: 1,
                shutter_speed: 125,
                aperture: 2.8,
                iso: 200,
                exposure_type: 4,
                cmd_id: 5,
                cutoff_time: 0.7,
            }),
            (
                DoDigicamConfigure::COMMAND_ID,
                MissionFrame::Mission,
                [1.0, 125.0, 2.8, 200.0],
                4,
                5,
                0.7,
            ),
        ),
        (
            MissionCommand::from(DoDigicamControl {
                session: 1,
                zoom_pos: 2,
                zoom_step: -3,
                focus_lock: 4,
                shooting_cmd: 5,
                cmd_id: 6,
            }),
            (
                DoDigicamControl::COMMAND_ID,
                MissionFrame::Mission,
                [1.0, 2.0, -3.0, 4.0],
                5,
                6,
                0.0,
            ),
        ),
        (
            MissionCommand::from(DoSetServo {
                channel: 9,
                pwm: 1500,
            }),
            (
                DoSetServo::COMMAND_ID,
                MissionFrame::Mission,
                [9.0, 1500.0, 0.0, 0.0],
                0,
                0,
                0.0,
            ),
        ),
        (
            MissionCommand::from(DoSetRelay {
                number: 2,
                state: true,
            }),
            (
                DoSetRelay::COMMAND_ID,
                MissionFrame::Mission,
                [2.0, 1.0, 0.0, 0.0],
                0,
                0,
                0.0,
            ),
        ),
        (
            MissionCommand::from(DoRepeatServo {
                channel: 10,
                pwm: 1200,
                count: 4,
                cycle_time_s: 2.0,
            }),
            (
                DoRepeatServo::COMMAND_ID,
                MissionFrame::Mission,
                [10.0, 1200.0, 4.0, 2.0],
                0,
                0,
                0.0,
            ),
        ),
        (
            MissionCommand::from(DoRepeatRelay {
                number: 3,
                count: 5,
                cycle_time_s: 1.5,
            }),
            (
                DoRepeatRelay::COMMAND_ID,
                MissionFrame::Mission,
                [3.0, 5.0, 1.5, 0.0],
                0,
                0,
                0.0,
            ),
        ),
        (
            MissionCommand::from(DoFenceEnable {
                action: FenceAction::Enable,
            }),
            (
                DoFenceEnable::COMMAND_ID,
                MissionFrame::Mission,
                [1.0, 0.0, 0.0, 0.0],
                0,
                0,
                0.0,
            ),
        ),
        (
            MissionCommand::from(DoParachute {
                action: ParachuteAction::Release,
            }),
            (
                DoParachute::COMMAND_ID,
                MissionFrame::Mission,
                [2.0, 0.0, 0.0, 0.0],
                0,
                0,
                0.0,
            ),
        ),
        (
            MissionCommand::from(DoGripper {
                number: 1,
                action: GripperAction::Grab,
            }),
            (
                DoGripper::COMMAND_ID,
                MissionFrame::Mission,
                [1.0, 1.0, 0.0, 0.0],
                0,
                0,
                0.0,
            ),
        ),
        (
            MissionCommand::from(DoSprayer { enabled: true }),
            (
                DoSprayer::COMMAND_ID,
                MissionFrame::Mission,
                [1.0, 0.0, 0.0, 0.0],
                0,
                0,
                0.0,
            ),
        ),
        (
            MissionCommand::from(DoWinch {
                number: 1,
                action: WinchAction::RateControl,
                release_length_m: 12.5,
                release_rate_mps: -1.25,
            }),
            (
                DoWinch::COMMAND_ID,
                MissionFrame::Mission,
                [1.0, 2.0, 12.5, -1.25],
                0,
                0,
                0.0,
            ),
        ),
        (
            MissionCommand::from(DoEngineControl {
                start: true,
                cold_start: false,
                height_delay_m: 6.5,
                allow_disarmed: true,
            }),
            (
                DoEngineControl::COMMAND_ID,
                MissionFrame::Mission,
                [1.0, 0.0, 6.5, 1.0],
                0,
                0,
                0.0,
            ),
        ),
        (
            MissionCommand::from(DoInvertedFlight { inverted: true }),
            (
                DoInvertedFlight::COMMAND_ID,
                MissionFrame::Mission,
                [1.0, 0.0, 0.0, 0.0],
                0,
                0,
                0.0,
            ),
        ),
        (
            MissionCommand::from(DoAutotuneEnable { enabled: true }),
            (
                DoAutotuneEnable::COMMAND_ID,
                MissionFrame::Mission,
                [1.0, 0.0, 0.0, 0.0],
                0,
                0,
                0.0,
            ),
        ),
        (
            MissionCommand::from(DoVtolTransition { target_state: 4 }),
            (
                DoVtolTransition::COMMAND_ID,
                MissionFrame::Mission,
                [4.0, 0.0, 0.0, 0.0],
                0,
                0,
                0.0,
            ),
        ),
        (
            MissionCommand::from(DoGuidedLimits {
                max_time_s: 10.0,
                min_alt_m: 20.0,
                max_alt_m: 30.0,
                max_horiz_m: 40.0,
            }),
            (
                DoGuidedLimits::COMMAND_ID,
                MissionFrame::Mission,
                [10.0, 20.0, 30.0, 40.0],
                0,
                0,
                0.0,
            ),
        ),
        (
            MissionCommand::from(DoSetResumeRepeatDist { distance_m: 18.0 }),
            (
                DoSetResumeRepeatDist::COMMAND_ID,
                MissionFrame::Mission,
                [18.0, 0.0, 0.0, 0.0],
                0,
                0,
                0.0,
            ),
        ),
        (
            MissionCommand::from(DoAuxFunction {
                function: 42,
                switch_pos: 2,
            }),
            (
                DoAuxFunction::COMMAND_ID,
                MissionFrame::Mission,
                [42.0, 2.0, 0.0, 0.0],
                0,
                0,
                0.0,
            ),
        ),
        (
            MissionCommand::from(DoSendScriptMessage {
                id: 7,
                p1: 1.25,
                p2: -2.5,
                p3: 3.75,
            }),
            (
                DoSendScriptMessage::COMMAND_ID,
                MissionFrame::Mission,
                [7.0, 1.25, -2.5, 3.75],
                0,
                0,
                0.0,
            ),
        ),
    ];

    for (original, expected_wire) in cases {
        assert_roundtrip(original, expected_wire);
    }
}
