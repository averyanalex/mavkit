import pytest

import mavkit


def nav_waypoint_command(
    *,
    latitude_deg: float = 0.0,
    longitude_deg: float = 0.0,
    altitude_m: float = 0.0,
    frame: mavkit.MissionFrame = mavkit.MissionFrame.GlobalInt,
) -> mavkit.NavWaypoint:
    return mavkit.NavWaypoint(
        latitude_deg=latitude_deg,
        longitude_deg=longitude_deg,
        altitude_m=altitude_m,
        frame=frame,
    )


def raw_command(
    *,
    command: int,
    frame: mavkit.MissionFrame,
    x: int = 0,
    y: int = 0,
    z: float = 0.0,
    param1: float = 0.0,
    param2: float = 0.0,
    param3: float = 0.0,
    param4: float = 0.0,
) -> mavkit.RawMissionCommand:
    return mavkit.RawMissionCommand(
        command=command,
        frame=frame,
        x=x,
        y=y,
        z=z,
        param1=param1,
        param2=param2,
        param3=param3,
        param4=param4,
    )


class TestMissionItemBoundaryValues:
    def test_max_command(self):
        item = mavkit.MissionItem(
            command=raw_command(command=65535, frame=mavkit.MissionFrame.GlobalInt),
        )
        assert item.command == 65535

    def test_large_positive_coordinates(self):
        item = mavkit.MissionItem(
            command=nav_waypoint_command(latitude_deg=90.0, longitude_deg=180.0),
        )
        assert item.x == 900000000
        assert item.y == 1800000000

    def test_large_negative_coordinates(self):
        item = mavkit.MissionItem(
            command=nav_waypoint_command(latitude_deg=-90.0, longitude_deg=-180.0),
        )
        assert item.x == -900000000
        assert item.y == -1800000000

    def test_float_precision_z(self):
        item = mavkit.MissionItem(
            command=nav_waypoint_command(altitude_m=123.456),
        )
        assert item.z == pytest.approx(123.456, rel=1e-3)

    def test_float_precision_params(self):
        item = mavkit.MissionItem(
            command=raw_command(
                command=16,
                frame=mavkit.MissionFrame.GlobalInt,
                param1=0.001,
                param2=-999.99,
                param3=1e6,
                param4=0.0,
            ),
        )
        assert item.param1 == pytest.approx(0.001, rel=1e-3)
        assert item.param2 == pytest.approx(-999.99, rel=1e-3)
        assert item.param3 == pytest.approx(1e6, rel=1e-3)
        assert item.param4 == pytest.approx(0.0)


class TestHomePositionBoundaryValues:
    def test_extreme_latitude(self):
        home = mavkit.HomePosition(latitude_deg=90.0, longitude_deg=0.0)
        assert home.latitude_deg == pytest.approx(90.0)

    def test_negative_extreme_latitude(self):
        home = mavkit.HomePosition(latitude_deg=-90.0, longitude_deg=0.0)
        assert home.latitude_deg == pytest.approx(-90.0)

    def test_extreme_longitude(self):
        home = mavkit.HomePosition(latitude_deg=0.0, longitude_deg=180.0)
        assert home.longitude_deg == pytest.approx(180.0)

    def test_negative_extreme_longitude(self):
        home = mavkit.HomePosition(latitude_deg=0.0, longitude_deg=-180.0)
        assert home.longitude_deg == pytest.approx(-180.0)


class TestMissionPlanItemListMarshaling:
    def test_many_items(self):
        items = [mavkit.MissionItem(command=nav_waypoint_command()) for i in range(100)]
        plan = mavkit.MissionPlan(items=items)
        assert len(plan) == 100

    def test_items_are_copies(self):
        items = [mavkit.MissionItem(command=nav_waypoint_command())]
        plan = mavkit.MissionPlan(items=items)
        retrieved = plan.items
        assert len(retrieved) == 1
        retrieved2 = plan.items
        assert len(retrieved2) == 1


class TestEnumRoundtrip:
    def test_mission_frame_roundtrip(self):
        expected_frames = [
            (mavkit.MissionFrame.Mission, mavkit.MissionFrame.Mission),
            (mavkit.MissionFrame.GlobalInt, mavkit.MissionFrame.GlobalInt),
            (
                mavkit.MissionFrame.GlobalRelativeAltInt,
                mavkit.MissionFrame.GlobalRelativeAltInt,
            ),
            (
                mavkit.MissionFrame.GlobalTerrainAltInt,
                mavkit.MissionFrame.GlobalTerrainAltInt,
            ),
            (mavkit.MissionFrame.LocalNed, mavkit.MissionFrame.LocalNed),
            (mavkit.MissionFrame.Other, mavkit.MissionFrame.Other),
        ]
        for frame, expected in expected_frames:
            item = mavkit.MissionItem(
                command=raw_command(command=16, frame=frame),
            )
            assert item.frame == expected


class TestWireRoundtrip:
    def test_mission_wire_roundtrip(self):
        items = [
            mavkit.MissionItem(
                command=nav_waypoint_command(
                    latitude_deg=47.43,
                    longitude_deg=-122.3,
                    altitude_m=100.0,
                    frame=mavkit.MissionFrame.GlobalRelativeAltInt,
                ),
            ),
        ]
        original = mavkit.MissionPlan(items=items)

        wire_items = mavkit.mission_items_for_upload(original)
        assert len(wire_items) == 2

        reconstructed = mavkit.mission_plan_from_download(wire_items)

        assert not hasattr(reconstructed, "home")
        assert len(reconstructed.items) == 1
        assert reconstructed.items[0].command == 16
        assert reconstructed.items[0].x == items[0].x
        assert reconstructed.items[0].y == items[0].y
        assert reconstructed.items[0].z == pytest.approx(items[0].z)


class TestTypedMissionCommandMarshaling:
    def test_nav_return_to_launch_marshals_as_fieldless_nav_command(self):
        item = mavkit.MissionItem(command=mavkit.NavReturnToLaunch())
        assert item.command == 20
        assert item.frame == mavkit.MissionFrame.Mission
        assert item.x == 0
        assert item.y == 0
        assert item.z == pytest.approx(0.0)

    def test_do_change_speed_string_mapping_roundtrip(self):
        command = mavkit.DoChangeSpeed(
            speed_mps=6.0,
            throttle_pct=42.0,
            speed_type="groundspeed",
        )
        item = mavkit.MissionItem(command=command)
        assert command.speed_type == "groundspeed"
        assert item.command == 178
        assert item.param1 == pytest.approx(1.0)
        assert item.param2 == pytest.approx(6.0)
        assert item.param3 == pytest.approx(42.0)

    def test_cond_yaw_direction_and_relative_flags(self):
        command = mavkit.CondYaw(
            angle_deg=30.0,
            turn_rate_dps=15.0,
            direction="clockwise",
            relative=False,
        )
        item = mavkit.MissionItem(command=command)
        assert command.direction == "clockwise"
        assert item.command == 115
        assert item.param1 == pytest.approx(30.0)
        assert item.param2 == pytest.approx(15.0)
        assert item.param3 == pytest.approx(1.0)
        assert item.param4 == pytest.approx(0.0)

    def test_geo_point3d_helper_preserves_frame_semantics(self):
        item = mavkit.MissionItem(
            command=mavkit.NavWaypoint.from_point(
                position=mavkit.GeoPoint3d.rel_home(
                    latitude_deg=-12.34,
                    longitude_deg=56.78,
                    relative_alt_m=9.5,
                )
            ),
        )
        assert item.frame == mavkit.MissionFrame.GlobalRelativeAltInt
        assert item.x == int(-12.34 * 1e7)
        assert item.y == int(56.78 * 1e7)
        assert item.z == pytest.approx(9.5)

    def test_nav_land_and_nav_loiter_time_support_typed_nav_family(self):
        land_item = mavkit.MissionItem(
            command=mavkit.NavLand.from_point(
                position=mavkit.GeoPoint3d.msl(
                    latitude_deg=47.7,
                    longitude_deg=-122.4,
                    altitude_msl_m=30.0,
                ),
                abort_alt_m=14.0,
            ),
        )
        assert land_item.command == 21
        assert land_item.frame == mavkit.MissionFrame.GlobalInt
        assert land_item.param1 == pytest.approx(14.0)

        loiter_item = mavkit.MissionItem(
            command=mavkit.NavLoiterTime(
                latitude_deg=47.71,
                longitude_deg=-122.41,
                altitude_m=45.0,
                frame=mavkit.MissionFrame.GlobalRelativeAltInt,
                time_s=12.0,
                direction="counter_clockwise",
                exit_xtrack=True,
            ),
        )
        assert loiter_item.command == 19
        assert loiter_item.param1 == pytest.approx(12.0)
        assert loiter_item.param4 == pytest.approx(1.0)

    def test_nav_guided_enable_and_do_fieldless_command_marshaling(self):
        guided_item = mavkit.MissionItem(
            command=mavkit.NavGuidedEnable(enabled=False),
        )
        assert guided_item.command == 92
        assert guided_item.frame == mavkit.MissionFrame.Mission
        assert guided_item.param1 == pytest.approx(0.0)

        roi_none_item = mavkit.MissionItem(command=mavkit.DoSetRoiNone())
        assert roi_none_item.command == 197
        assert roi_none_item.frame == mavkit.MissionFrame.Mission
        assert roi_none_item.param1 == pytest.approx(0.0)

    def test_do_and_condition_representative_typed_marshaling(self):
        set_home_item = mavkit.MissionItem(
            command=mavkit.DoSetHome(
                latitude_deg=47.72,
                longitude_deg=-122.42,
                altitude_m=18.0,
                frame=mavkit.MissionFrame.GlobalTerrainAltInt,
                use_current=True,
            ),
        )
        assert set_home_item.command == 179
        assert set_home_item.frame == mavkit.MissionFrame.GlobalTerrainAltInt
        assert set_home_item.param1 == pytest.approx(1.0)

        relay_item = mavkit.MissionItem(
            command=mavkit.DoSetRelay(number=5, state=True),
        )
        assert relay_item.command == 181
        assert relay_item.param1 == pytest.approx(5.0)
        assert relay_item.param2 == pytest.approx(1.0)

        delay_item = mavkit.MissionItem(command=mavkit.CondDelay(delay_s=6.0))
        assert delay_item.command == 112
        assert delay_item.param1 == pytest.approx(6.0)

        distance_item = mavkit.MissionItem(
            command=mavkit.CondDistance(distance_m=33.0),
        )
        assert distance_item.command == 114
        assert distance_item.param1 == pytest.approx(33.0)

    def test_extended_nav_family_marshaling(self):
        loiter_turns_item = mavkit.MissionItem(
            command=mavkit.NavLoiterTurns(
                latitude_deg=47.0,
                longitude_deg=-122.0,
                altitude_m=30.0,
                turns=2.5,
                radius_m=20.0,
                direction="counter_clockwise",
                exit_xtrack=True,
            ),
        )
        assert loiter_turns_item.command == 18
        assert loiter_turns_item.param1 == pytest.approx(2.5)
        assert loiter_turns_item.param3 == pytest.approx(-20.0)
        assert loiter_turns_item.param4 == pytest.approx(1.0)

        continue_alt_item = mavkit.MissionItem(
            command=mavkit.NavContinueAndChangeAlt(
                latitude_deg=47.1,
                longitude_deg=-122.1,
                altitude_m=31.0,
                action="descend",
            ),
        )
        assert continue_alt_item.command == 30
        assert continue_alt_item.param1 == pytest.approx(2.0)

        script_time_item = mavkit.MissionItem(
            command=mavkit.NavScriptTime(
                command=12,
                timeout_s=3.0,
                arg1=1.0,
                arg2=2.0,
                arg3=7,
                arg4=-5,
            ),
        )
        assert script_time_item.command == 42702
        assert script_time_item.param1 == pytest.approx(12.0)
        assert script_time_item.x == 7
        assert script_time_item.y == -5

    def test_extended_do_family_marshaling(self):
        jump_item = mavkit.MissionItem(
            command=mavkit.DoJump(target_index=4, repeat_count=2)
        )
        assert jump_item.command == 177
        assert jump_item.param1 == pytest.approx(4.0)
        assert jump_item.param2 == pytest.approx(2.0)

        servo_item = mavkit.MissionItem(command=mavkit.DoSetServo(channel=9, pwm=1550))
        assert servo_item.command == 183
        assert servo_item.param1 == pytest.approx(9.0)
        assert servo_item.param2 == pytest.approx(1550.0)

        roi_item = mavkit.MissionItem(
            command=mavkit.DoSetRoi(
                latitude_deg=47.2,
                longitude_deg=-122.2,
                altitude_m=20.0,
                mode=3,
            ),
        )
        assert roi_item.command == 201
        assert roi_item.param1 == pytest.approx(3.0)

        camera_item = mavkit.MissionItem(
            command=mavkit.DoSetCameraSource(instance=0, primary=2, secondary=3),
        )
        assert camera_item.command == 534
        assert camera_item.param1 == pytest.approx(0.0)
        assert camera_item.param2 == pytest.approx(2.0)
        assert camera_item.param3 == pytest.approx(3.0)

        limits_item = mavkit.MissionItem(
            command=mavkit.DoGuidedLimits(
                max_time_s=5.0,
                min_alt_m=10.0,
                max_alt_m=50.0,
                max_horiz_m=120.0,
            ),
        )
        assert limits_item.command == 222
        assert limits_item.param1 == pytest.approx(5.0)
        assert limits_item.param2 == pytest.approx(10.0)
        assert limits_item.param3 == pytest.approx(50.0)
        assert limits_item.param4 == pytest.approx(120.0)

    def test_remaining_do_command_parity_marshaling(self):
        mount_item = mavkit.MissionItem(
            command=mavkit.DoMountControl(pitch_deg=1.0, roll_deg=2.0, yaw_deg=3.0),
        )
        assert mount_item.command == 205
        assert mount_item.param1 == pytest.approx(1.0)
        assert mount_item.param2 == pytest.approx(2.0)
        assert mount_item.param3 == pytest.approx(3.0)

        gimbal_item = mavkit.MissionItem(
            command=mavkit.DoGimbalManagerPitchYaw(
                pitch_deg=4.0,
                yaw_deg=5.0,
                pitch_rate_dps=6.0,
                yaw_rate_dps=7.0,
                flags=8,
                gimbal_id=9,
            ),
        )
        assert gimbal_item.command == 1000
        assert gimbal_item.param1 == pytest.approx(4.0)
        assert gimbal_item.param2 == pytest.approx(5.0)
        assert gimbal_item.param3 == pytest.approx(6.0)
        assert gimbal_item.param4 == pytest.approx(7.0)
        assert gimbal_item.x == 8
        assert gimbal_item.z == pytest.approx(9.0)

        trigger_item = mavkit.MissionItem(
            command=mavkit.DoCamTriggerDistance(meters=12.5, trigger_now=True),
        )
        assert trigger_item.command == 206
        assert trigger_item.param1 == pytest.approx(12.5)
        assert trigger_item.param3 == pytest.approx(1.0)

        digicam_config_item = mavkit.MissionItem(
            command=mavkit.DoDigicamConfigure(
                shooting_mode=1,
                shutter_speed=2,
                aperture=3.5,
                iso=400,
                exposure_type=4,
                cmd_id=5,
                cutoff_time=6.0,
            ),
        )
        assert digicam_config_item.command == 202
        assert digicam_config_item.param1 == pytest.approx(1.0)
        assert digicam_config_item.param2 == pytest.approx(2.0)
        assert digicam_config_item.param3 == pytest.approx(3.5)
        assert digicam_config_item.param4 == pytest.approx(400.0)
        assert digicam_config_item.x == 4
        assert digicam_config_item.y == 5
        assert digicam_config_item.z == pytest.approx(6.0)

        digicam_control_item = mavkit.MissionItem(
            command=mavkit.DoDigicamControl(
                session=1,
                zoom_pos=2,
                zoom_step=-3,
                focus_lock=4,
                shooting_cmd=5,
                cmd_id=6,
            ),
        )
        assert digicam_control_item.command == 203
        assert digicam_control_item.param1 == pytest.approx(1.0)
        assert digicam_control_item.param2 == pytest.approx(2.0)
        assert digicam_control_item.param3 == pytest.approx(-3.0)
        assert digicam_control_item.param4 == pytest.approx(4.0)
        assert digicam_control_item.x == 5
        assert digicam_control_item.y == 6

        fence_item = mavkit.MissionItem(
            command=mavkit.DoFenceEnable(action="disable_floor"),
        )
        assert fence_item.command == 207
        assert fence_item.param1 == pytest.approx(2.0)

        parachute_item = mavkit.MissionItem(
            command=mavkit.DoParachute(action="release"),
        )
        assert parachute_item.command == 208
        assert parachute_item.param1 == pytest.approx(2.0)

        gripper_item = mavkit.MissionItem(
            command=mavkit.DoGripper(number=2, action="grab"),
        )
        assert gripper_item.command == 211
        assert gripper_item.param1 == pytest.approx(2.0)
        assert gripper_item.param2 == pytest.approx(1.0)

        sprayer_item = mavkit.MissionItem(
            command=mavkit.DoSprayer(enabled=True),
        )
        assert sprayer_item.command == 216
        assert sprayer_item.param1 == pytest.approx(1.0)

        winch_item = mavkit.MissionItem(
            command=mavkit.DoWinch(
                number=1,
                action="rate_control",
                release_length_m=20.0,
                release_rate_mps=3.0,
            ),
        )
        assert winch_item.command == 42600
        assert winch_item.param1 == pytest.approx(1.0)
        assert winch_item.param2 == pytest.approx(2.0)
        assert winch_item.param3 == pytest.approx(20.0)
        assert winch_item.param4 == pytest.approx(3.0)

        engine_item = mavkit.MissionItem(
            command=mavkit.DoEngineControl(
                start=True,
                cold_start=False,
                height_delay_m=15.0,
                allow_disarmed=True,
            ),
        )
        assert engine_item.command == 223
        assert engine_item.param1 == pytest.approx(1.0)
        assert engine_item.param2 == pytest.approx(0.0)
        assert engine_item.param3 == pytest.approx(15.0)
        assert engine_item.param4 == pytest.approx(1.0)

        inverted_item = mavkit.MissionItem(
            command=mavkit.DoInvertedFlight(inverted=True),
        )
        assert inverted_item.command == 210
        assert inverted_item.param1 == pytest.approx(1.0)

        autotune_item = mavkit.MissionItem(
            command=mavkit.DoAutotuneEnable(enabled=True),
        )
        assert autotune_item.command == 212
        assert autotune_item.param1 == pytest.approx(1.0)
