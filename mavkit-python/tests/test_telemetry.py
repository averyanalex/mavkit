import mavkit


class TestTelemetryBindings:
    def test_telemetry_handle_exposes_namespace_accessors(self):
        for name in (
            "position",
            "attitude",
            "battery",
            "gps",
            "navigation",
            "terrain",
            "rc",
            "actuators",
            "messages",
            "armed",
            "sensor_health",
            "home",
            "origin",
        ):
            assert hasattr(mavkit.TelemetryHandle, name)
            assert callable(getattr(mavkit.TelemetryHandle, name))

    def test_telemetry_wrapper_classes_are_exported(self):
        for name in (
            "TelemetryPositionNamespace",
            "TelemetryAttitudeNamespace",
            "TelemetryBatteryNamespace",
            "TelemetryGpsNamespace",
            "TelemetryNavigationNamespace",
            "TelemetryTerrainNamespace",
            "TelemetryRcNamespace",
            "TelemetryActuatorsNamespace",
            "TelemetryMessagesHandle",
            "MetricHandle",
            "PeriodicMessageHandle",
            "EventMessageHandle",
            "MessageHandle",
            "MetricSample",
            "MessageSample",
            "MetricSubscription",
            "MessageSubscription",
            "GlobalPosition",
            "EulerAttitude",
            "GpsQuality",
            "CellVoltages",
            "WaypointProgress",
            "GuidanceState",
            "TerrainClearance",
            "GeoPoint3dMsl",
            "SensorHealthSummary",
            "SensorHealthState",
            "StatusTextEvent",
        ):
            assert hasattr(mavkit, name)

    def test_metric_and_message_handle_methods_match_observation_shapes(self):
        for name in ("latest", "wait", "wait_timeout", "subscribe"):
            assert hasattr(mavkit.MetricHandle, name)
            assert callable(getattr(mavkit.MetricHandle, name))

        for name in (
            "latest",
            "wait",
            "wait_timeout",
            "subscribe",
            "request",
            "set_rate",
        ):
            assert hasattr(mavkit.PeriodicMessageHandle, name)
            assert callable(getattr(mavkit.PeriodicMessageHandle, name))

        for name in ("latest", "wait", "wait_timeout", "subscribe", "request"):
            assert hasattr(mavkit.EventMessageHandle, name)
            assert callable(getattr(mavkit.EventMessageHandle, name))

        for name in ("latest", "wait", "wait_timeout", "subscribe"):
            assert hasattr(mavkit.MessageHandle, name)
            assert callable(getattr(mavkit.MessageHandle, name))

        assert not hasattr(mavkit.EventMessageHandle, "set_rate")
        assert not hasattr(mavkit.MessageHandle, "request")
        assert not hasattr(mavkit.MessageHandle, "set_rate")

    def test_python_safe_global_position_accessor_is_available(self):
        assert hasattr(mavkit.TelemetryPositionNamespace, "global_pos")
        assert callable(mavkit.TelemetryPositionNamespace.global_pos)

    def test_grouped_value_objects_expose_fields_for_value_reading(self):
        global_position = mavkit.GlobalPosition(
            latitude_deg=47.397742,
            longitude_deg=8.545594,
            altitude_msl_m=510.0,
            relative_alt_m=50.0,
        )
        assert global_position.latitude_deg == 47.397742
        assert global_position.longitude_deg == 8.545594
        assert global_position.altitude_msl_m == 510.0
        assert global_position.relative_alt_m == 50.0

        gps_quality = mavkit.GpsQuality(
            fix_type=mavkit.GpsFixType.Fix3d,
            satellites=10,
            hdop=0.7,
        )
        assert gps_quality.fix_type == mavkit.GpsFixType.Fix3d
        assert gps_quality.satellites == 10
        assert gps_quality.hdop == 0.7

        home = mavkit.GeoPoint3dMsl(
            latitude_deg=47.397742,
            longitude_deg=8.545594,
            altitude_msl_m=488.0,
        )
        assert home.altitude_msl_m == 488.0

        health = mavkit.SensorHealthSummary(
            gyro=mavkit.SensorHealthState.Healthy,
            accel=mavkit.SensorHealthState.Healthy,
            mag=mavkit.SensorHealthState.Disabled,
            baro=mavkit.SensorHealthState.Healthy,
            gps=mavkit.SensorHealthState.Healthy,
            airspeed=mavkit.SensorHealthState.NotPresent,
            rc_receiver=mavkit.SensorHealthState.Healthy,
            battery=mavkit.SensorHealthState.Healthy,
            terrain=mavkit.SensorHealthState.NotPresent,
            geofence=mavkit.SensorHealthState.Unhealthy,
        )
        assert health.gyro == mavkit.SensorHealthState.Healthy
        assert health.mag == mavkit.SensorHealthState.Disabled
        assert health.geofence == mavkit.SensorHealthState.Unhealthy

        status_text = mavkit.StatusTextEvent(
            text="EKF variance",
            severity=mavkit.MavSeverity.Warning,
            id=7,
            source_system=1,
            source_component=1,
        )
        assert status_text.text == "EKF variance"
        assert status_text.id == 7
        assert status_text.source_system == 1
