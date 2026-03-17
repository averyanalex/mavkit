import mavkit


class TestModesBindings:
    def test_modes_handle_exposes_observation_accessors(self):
        modes_handle = getattr(mavkit, "ModesHandle")
        for name in ("support", "catalog", "current"):
            assert hasattr(modes_handle, name)
            assert callable(getattr(modes_handle, name))

    def test_modes_wrapper_classes_are_exported(self):
        for name in (
            "ModeCatalogSource",
            "ModeDescriptor",
            "CurrentModeSource",
            "CurrentMode",
            "ModeCatalogHandle",
            "ModeCatalogSubscription",
            "CurrentModeHandle",
            "CurrentModeSubscription",
        ):
            assert hasattr(mavkit, name)

    def test_modes_value_objects_expose_fields(self):
        mode_descriptor = getattr(mavkit, "ModeDescriptor")
        mode_catalog_source = getattr(mavkit, "ModeCatalogSource")
        current_mode = getattr(mavkit, "CurrentMode")
        current_mode_source = getattr(mavkit, "CurrentModeSource")

        mode = mode_descriptor(
            custom_mode=4,
            name="GUIDED",
            user_selectable=True,
            source=mode_catalog_source.StaticArduPilotTable,
        )
        assert mode.custom_mode == 4
        assert mode.name == "GUIDED"
        assert mode.user_selectable is True
        assert mode.source == mode_catalog_source.StaticArduPilotTable

        current = current_mode(
            custom_mode=6,
            name="RTL",
            intended_custom_mode=7,
            source=current_mode_source.Heartbeat,
        )
        assert current.custom_mode == 6
        assert current.name == "RTL"
        assert current.intended_custom_mode == 7
        assert current.source == current_mode_source.Heartbeat
