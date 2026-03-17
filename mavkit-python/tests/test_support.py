import mavkit


class TestSupportBindings:
    def test_support_handle_exposes_capability_accessors(self):
        support_handle = getattr(mavkit, "SupportHandle")
        for name in (
            "command_int",
            "ftp",
            "terrain",
            "mission_fence",
            "mission_rally",
            "ardupilot",
        ):
            assert hasattr(support_handle, name)
            assert callable(getattr(support_handle, name))

    def test_support_wrapper_classes_are_exported(self):
        for name in ("SupportState", "SupportStateHandle", "SupportStateSubscription"):
            assert hasattr(mavkit, name)

    def test_support_state_enum_members_are_available(self):
        support_state = getattr(mavkit, "SupportState")
        assert support_state.Unknown != support_state.Supported
        assert support_state.Unsupported != support_state.Supported
