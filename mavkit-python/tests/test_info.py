import mavkit


class TestInfoBindings:
    def test_info_handle_exposes_identity_accessors(self):
        info_handle = getattr(mavkit, "InfoHandle")
        for name in (
            "firmware",
            "hardware",
            "unique_ids",
            "best_unique_id",
            "best_display_id",
            "persistent_identity",
        ):
            assert hasattr(info_handle, name)
            assert callable(getattr(info_handle, name))

    def test_info_wrapper_classes_are_exported(self):
        for name in (
            "FirmwareInfo",
            "HardwareInfo",
            "UniqueIds",
            "PersistentIdentity",
            "FirmwareInfoHandle",
            "FirmwareInfoSubscription",
            "HardwareInfoHandle",
            "HardwareInfoSubscription",
            "UniqueIdsHandle",
            "UniqueIdsSubscription",
            "PersistentIdentityHandle",
            "PersistentIdentitySubscription",
        ):
            assert hasattr(mavkit, name)

    def test_info_value_objects_expose_fields(self):
        firmware_info = getattr(mavkit, "FirmwareInfo")
        hardware_info = getattr(mavkit, "HardwareInfo")
        unique_ids = getattr(mavkit, "UniqueIds")
        persistent_identity = getattr(mavkit, "PersistentIdentity")

        firmware = firmware_info(
            version="1.2.3",
            custom_version=[0xDE, 0xAD],
            git_hash="deadbeef",
            os_version="4.5.6",
        )
        assert firmware.version == "1.2.3"
        assert firmware.custom_version == [0xDE, 0xAD]
        assert firmware.git_hash == "deadbeef"
        assert firmware.os_version == "4.5.6"

        hardware = hardware_info(
            board_vendor_id=42,
            board_product_id=99,
            usb_vendor_id=0x1209,
            usb_product_id=0x5741,
            board_version=77,
        )
        assert hardware.board_vendor_id == 42
        assert hardware.usb_vendor_id == 0x1209
        assert hardware.board_version == 77

        ids = unique_ids(
            hardware_uid=[1, 2, 3, 4],
            uid=0x1234,
            remote_id="RID123",
            board_id="apj-140",
        )
        assert ids.hardware_uid == [1, 2, 3, 4]
        assert ids.uid == 0x1234
        assert ids.remote_id == "RID123"
        assert ids.board_id == "apj-140"

        pending = persistent_identity(state="pending", system_id=1, component_id=2)
        assert pending.state == "pending"
        assert pending.system_id == 1
        assert pending.component_id == 2
        assert pending.canonical_id is None

        ready = persistent_identity(
            state="ready",
            canonical_id="uid64:0123",
            aliases=["uid64:0123", "mcu:abcd"],
        )
        assert ready.state == "ready"
        assert ready.canonical_id == "uid64:0123"
        assert ready.aliases == ["uid64:0123", "mcu:abcd"]
