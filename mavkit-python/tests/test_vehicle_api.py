import mavkit


class TestVehicleApi:
    def test_vehicle_exposes_reboot_to_bootloader(self):
        assert hasattr(mavkit.Vehicle, "reboot_to_bootloader")
