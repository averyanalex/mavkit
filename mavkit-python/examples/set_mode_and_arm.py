"""Set flight mode, then send an arm command with truthful ACK semantics."""

import asyncio
import os

import mavkit


async def main():
    bind_addr = os.environ.get("MAVKIT_EXAMPLE_UDP_BIND", "0.0.0.0:14550")
    mode = os.environ.get("MAVKIT_EXAMPLE_MODE", "GUIDED")

    async with await mavkit.Vehicle.connect_udp(bind_addr) as vehicle:
        current_mode_handle = vehicle.available_modes().current()
        armed_handle = vehicle.telemetry().armed()

        before_mode = await current_mode_handle.wait()
        before_armed = await armed_handle.wait()
        print(f"before: mode={before_mode.name} armed={before_armed.value}")

        print(f"setting mode to {mode}...")
        await vehicle.set_mode_by_name(mode)
        confirmed_mode = current_mode_handle.latest()
        if confirmed_mode is None:
            raise RuntimeError("mode observation missing after set_mode_by_name")
        print(
            "mode confirmed by telemetry: "
            f"{confirmed_mode.name} ({confirmed_mode.custom_mode})"
        )

        print("sending arm command...")
        await vehicle.arm()
        print(
            "arm command acknowledged. Subscribe to telemetry().armed() "
            "before arming if you need a fresh armed update."
        )


asyncio.run(main())
