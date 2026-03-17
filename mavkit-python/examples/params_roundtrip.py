"""Download all parameters and round-trip through the param file format."""

import asyncio
import os

import mavkit


async def main():
    bind_addr = os.environ.get("MAVKIT_EXAMPLE_UDP_BIND", "0.0.0.0:14550")

    vehicle = await mavkit.Vehicle.connect_udp(bind_addr)
    params = vehicle.params()
    download = params.download_all()

    store = await download.wait()
    state = params.latest()
    print(
        f"downloaded {len(store)} params (sync={state.sync if state is not None else None})"
    )

    text = mavkit.format_param_file(store)
    parsed = mavkit.parse_param_file(text)
    print(f"formatted+parsed {len(parsed)} params")

    await vehicle.disconnect()


asyncio.run(main())
