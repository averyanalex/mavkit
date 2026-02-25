"""Print STATUSTEXT messages from the vehicle (Ctrl+C to stop)."""

import asyncio
import os

import mavkit


async def main():
    bind_addr = os.environ.get("MAVKIT_EXAMPLE_UDP_BIND", "0.0.0.0:14550")

    vehicle = await mavkit.Vehicle.connect_udp(bind_addr)
    print("listening for status messages (Ctrl+C to stop)...\n")

    while True:
        msg = await vehicle.wait_statustext()
        if msg is not None:
            print(f"[{msg.severity.name}] {msg.text}")


asyncio.run(main())
