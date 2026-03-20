import asyncio
import os

import mavkit


def is_terminal(state: mavkit.LinkState) -> bool:
    return state in (mavkit.LinkState.Disconnected, mavkit.LinkState.Error)


async def main():
    bind_addr = os.environ.get("MAVKIT_EXAMPLE_UDP_BIND", "0.0.0.0:14550")

    async with await mavkit.Vehicle.connect_udp(bind_addr) as vehicle:
        state_handle = vehicle.link().state()
        current = state_handle.latest()

        if current is not None:
            print(f"link state: {current}")
            if is_terminal(current):
                return

        subscription = state_handle.subscribe()
        last_state = current

        while True:
            state = await subscription.recv()
            if state == last_state:
                continue

            print(f"link state: {state}")
            if is_terminal(state):
                return

            last_state = state


asyncio.run(main())
