import asyncio
import os

import mavkit

CLEAR = "\x1b[2J\x1b[H"


def latest_value(sample):
    return sample.value if sample is not None else None


async def main():
    bind_addr = os.environ.get("MAVKIT_EXAMPLE_UDP_BIND", "0.0.0.0:14550")

    vehicle = await mavkit.Vehicle.connect_udp(bind_addr)
    telemetry = vehicle.telemetry()
    position_handle = telemetry.position().global_pos()
    groundspeed_handle = telemetry.position().groundspeed_mps()
    airspeed_handle = telemetry.position().airspeed_mps()
    climb_handle = telemetry.position().climb_rate_mps()
    heading_handle = telemetry.position().heading_deg()
    attitude_handle = telemetry.attitude().euler()
    battery_voltage_handle = telemetry.battery().voltage_v()
    battery_current_handle = telemetry.battery().current_a()
    battery_remaining_handle = telemetry.battery().remaining_pct()
    gps_quality_handle = telemetry.gps().quality()

    print("connected, streaming telemetry (Ctrl+C to stop)...\n")

    while True:
        position = (await position_handle.wait()).value
        attitude = latest_value(attitude_handle.latest())
        groundspeed = latest_value(groundspeed_handle.latest())
        airspeed = latest_value(airspeed_handle.latest())
        climb_rate = latest_value(climb_handle.latest())
        heading = latest_value(heading_handle.latest())
        battery_voltage = latest_value(battery_voltage_handle.latest())
        battery_current = latest_value(battery_current_handle.latest())
        battery_remaining = latest_value(battery_remaining_handle.latest())
        gps_quality = latest_value(gps_quality_handle.latest())

        print(CLEAR, end="")

        print("=== Position ===")
        print(f"  lat: {position.latitude_deg:>12.7f}°")
        print(f"  lon: {position.longitude_deg:>12.7f}°")
        print(f"  alt: {position.relative_alt_m:>8.1f} m")

        print("\n=== Movement ===")
        print(f"  speed:    {groundspeed or 0:>6.1f} m/s")
        print(f"  airspeed: {airspeed or 0:>6.1f} m/s")
        print(f"  climb:    {climb_rate or 0:>6.1f} m/s")
        print(f"  heading:  {heading or 0:>6.1f}°")

        print("\n=== Attitude ===")
        print(f"  roll:  {attitude.roll_deg if attitude else 0:>7.2f}°")
        print(f"  pitch: {attitude.pitch_deg if attitude else 0:>7.2f}°")
        print(f"  yaw:   {attitude.yaw_deg if attitude else 0:>7.2f}°")

        print("\n=== Battery ===")
        print(f"  voltage:   {battery_voltage or 0:>6.2f} V")
        print(f"  current:   {battery_current or 0:>6.2f} A")
        print(f"  remaining: {battery_remaining or 0:>3.0f}%")

        print("\n=== GPS ===")
        print(f"  fix:        {gps_quality.fix_type if gps_quality else None}")
        sats = (
            str(gps_quality.satellites)
            if gps_quality and gps_quality.satellites is not None
            else "-"
        )
        print(f"  satellites: {sats}")
        print(
            f"  hdop:       {gps_quality.hdop if gps_quality and gps_quality.hdop is not None else 0:.2f}"
        )


asyncio.run(main())
