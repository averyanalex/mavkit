use mavkit::{MetricSample, Vehicle};

fn latest_value<T>(sample: Option<MetricSample<T>>) -> Option<T> {
    sample.map(|sample| sample.value)
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let bind_addr =
        std::env::var("MAVKIT_EXAMPLE_UDP_BIND").unwrap_or_else(|_| "0.0.0.0:14550".to_string());

    let vehicle = Vehicle::connect_udp(&bind_addr).await?;
    let telemetry = vehicle.telemetry();
    let position_handle = telemetry.position().global();
    let groundspeed_handle = telemetry.position().groundspeed_mps();
    let airspeed_handle = telemetry.position().airspeed_mps();
    let climb_handle = telemetry.position().climb_rate_mps();
    let heading_handle = telemetry.position().heading_deg();
    let attitude_handle = telemetry.attitude().euler();
    let battery_voltage_handle = telemetry.battery().voltage_v();
    let battery_current_handle = telemetry.battery().current_a();
    let battery_remaining_handle = telemetry.battery().remaining_pct();
    let gps_quality_handle = telemetry.gps().quality();

    println!("connected, streaming telemetry (Ctrl+C to stop)...\n");

    loop {
        let position = position_handle.wait().await?.value;
        let attitude = latest_value(attitude_handle.latest());
        let groundspeed = latest_value(groundspeed_handle.latest());
        let airspeed = latest_value(airspeed_handle.latest());
        let climb_rate = latest_value(climb_handle.latest());
        let heading = latest_value(heading_handle.latest());
        let battery_voltage = latest_value(battery_voltage_handle.latest());
        let battery_current = latest_value(battery_current_handle.latest());
        let battery_remaining = latest_value(battery_remaining_handle.latest());
        let gps_quality = latest_value(gps_quality_handle.latest());

        print!("\x1b[2J\x1b[H"); // clear terminal

        println!("=== Position ===");
        println!("  lat: {:>12.7}°", position.latitude_deg);
        println!("  lon: {:>12.7}°", position.longitude_deg);
        println!("  alt: {:>8.1} m", position.relative_alt_m);

        println!("\n=== Movement ===");
        println!("  speed:    {:>6.1} m/s", groundspeed.unwrap_or_default());
        println!("  airspeed: {:>6.1} m/s", airspeed.unwrap_or_default());
        println!("  climb:    {:>6.1} m/s", climb_rate.unwrap_or_default());
        println!("  heading:  {:>6.1}°", heading.unwrap_or_default());

        println!("\n=== Attitude ===");
        println!(
            "  roll:  {:>7.2}°",
            attitude.as_ref().map_or(0.0, |value| value.roll_deg)
        );
        println!(
            "  pitch: {:>7.2}°",
            attitude.as_ref().map_or(0.0, |value| value.pitch_deg)
        );
        println!(
            "  yaw:   {:>7.2}°",
            attitude.as_ref().map_or(0.0, |value| value.yaw_deg)
        );

        println!("\n=== Battery ===");
        println!("  voltage: {:>6.2} V", battery_voltage.unwrap_or_default());
        println!("  current: {:>6.2} A", battery_current.unwrap_or_default());
        println!(
            "  remaining: {:>3.0}%",
            battery_remaining.unwrap_or_default()
        );

        println!("\n=== GPS ===");
        println!(
            "  fix:        {:?}",
            gps_quality.as_ref().map(|value| value.fix_type)
        );
        println!(
            "  satellites: {}",
            gps_quality
                .as_ref()
                .and_then(|value| value.satellites)
                .map_or("-".to_string(), |s| s.to_string())
        );
        println!(
            "  hdop:       {:.2}",
            gps_quality
                .as_ref()
                .and_then(|value| value.hdop)
                .unwrap_or(0.0)
        );
    }
}
