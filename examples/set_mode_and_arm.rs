use mavkit::Vehicle;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let bind_addr =
        std::env::var("MAVKIT_EXAMPLE_UDP_BIND").unwrap_or_else(|_| "0.0.0.0:14550".to_string());
    let mode = std::env::var("MAVKIT_EXAMPLE_MODE").unwrap_or_else(|_| "GUIDED".to_string());

    let vehicle = Vehicle::connect_udp(&bind_addr).await?;
    let current_mode = vehicle.available_modes().current();
    let armed = vehicle.telemetry().armed();

    let before_mode = current_mode.wait().await?;
    let before_armed = armed.wait().await?.value;
    println!("before: mode={} armed={}", before_mode.name, before_armed);

    println!("setting mode to {mode}...");
    vehicle.set_mode_by_name(&mode).await?;
    let confirmed_mode = current_mode
        .latest()
        .ok_or_else(|| std::io::Error::other("mode observation missing after set_mode_by_name"))?;
    println!(
        "mode confirmed by telemetry: {} ({})",
        confirmed_mode.name, confirmed_mode.custom_mode
    );

    println!("sending arm command...");
    vehicle.arm().await?;
    println!(
        "arm command acknowledged. Subscribe to telemetry().armed() before arming if you need a fresh armed update."
    );

    vehicle.disconnect().await?;
    Ok(())
}
