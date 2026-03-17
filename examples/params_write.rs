use mavkit::Vehicle;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let bind_addr =
        std::env::var("MAVKIT_EXAMPLE_UDP_BIND").unwrap_or_else(|_| "0.0.0.0:14550".to_string());

    let vehicle = Vehicle::connect_udp(&bind_addr).await?;
    let params = vehicle.params();

    // --- Single parameter write ---
    let param = params.write("WPNAV_SPEED", 500.0).await?;
    println!(
        "wrote {}: requested={} confirmed={} success={}",
        param.name, param.requested_value, param.confirmed_value, param.success
    );

    // --- Batch write ---
    let results = params
        .write_batch(vec![
            ("WPNAV_SPEED".to_string(), 500.0),
            ("WPNAV_SPEED_DN".to_string(), 150.0),
            ("WPNAV_SPEED_UP".to_string(), 250.0),
        ])?
        .wait()
        .await?;

    println!("\nbatch results:");
    for r in &results {
        let status = if r.success { "ok" } else { "MISMATCH" };
        println!(
            "  {} = {} (requested {}) [{}]",
            r.name, r.confirmed_value, r.requested_value, status
        );
    }

    vehicle.disconnect().await?;
    Ok(())
}
