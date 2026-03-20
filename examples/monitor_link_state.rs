use mavkit::{LinkState, Vehicle};

fn is_terminal(state: &LinkState) -> bool {
    matches!(state, LinkState::Disconnected | LinkState::Error(_))
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let bind_addr =
        std::env::var("MAVKIT_EXAMPLE_UDP_BIND").unwrap_or_else(|_| "0.0.0.0:14550".to_string());

    let vehicle = Vehicle::connect_udp(&bind_addr).await?;
    let state_handle = vehicle.link().state();

    if let Some(state) = state_handle.latest() {
        println!("link state: {state:?}");
        if is_terminal(&state) {
            return Ok(());
        }
    }

    let mut subscription = state_handle.subscribe();
    let mut last_state = state_handle.latest();

    while let Some(state) = subscription.recv().await {
        if last_state.as_ref() == Some(&state) {
            continue;
        }

        println!("link state: {state:?}");
        if is_terminal(&state) {
            break;
        }

        last_state = Some(state);
    }

    Ok(())
}
