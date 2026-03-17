use mavkit::Vehicle;
use mavkit::dialect;
use mavkit::tlog::TlogWriter;
use mavlink::Message;
use mavlink::{MavHeader, MavlinkVersion};
use std::io::BufWriter;
use tokio_stream::StreamExt;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let bind_addr = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "0.0.0.0:14550".to_string());
    let path = std::env::args()
        .nth(2)
        .unwrap_or_else(|| "recording.tlog".to_string());

    println!("connecting to {bind_addr}...");
    let vehicle = Vehicle::connect_udp(&bind_addr).await?;
    println!("connected, recording to {path}");

    let mut stream = vehicle.raw().subscribe();
    let file = std::fs::File::create(&path)?;
    let mut writer = TlogWriter::new(BufWriter::new(file), MavlinkVersion::V2);

    let mut count = 0u64;
    while let Some(raw) = stream.next().await {
        match dialect::MavMessage::parse(MavlinkVersion::V2, raw.message_id, &raw.payload) {
            Ok(message) => {
                let header = MavHeader {
                    sequence: 0,
                    system_id: raw.system_id,
                    component_id: raw.component_id,
                };
                writer.write_now(&header, &message)?;
                count += 1;
                if count.is_multiple_of(100) {
                    writer.flush()?;
                    println!("{count} messages recorded");
                }
            }
            Err(err) => eprintln!("warning: failed to parse message {}: {err}", raw.message_id),
        }
    }

    writer.flush()?;
    println!("{count} messages total");
    Ok(())
}
