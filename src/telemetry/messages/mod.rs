mod backing;
mod namespace;
mod request_handles;

#[cfg(test)]
mod tests;

pub(crate) use backing::{
    TelemetryMessageHandles, TelemetryMessageWriters, create_telemetry_message_backing_stores,
};
pub use namespace::MessagesHandle;
pub use request_handles::{EventMessageHandle, PeriodicMessageHandle};
