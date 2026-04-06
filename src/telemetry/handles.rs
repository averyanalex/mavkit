//! Thin root for telemetry metric backing stores and grouped accessors.
//!
//! The writer-side backing store creation lives in [`backing`], while the public grouped
//! accessors exposed from [`crate::Vehicle::telemetry`] live in [`namespace`]. This module keeps
//! the stable telemetry handle/type paths intact via re-exports.

mod backing;
mod namespace;

pub(crate) use backing::{
    TelemetryMetricHandles, TelemetryMetricWriters, create_telemetry_backing_stores,
};

pub use namespace::{
    ActuatorsNamespace, AttitudeNamespace, BatteryNamespace, GpsNamespace, NavigationNamespace,
    PositionNamespace, RcNamespace, TelemetryHandle, TerrainNamespace,
};
