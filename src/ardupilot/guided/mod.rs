mod copter;
mod plane;
mod rover;
mod session;
mod sub;

pub use plane::{ArduPlaneGuidedHandle, ArduPlaneKind, ArduPlaneVtolGuidedHandle};
pub use session::ArduGuidedSession;
pub(crate) use session::GuidedLeaseScope;
pub(crate) use session::GuidedSessionInit;

use serde::{Deserialize, Serialize};

/// Vehicle-family discriminator for an active guided session.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ArduGuidedKind {
    Copter,
    Plane,
    Rover,
    Sub,
}

/// Family-specific guided control handle for a session.
#[derive(Debug)]
pub enum GuidedSpecific<'a> {
    Copter(ArduCopterGuidedHandle<'a>),
    Plane(ArduPlaneGuidedHandle<'a>),
    Rover(ArduRoverGuidedHandle<'a>),
    Sub(ArduSubGuidedHandle<'a>),
}

/// Relative climb target used by VTOL and copter takeoff helpers.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct RelativeClimbTarget {
    pub relative_climb_m: f32,
}

/// Horizontal point and depth target for submarine guided goto.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SubGotoDepthTarget {
    pub point: crate::GeoPoint2d,
    pub depth_m: f32,
}

/// Guided control surface for ArduCopter sessions.
#[derive(Debug)]
pub struct ArduCopterGuidedHandle<'a> {
    pub(crate) _session: &'a ArduGuidedSession,
}

/// Guided control surface for ArduRover sessions.
#[derive(Debug)]
pub struct ArduRoverGuidedHandle<'a> {
    pub(crate) _session: &'a ArduGuidedSession,
}

impl<'a> ArduRoverGuidedHandle<'a> {
    pub(crate) fn new(session: &'a ArduGuidedSession) -> Self {
        Self { _session: session }
    }
}

/// Guided control surface for ArduSub sessions.
#[derive(Debug)]
pub struct ArduSubGuidedHandle<'a> {
    pub(crate) _session: &'a ArduGuidedSession,
}

impl<'a> ArduSubGuidedHandle<'a> {
    pub(crate) fn new(session: &'a ArduGuidedSession) -> Self {
        Self { _session: session }
    }
}
