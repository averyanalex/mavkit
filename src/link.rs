use crate::observation::ObservationHandle;
use crate::state::LinkState;
use crate::vehicle::VehicleInner;

#[derive(Clone)]
/// Accessor for connection lifecycle observations.
pub struct LinkHandle<'a> {
    inner: &'a VehicleInner,
}

impl<'a> LinkHandle<'a> {
    pub(crate) fn new(inner: &'a VehicleInner) -> Self {
        Self { inner }
    }

    pub fn state(&self) -> ObservationHandle<LinkState> {
        self.inner.stores.link_state_observation.clone()
    }
}
