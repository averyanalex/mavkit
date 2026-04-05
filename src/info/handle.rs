use crate::info::logic::{best_display_id, best_unique_id};
use crate::info::{FirmwareInfo, HardwareInfo, PersistentIdentity, UniqueIds};
use crate::observation::ObservationHandle;
use crate::vehicle::VehicleInner;

/// Accessor for firmware, hardware, and persistent identity observations.
///
/// Obtained from [`Vehicle::info`](crate::Vehicle::info). All observations here are derived
/// from the `AUTOPILOT_VERSION` message requested during connection initialisation.
/// They are available once the init sequence completes and remain stable for the session.
#[derive(Clone)]
pub struct InfoHandle<'a> {
    inner: &'a VehicleInner,
}

impl<'a> InfoHandle<'a> {
    pub(crate) fn new(inner: &'a VehicleInner) -> Self {
        Self { inner }
    }

    /// Firmware version, git hash, and OS version parsed from `AUTOPILOT_VERSION`.
    ///
    /// Returns `None` while the init sequence is still in progress.
    pub fn firmware(&self) -> ObservationHandle<FirmwareInfo> {
        self.inner.info.firmware()
    }

    /// Board vendor/product IDs parsed from `AUTOPILOT_VERSION`.
    ///
    /// Returns `None` while the init sequence is still in progress.
    pub fn hardware(&self) -> ObservationHandle<HardwareInfo> {
        self.inner.info.hardware()
    }

    /// Hardware UID, 64-bit UID, and any remote-ID or board-ID strings.
    ///
    /// Returns `None` while the init sequence is still in progress.
    pub fn unique_ids(&self) -> ObservationHandle<UniqueIds> {
        self.inner.info.unique_ids()
    }

    /// Returns the most stable hardware-unique identifier available, or `None` if none is known.
    ///
    /// Preference order: MCU UID2 (`mcu:…`) > UID64 (`uid64:…`) > remote ID (`rid:…`).
    /// The board ID string (`board:…`) is intentionally excluded because it is a firmware
    /// constant rather than a hardware address and is not unique across device instances.
    pub fn best_unique_id(&self) -> Option<String> {
        let unique_ids = self.unique_ids().latest();
        best_unique_id(unique_ids.as_ref())
    }

    /// Returns a stable human-readable identifier for UI labels and cache keys.
    ///
    /// Falls back through: hardware UID → board ID → USB VID:PID → `sys:{id}/{comp}`.
    /// The fallback is always non-`None`.
    pub fn best_display_id(&self) -> String {
        let vehicle_state = self.inner.stores.vehicle_state.borrow().clone();
        let unique_ids = self.unique_ids().latest();
        let hardware = self.hardware().latest();
        best_display_id(unique_ids.as_ref(), hardware.as_ref(), &vehicle_state)
    }

    /// Persistent identity observation that transitions from [`PersistentIdentity::Pending`] to
    /// [`PersistentIdentity::Ready`] once a hardware UID is available.
    ///
    /// Suitable as a cache key that survives across reconnects. A `Pending` value falls back to
    /// the MAVLink `system_id`/`component_id` pair, which is session-scoped only.
    pub fn persistent_identity(&self) -> ObservationHandle<PersistentIdentity> {
        self.inner.info.persistent_identity()
    }
}
