use crate::dialect;
use crate::event_loop::{InitManager, InitSnapshot, InitState};
use crate::observation::{ObservationHandle, ObservationWriter};
use crate::state::{StateChannels, VehicleState};
use crate::vehicle::VehicleInner;
use serde::{Deserialize, Serialize};
use std::fmt::Write as _;
use std::sync::{Arc, Mutex};

/// Parsed firmware identity details from `AUTOPILOT_VERSION`.
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct FirmwareInfo {
    pub version: Option<String>,
    pub custom_version: Option<[u8; 8]>,
    pub git_hash: Option<String>,
    pub os_version: Option<String>,
}

/// Parsed hardware board and USB identity details.
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct HardwareInfo {
    pub board_vendor_id: Option<u16>,
    pub board_product_id: Option<u16>,
    pub usb_vendor_id: Option<u16>,
    pub usb_product_id: Option<u16>,
    pub board_version: Option<u32>,
}

/// Stable identifiers MAVKit can use for cross-session correlation.
#[derive(Debug, Clone, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct UniqueIds {
    pub hardware_uid: Option<Vec<u8>>,
    pub uid: Option<u64>,
    pub remote_id: Option<String>,
    pub board_id: Option<String>,
}

/// Persistent identity readiness for UI labels and cache keys.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case", tag = "state")]
pub enum PersistentIdentity {
    Pending {
        system_id: u8,
        component_id: u8,
    },
    Ready {
        canonical_id: String,
        aliases: Vec<String>,
    },
}

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

#[derive(Clone)]
pub(crate) struct InfoDomain {
    inner: Arc<InfoDomainInner>,
}

struct InfoDomainInner {
    firmware_writer: ObservationWriter<FirmwareInfo>,
    firmware: ObservationHandle<FirmwareInfo>,
    hardware_writer: ObservationWriter<HardwareInfo>,
    hardware: ObservationHandle<HardwareInfo>,
    unique_ids_writer: ObservationWriter<UniqueIds>,
    unique_ids: ObservationHandle<UniqueIds>,
    persistent_identity_writer: ObservationWriter<PersistentIdentity>,
    persistent_identity: ObservationHandle<PersistentIdentity>,
    tracker: Mutex<InfoTracker>,
}

#[derive(Default)]
struct InfoTracker {
    vehicle_state: VehicleState,
    init_snapshot: InitSnapshot,
    firmware: Option<FirmwareInfo>,
    hardware: Option<HardwareInfo>,
    unique_ids: Option<UniqueIds>,
    persistent_identity: Option<PersistentIdentity>,
}

impl InfoDomain {
    pub(crate) fn new() -> Self {
        let (firmware_writer, firmware) = ObservationHandle::watch();
        let (hardware_writer, hardware) = ObservationHandle::watch();
        let (unique_ids_writer, unique_ids) = ObservationHandle::watch();
        let (persistent_identity_writer, persistent_identity) = ObservationHandle::watch();

        Self {
            inner: Arc::new(InfoDomainInner {
                firmware_writer,
                firmware,
                hardware_writer,
                hardware,
                unique_ids_writer,
                unique_ids,
                persistent_identity_writer,
                persistent_identity,
                tracker: Mutex::new(InfoTracker::default()),
            }),
        }
    }

    pub(crate) fn start(&self, stores: &StateChannels, init_manager: &InitManager) {
        let inner = self.inner.clone();
        let mut vehicle_state_rx = stores.vehicle_state.clone();
        let mut init_rx = init_manager.subscribe();

        inner.handle_vehicle_state(&vehicle_state_rx.borrow().clone());
        inner.handle_init_snapshot(&init_rx.borrow().clone());

        tokio::spawn(async move {
            loop {
                tokio::select! {
                    changed = vehicle_state_rx.changed() => {
                        if changed.is_err() {
                            break;
                        }
                        inner.handle_vehicle_state(&vehicle_state_rx.borrow_and_update().clone());
                    }
                    changed = init_rx.changed() => {
                        if changed.is_err() {
                            break;
                        }
                        inner.handle_init_snapshot(&init_rx.borrow_and_update().clone());
                    }
                }
            }
        });
    }

    pub(crate) fn firmware(&self) -> ObservationHandle<FirmwareInfo> {
        self.inner.firmware.clone()
    }

    pub(crate) fn hardware(&self) -> ObservationHandle<HardwareInfo> {
        self.inner.hardware.clone()
    }

    pub(crate) fn unique_ids(&self) -> ObservationHandle<UniqueIds> {
        self.inner.unique_ids.clone()
    }

    pub(crate) fn persistent_identity(&self) -> ObservationHandle<PersistentIdentity> {
        self.inner.persistent_identity.clone()
    }
}

impl InfoDomainInner {
    fn handle_vehicle_state(&self, vehicle_state: &VehicleState) {
        let mut tracker = self.tracker.lock().unwrap();
        tracker.vehicle_state = vehicle_state.clone();
        self.recompute(&mut tracker);
    }

    fn handle_init_snapshot(&self, init_snapshot: &InitSnapshot) {
        let mut tracker = self.tracker.lock().unwrap();
        tracker.init_snapshot = init_snapshot.clone();
        self.recompute(&mut tracker);
    }

    fn recompute(&self, tracker: &mut InfoTracker) {
        let firmware = firmware_from_init_state(&tracker.init_snapshot.autopilot_version);
        publish_if_changed(&mut tracker.firmware, firmware, &self.firmware_writer);

        let hardware = hardware_from_init_state(&tracker.init_snapshot.autopilot_version);
        publish_if_changed(&mut tracker.hardware, hardware, &self.hardware_writer);

        let unique_ids = unique_ids_from_init_state(&tracker.init_snapshot.autopilot_version);
        publish_if_changed(
            &mut tracker.unique_ids,
            unique_ids.clone(),
            &self.unique_ids_writer,
        );

        let identity = persistent_identity(&tracker.vehicle_state, unique_ids.as_ref());
        publish_value_if_changed(
            &mut tracker.persistent_identity,
            identity,
            &self.persistent_identity_writer,
        );
    }
}

fn publish_if_changed<T: Clone + Send + Sync + PartialEq + 'static>(
    slot: &mut Option<T>,
    next: Option<T>,
    writer: &ObservationWriter<T>,
) {
    if *slot == next {
        return;
    }

    *slot = next.clone();
    if let Some(value) = next {
        let _ = writer.publish(value);
    }
}

fn publish_value_if_changed<T: Clone + Send + Sync + PartialEq + 'static>(
    slot: &mut Option<T>,
    next: T,
    writer: &ObservationWriter<T>,
) {
    if slot.as_ref() == Some(&next) {
        return;
    }

    *slot = Some(next.clone());
    let _ = writer.publish(next);
}

fn firmware_from_init_state(
    state: &InitState<dialect::AUTOPILOT_VERSION_DATA>,
) -> Option<FirmwareInfo> {
    match state {
        InitState::Unknown | InitState::Requesting { .. } => None,
        InitState::Available(version) => Some(firmware_info_from_autopilot_version(version)),
        InitState::Unavailable { .. } => Some(FirmwareInfo::default()),
    }
}

fn hardware_from_init_state(
    state: &InitState<dialect::AUTOPILOT_VERSION_DATA>,
) -> Option<HardwareInfo> {
    match state {
        InitState::Unknown | InitState::Requesting { .. } => None,
        InitState::Available(version) => Some(hardware_info_from_autopilot_version(version)),
        InitState::Unavailable { .. } => Some(HardwareInfo::default()),
    }
}

fn unique_ids_from_init_state(
    state: &InitState<dialect::AUTOPILOT_VERSION_DATA>,
) -> Option<UniqueIds> {
    match state {
        InitState::Unknown | InitState::Requesting { .. } => None,
        InitState::Available(version) => Some(unique_ids_from_autopilot_version(version)),
        InitState::Unavailable { .. } => Some(UniqueIds::default()),
    }
}

pub(crate) fn firmware_info_from_autopilot_version(
    version: &dialect::AUTOPILOT_VERSION_DATA,
) -> FirmwareInfo {
    let custom_version = nonzero_array(version.flight_custom_version);

    FirmwareInfo {
        version: parse_semver(version.flight_sw_version),
        custom_version,
        git_hash: custom_version.as_ref().map(|bytes| hex_bytes(bytes)),
        os_version: parse_semver(version.os_sw_version),
    }
}

fn hardware_info_from_autopilot_version(version: &dialect::AUTOPILOT_VERSION_DATA) -> HardwareInfo {
    HardwareInfo {
        board_vendor_id: nonzero_u16(version.vendor_id),
        board_product_id: nonzero_u16(version.product_id),
        usb_vendor_id: None,
        usb_product_id: None,
        board_version: nonzero_u32(version.board_version),
    }
}

fn unique_ids_from_autopilot_version(version: &dialect::AUTOPILOT_VERSION_DATA) -> UniqueIds {
    let uid = nonzero_u64(version.uid);
    let hardware_uid = if version.uid2.iter().any(|byte| *byte != 0) {
        Some(version.uid2.to_vec())
    } else {
        uid.map(|value| value.to_be_bytes().to_vec())
    };

    UniqueIds {
        hardware_uid,
        uid,
        remote_id: None,
        board_id: None,
    }
}

pub(crate) fn best_unique_id(unique_ids: Option<&UniqueIds>) -> Option<String> {
    let unique_ids = unique_ids?;

    uid2_display_id(unique_ids)
        .or_else(|| uid_display_id(unique_ids))
        .or_else(|| {
            unique_ids
                .remote_id
                .as_ref()
                .map(|remote_id| format!("rid:{remote_id}"))
        })
}

pub(crate) fn best_display_id(
    unique_ids: Option<&UniqueIds>,
    hardware: Option<&HardwareInfo>,
    vehicle_state: &VehicleState,
) -> String {
    if let Some(best) = best_unique_id(unique_ids) {
        return best;
    }

    if let Some(board_id) = unique_ids.and_then(|ids| ids.board_id.as_ref()) {
        return format!("board:{board_id}");
    }

    if let Some(hardware) = hardware
        && let (Some(vendor_id), Some(product_id)) =
            (hardware.usb_vendor_id, hardware.usb_product_id)
    {
        return format!("usb:{vendor_id:04x}:{product_id:04x}");
    }

    format!(
        "sys:{}/{}",
        vehicle_state.system_id, vehicle_state.component_id
    )
}

pub(crate) fn persistent_identity(
    vehicle_state: &VehicleState,
    unique_ids: Option<&UniqueIds>,
) -> PersistentIdentity {
    let aliases = persistent_aliases(unique_ids);
    if let Some(canonical_id) = aliases.first().cloned() {
        PersistentIdentity::Ready {
            canonical_id,
            aliases,
        }
    } else {
        PersistentIdentity::Pending {
            system_id: vehicle_state.system_id,
            component_id: vehicle_state.component_id,
        }
    }
}

fn persistent_aliases(unique_ids: Option<&UniqueIds>) -> Vec<String> {
    let mut aliases = Vec::new();

    if let Some(unique_ids) = unique_ids {
        if let Some(uid2) = uid2_display_id(unique_ids) {
            aliases.push(uid2);
        }
        if let Some(uid) = uid_display_id(unique_ids) {
            aliases.push(uid);
        }
    }

    aliases
}

fn uid2_display_id(unique_ids: &UniqueIds) -> Option<String> {
    uid2_bytes(unique_ids).map(|bytes| format!("mcu:{}", hex_bytes(bytes)))
}

fn uid_display_id(unique_ids: &UniqueIds) -> Option<String> {
    unique_ids.uid.map(|uid| format!("uid64:{uid:016x}"))
}

fn uid2_bytes(unique_ids: &UniqueIds) -> Option<&[u8]> {
    let hardware_uid = unique_ids.hardware_uid.as_deref()?;
    let Some(uid) = unique_ids.uid else {
        return Some(hardware_uid);
    };

    let uid_bytes = uid.to_be_bytes();
    if hardware_uid == uid_bytes {
        None
    } else {
        Some(hardware_uid)
    }
}

fn parse_semver(version: u32) -> Option<String> {
    if version == 0 {
        return None;
    }

    let major = (version >> 24) & 0xff;
    let minor = (version >> 16) & 0xff;
    let patch = (version >> 8) & 0xff;
    Some(format!("{major}.{minor}.{patch}"))
}

fn hex_bytes(bytes: &[u8]) -> String {
    let mut formatted = String::with_capacity(bytes.len() * 2);
    for byte in bytes {
        let _ = write!(&mut formatted, "{byte:02x}");
    }
    formatted
}

fn nonzero_array(bytes: [u8; 8]) -> Option<[u8; 8]> {
    bytes.iter().any(|byte| *byte != 0).then_some(bytes)
}

fn nonzero_u16(value: u16) -> Option<u16> {
    (value != 0).then_some(value)
}

fn nonzero_u32(value: u32) -> Option<u32> {
    (value != 0).then_some(value)
}

fn nonzero_u64(value: u64) -> Option<u64> {
    (value != 0).then_some(value)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn vehicle_state(system_id: u8, component_id: u8) -> VehicleState {
        VehicleState {
            system_id,
            component_id,
            ..VehicleState::default()
        }
    }

    #[test]
    fn info_display_id_precedence_uses_required_prefixes() {
        let state = vehicle_state(1, 1);
        let hardware = HardwareInfo {
            usb_vendor_id: Some(0x1209),
            usb_product_id: Some(0x5741),
            ..HardwareInfo::default()
        };
        let board = UniqueIds {
            board_id: Some("apj-140".into()),
            ..UniqueIds::default()
        };
        let remote = UniqueIds {
            remote_id: Some("ABC123DEF456".into()),
            ..UniqueIds::default()
        };
        let uid = UniqueIds {
            hardware_uid: Some(0x0123_4567_89ab_cdef_u64.to_be_bytes().to_vec()),
            uid: Some(0x0123_4567_89ab_cdef),
            ..UniqueIds::default()
        };
        let uid2 = UniqueIds {
            hardware_uid: Some(vec![
                0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb,
            ]),
            uid: Some(0x0123_4567_89ab_cdef),
            ..UniqueIds::default()
        };

        assert_eq!(best_display_id(None, None, &state), "sys:1/1");
        assert_eq!(
            best_display_id(None, Some(&hardware), &state),
            "usb:1209:5741"
        );
        assert_eq!(
            best_display_id(Some(&board), Some(&hardware), &state),
            "board:apj-140"
        );
        assert_eq!(
            best_display_id(Some(&remote), Some(&hardware), &state),
            "rid:ABC123DEF456"
        );
        assert_eq!(
            best_display_id(Some(&uid), Some(&hardware), &state),
            "uid64:0123456789abcdef"
        );
        assert_eq!(
            best_display_id(Some(&uid2), Some(&hardware), &state),
            "mcu:00112233445566778899aabb"
        );
    }

    #[test]
    fn info_best_unique_id_only_returns_unique_sources() {
        let board_only = UniqueIds {
            board_id: Some("apj-140".into()),
            ..UniqueIds::default()
        };
        let remote = UniqueIds {
            remote_id: Some("ABC123DEF456".into()),
            ..UniqueIds::default()
        };
        let uid = UniqueIds {
            hardware_uid: Some(0x0123_4567_89ab_cdef_u64.to_be_bytes().to_vec()),
            uid: Some(0x0123_4567_89ab_cdef),
            ..UniqueIds::default()
        };
        let uid2 = UniqueIds {
            hardware_uid: Some(vec![
                0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb,
            ]),
            uid: Some(0x0123_4567_89ab_cdef),
            ..UniqueIds::default()
        };

        assert_eq!(best_unique_id(Some(&board_only)), None);
        assert_eq!(
            best_unique_id(Some(&remote)),
            Some("rid:ABC123DEF456".into())
        );
        assert_eq!(
            best_unique_id(Some(&uid)),
            Some("uid64:0123456789abcdef".into())
        );
        assert_eq!(
            best_unique_id(Some(&uid2)),
            Some("mcu:00112233445566778899aabb".into())
        );
    }

    #[test]
    fn info_firmware_parsing_extracts_semver_and_git_hash() {
        let info = firmware_info_from_autopilot_version(&dialect::AUTOPILOT_VERSION_DATA {
            flight_sw_version: 0x01020304,
            os_sw_version: 0x05060708,
            flight_custom_version: [0xde, 0xad, 0xbe, 0xef, 0xca, 0xfe, 0xba, 0xbe],
            ..dialect::AUTOPILOT_VERSION_DATA::default()
        });

        assert_eq!(info.version.as_deref(), Some("1.2.3"));
        assert_eq!(info.os_version.as_deref(), Some("5.6.7"));
        assert_eq!(
            info.custom_version,
            Some([0xde, 0xad, 0xbe, 0xef, 0xca, 0xfe, 0xba, 0xbe])
        );
        assert_eq!(info.git_hash.as_deref(), Some("deadbeefcafebabe"));
    }

    #[test]
    fn info_persistent_identity_starts_pending_and_prefers_stronger_aliases() {
        let state = vehicle_state(1, 1);
        let uid = UniqueIds {
            hardware_uid: Some(0x0123_4567_89ab_cdef_u64.to_be_bytes().to_vec()),
            uid: Some(0x0123_4567_89ab_cdef),
            ..UniqueIds::default()
        };
        let uid2 = UniqueIds {
            hardware_uid: Some(vec![
                0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb,
            ]),
            uid: Some(0x0123_4567_89ab_cdef),
            ..UniqueIds::default()
        };

        assert_eq!(
            persistent_identity(&state, None),
            PersistentIdentity::Pending {
                system_id: 1,
                component_id: 1,
            }
        );
        assert_eq!(
            persistent_identity(&state, Some(&uid)),
            PersistentIdentity::Ready {
                canonical_id: "uid64:0123456789abcdef".into(),
                aliases: vec!["uid64:0123456789abcdef".into()],
            }
        );
        assert_eq!(
            persistent_identity(&state, Some(&uid2)),
            PersistentIdentity::Ready {
                canonical_id: "mcu:00112233445566778899aabb".into(),
                aliases: vec![
                    "mcu:00112233445566778899aabb".into(),
                    "uid64:0123456789abcdef".into(),
                ],
            }
        );
    }
}
