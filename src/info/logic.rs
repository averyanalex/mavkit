use crate::dialect;
use crate::event_loop::InitState;
use crate::info::{FirmwareInfo, HardwareInfo, PersistentIdentity, UniqueIds};
use crate::state::VehicleState;
use std::fmt::Write as _;

pub(super) fn firmware_from_init_state(
    state: &InitState<dialect::AUTOPILOT_VERSION_DATA>,
) -> Option<FirmwareInfo> {
    match state {
        InitState::Unknown | InitState::Requesting { .. } => None,
        InitState::Available(version) => Some(firmware_info_from_autopilot_version(version)),
        InitState::Unavailable { .. } => Some(FirmwareInfo::default()),
    }
}

pub(super) fn hardware_from_init_state(
    state: &InitState<dialect::AUTOPILOT_VERSION_DATA>,
) -> Option<HardwareInfo> {
    match state {
        InitState::Unknown | InitState::Requesting { .. } => None,
        InitState::Available(version) => Some(hardware_info_from_autopilot_version(version)),
        InitState::Unavailable { .. } => Some(HardwareInfo::default()),
    }
}

pub(super) fn unique_ids_from_init_state(
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
