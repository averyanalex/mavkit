use super::request_handles::{EventMessageHandle, PeriodicMessageHandle};
use crate::dialect;
use crate::observation::MessageHandle;
use crate::telemetry::handles::TelemetryMetricHandles;
use crate::telemetry::status_text::StatusTextEvent;

pub(super) const MSG_ID_LOCAL_POSITION_NED: u32 = 32;
pub(super) const MSG_ID_GLOBAL_POSITION_INT: u32 = 33;
pub(super) const MSG_ID_SERVO_OUTPUT_RAW: u32 = 36;
pub(super) const MSG_ID_GPS_GLOBAL_ORIGIN: u32 = 49;
pub(super) const MSG_ID_NAV_CONTROLLER_OUTPUT: u32 = 62;
pub(super) const MSG_ID_RC_CHANNELS: u32 = 65;
pub(super) const MSG_ID_VFR_HUD: u32 = 74;
pub(super) const MSG_ID_TERRAIN_REPORT: u32 = 136;
pub(super) const MSG_ID_BATTERY_STATUS: u32 = 147;
pub(super) const MSG_ID_ATTITUDE: u32 = 30;
pub(super) const MSG_ID_GPS_RAW_INT: u32 = 24;
pub(super) const MSG_ID_SYS_STATUS: u32 = 1;
pub(super) const MSG_ID_HOME_POSITION: u32 = 242;

/// Namespace exposing typed raw MAVLink message streams.
pub struct MessagesHandle<'a> {
    pub(crate) handles: &'a TelemetryMetricHandles,
}

impl MessagesHandle<'_> {
    pub fn vfr_hud(&self) -> PeriodicMessageHandle<dialect::VFR_HUD_DATA> {
        PeriodicMessageHandle::new(
            self.handles.message_handles.vfr_hud.clone(),
            self.handles.message_handles.commands.clone(),
            MSG_ID_VFR_HUD,
            0.0,
        )
    }

    pub fn global_position_int(&self) -> PeriodicMessageHandle<dialect::GLOBAL_POSITION_INT_DATA> {
        PeriodicMessageHandle::new(
            self.handles.message_handles.global_position_int.clone(),
            self.handles.message_handles.commands.clone(),
            MSG_ID_GLOBAL_POSITION_INT,
            0.0,
        )
    }

    pub fn local_position_ned(&self) -> PeriodicMessageHandle<dialect::LOCAL_POSITION_NED_DATA> {
        PeriodicMessageHandle::new(
            self.handles.message_handles.local_position_ned.clone(),
            self.handles.message_handles.commands.clone(),
            MSG_ID_LOCAL_POSITION_NED,
            0.0,
        )
    }

    pub fn gps_raw_int(&self) -> PeriodicMessageHandle<dialect::GPS_RAW_INT_DATA> {
        PeriodicMessageHandle::new(
            self.handles.message_handles.gps_raw_int.clone(),
            self.handles.message_handles.commands.clone(),
            MSG_ID_GPS_RAW_INT,
            0.0,
        )
    }

    pub fn attitude(&self) -> PeriodicMessageHandle<dialect::ATTITUDE_DATA> {
        PeriodicMessageHandle::new(
            self.handles.message_handles.attitude.clone(),
            self.handles.message_handles.commands.clone(),
            MSG_ID_ATTITUDE,
            0.0,
        )
    }

    pub fn sys_status(&self) -> PeriodicMessageHandle<dialect::SYS_STATUS_DATA> {
        PeriodicMessageHandle::new(
            self.handles.message_handles.sys_status.clone(),
            self.handles.message_handles.commands.clone(),
            MSG_ID_SYS_STATUS,
            0.0,
        )
    }

    pub fn battery_status(
        &self,
        instance: u8,
    ) -> PeriodicMessageHandle<dialect::BATTERY_STATUS_DATA> {
        PeriodicMessageHandle::new(
            self.handles.message_handles.battery_status.handle(instance),
            self.handles.message_handles.commands.clone(),
            MSG_ID_BATTERY_STATUS,
            f32::from(instance),
        )
    }

    pub fn nav_controller_output(
        &self,
    ) -> PeriodicMessageHandle<dialect::NAV_CONTROLLER_OUTPUT_DATA> {
        PeriodicMessageHandle::new(
            self.handles.message_handles.nav_controller_output.clone(),
            self.handles.message_handles.commands.clone(),
            MSG_ID_NAV_CONTROLLER_OUTPUT,
            0.0,
        )
    }

    pub fn terrain_report(&self) -> PeriodicMessageHandle<dialect::TERRAIN_REPORT_DATA> {
        PeriodicMessageHandle::new(
            self.handles.message_handles.terrain_report.clone(),
            self.handles.message_handles.commands.clone(),
            MSG_ID_TERRAIN_REPORT,
            0.0,
        )
    }

    pub fn rc_channels(&self) -> PeriodicMessageHandle<dialect::RC_CHANNELS_DATA> {
        PeriodicMessageHandle::new(
            self.handles.message_handles.rc_channels.clone(),
            self.handles.message_handles.commands.clone(),
            MSG_ID_RC_CHANNELS,
            0.0,
        )
    }

    pub fn servo_output_raw(
        &self,
        port: u8,
    ) -> PeriodicMessageHandle<dialect::SERVO_OUTPUT_RAW_DATA> {
        PeriodicMessageHandle::new(
            self.handles.message_handles.servo_output_raw.handle(port),
            self.handles.message_handles.commands.clone(),
            MSG_ID_SERVO_OUTPUT_RAW,
            f32::from(port),
        )
    }

    pub fn home_position(&self) -> EventMessageHandle<dialect::HOME_POSITION_DATA> {
        EventMessageHandle::new(
            self.handles.message_handles.home_position.clone(),
            self.handles.message_handles.commands.clone(),
            MSG_ID_HOME_POSITION,
            0.0,
        )
    }

    pub fn gps_global_origin(&self) -> EventMessageHandle<dialect::GPS_GLOBAL_ORIGIN_DATA> {
        EventMessageHandle::new(
            self.handles.message_handles.gps_global_origin.clone(),
            self.handles.message_handles.commands.clone(),
            MSG_ID_GPS_GLOBAL_ORIGIN,
            0.0,
        )
    }

    pub fn status_text(&self) -> MessageHandle<StatusTextEvent> {
        self.handles.message_handles.status_text.clone()
    }
}
