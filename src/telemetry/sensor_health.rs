use crate::dialect;
use crate::types::{SensorHealthState, SensorHealthSummary};

pub(crate) fn sensor_health_summary_from_sys_status(
    present: dialect::MavSysStatusSensor,
    enabled: dialect::MavSysStatusSensor,
    health: dialect::MavSysStatusSensor,
) -> SensorHealthSummary {
    SensorHealthSummary {
        gyro: sensor_state(
            dialect::MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_3D_GYRO,
            present,
            enabled,
            health,
        ),
        accel: sensor_state(
            dialect::MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_3D_ACCEL,
            present,
            enabled,
            health,
        ),
        mag: sensor_state(
            dialect::MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_3D_MAG,
            present,
            enabled,
            health,
        ),
        baro: sensor_state(
            dialect::MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE,
            present,
            enabled,
            health,
        ),
        gps: sensor_state(
            dialect::MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_GPS,
            present,
            enabled,
            health,
        ),
        airspeed: sensor_state(
            dialect::MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE,
            present,
            enabled,
            health,
        ),
        rc_receiver: sensor_state(
            dialect::MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_RC_RECEIVER,
            present,
            enabled,
            health,
        ),
        battery: sensor_state(
            dialect::MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_BATTERY,
            present,
            enabled,
            health,
        ),
        terrain: sensor_state(
            dialect::MavSysStatusSensor::MAV_SYS_STATUS_TERRAIN,
            present,
            enabled,
            health,
        ),
        geofence: sensor_state(
            dialect::MavSysStatusSensor::MAV_SYS_STATUS_GEOFENCE,
            present,
            enabled,
            health,
        ),
    }
}

fn sensor_state(
    bit: dialect::MavSysStatusSensor,
    present: dialect::MavSysStatusSensor,
    enabled: dialect::MavSysStatusSensor,
    health: dialect::MavSysStatusSensor,
) -> SensorHealthState {
    if !present.contains(bit) {
        SensorHealthState::NotPresent
    } else if !enabled.contains(bit) {
        SensorHealthState::Disabled
    } else if !health.contains(bit) {
        SensorHealthState::Unhealthy
    } else {
        SensorHealthState::Healthy
    }
}
