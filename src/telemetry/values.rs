use crate::geo::{GeoPoint2d, GeoPoint3dMsl, GeoPoint3dRelHome};
use crate::state::GpsFixType;
use serde::{Deserialize, Serialize};

/// Global position payload grouped from MAVLink navigation messages.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct GlobalPosition {
    pub latitude_deg: f64,
    pub longitude_deg: f64,
    pub altitude_msl_m: f64,
    pub relative_alt_m: f64,
}

/// Euler-angle attitude snapshot in degrees.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct EulerAttitude {
    pub roll_deg: f64,
    pub pitch_deg: f64,
    pub yaw_deg: f64,
}

/// GPS quality summary from fix type, satellites, and HDOP.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct GpsQuality {
    pub fix_type: GpsFixType,
    pub satellites: Option<u8>,
    pub hdop: Option<f64>,
}

/// Battery cell voltage vector in volts.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CellVoltages {
    pub voltages_v: Vec<f64>,
}

/// Distance and bearing to current waypoint target.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct WaypointProgress {
    pub distance_m: f64,
    pub bearing_deg: f64,
}

/// Guidance state from navigation controller outputs.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct GuidanceState {
    pub bearing_deg: f64,
    pub cross_track_error_m: f64,
}

/// Terrain clearance values from terrain-report telemetry.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct TerrainClearance {
    pub terrain_height_m: f64,
    pub height_above_terrain_m: f64,
}

impl From<GlobalPosition> for GeoPoint2d {
    fn from(value: GlobalPosition) -> Self {
        Self {
            latitude_deg: value.latitude_deg,
            longitude_deg: value.longitude_deg,
        }
    }
}

impl From<GlobalPosition> for GeoPoint3dMsl {
    fn from(value: GlobalPosition) -> Self {
        Self {
            latitude_deg: value.latitude_deg,
            longitude_deg: value.longitude_deg,
            altitude_msl_m: value.altitude_msl_m,
        }
    }
}

impl From<GlobalPosition> for GeoPoint3dRelHome {
    fn from(value: GlobalPosition) -> Self {
        Self {
            latitude_deg: value.latitude_deg,
            longitude_deg: value.longitude_deg,
            relative_alt_m: value.relative_alt_m,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn global_position_to_geo_point_conversions() {
        let global = GlobalPosition {
            latitude_deg: 47.397742,
            longitude_deg: 8.545594,
            altitude_msl_m: 510.0,
            relative_alt_m: 50.0,
        };

        let point_2d: GeoPoint2d = global.clone().into();
        assert_eq!(point_2d.latitude_deg, 47.397742);
        assert_eq!(point_2d.longitude_deg, 8.545594);

        let point_msl: GeoPoint3dMsl = global.clone().into();
        assert_eq!(point_msl.latitude_deg, 47.397742);
        assert_eq!(point_msl.longitude_deg, 8.545594);
        assert_eq!(point_msl.altitude_msl_m, 510.0);

        let point_rel_home: GeoPoint3dRelHome = global.into();
        assert_eq!(point_rel_home.latitude_deg, 47.397742);
        assert_eq!(point_rel_home.longitude_deg, 8.545594);
        assert_eq!(point_rel_home.relative_alt_m, 50.0);
    }
}
