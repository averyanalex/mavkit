#[allow(dead_code)]
mod common;

use mavkit::SensorHealthState;
use std::time::Duration;

#[tokio::test]
#[ignore = "requires ArduPilot SITL endpoint"]
async fn sitl_telemetry_attitude_euler_available() {
    let vehicle = common::setup_sitl_vehicle().await;
    let result: Result<(), String> = async {
        let sample = vehicle
            .telemetry()
            .attitude()
            .euler()
            .wait_timeout(Duration::from_secs(10))
            .await
            .map_err(|e| e.to_string())?;

        if !sample.value.roll_deg.is_finite()
            || !sample.value.pitch_deg.is_finite()
            || !sample.value.yaw_deg.is_finite()
        {
            return Err(String::from("attitude contained non-finite values"));
        }

        // On ground, roll/pitch should be within ~17 degrees
        if sample.value.roll_deg.abs() > 17.0 || sample.value.pitch_deg.abs() > 17.0 {
            return Err(format!(
                "attitude roll/pitch unexpectedly large for stationary vehicle: roll={}, pitch={}",
                sample.value.roll_deg, sample.value.pitch_deg
            ));
        }

        Ok(())
    }
    .await;

    let _ = vehicle.disconnect().await;
    if let Err(err) = result {
        panic!("{err}");
    }
}

#[tokio::test]
#[ignore = "requires ArduPilot SITL endpoint"]
async fn sitl_telemetry_battery_voltage_available() {
    let vehicle = common::setup_sitl_vehicle().await;
    let result: Result<(), String> = async {
        let sample = vehicle
            .telemetry()
            .battery()
            .voltage_v()
            .wait_timeout(Duration::from_secs(10))
            .await
            .map_err(|e| e.to_string())?;

        if sample.value < 5.0 || sample.value > 60.0 {
            return Err(format!("battery voltage out of range: {} V", sample.value));
        }

        Ok(())
    }
    .await;

    let _ = vehicle.disconnect().await;
    if let Err(err) = result {
        panic!("{err}");
    }
}

#[tokio::test]
#[ignore = "requires ArduPilot SITL endpoint"]
async fn sitl_telemetry_gps_quality_available() {
    let vehicle = common::setup_sitl_vehicle().await;
    let result: Result<(), String> = async {
        let sample = vehicle
            .telemetry()
            .gps()
            .quality()
            .wait_timeout(Duration::from_secs(10))
            .await
            .map_err(|e| e.to_string())?;

        if sample.value.satellites.unwrap_or(0) == 0 {
            return Err(String::from("GPS reports 0 satellites in SITL"));
        }

        Ok(())
    }
    .await;

    let _ = vehicle.disconnect().await;
    if let Err(err) = result {
        panic!("{err}");
    }
}

#[tokio::test]
#[ignore = "requires ArduPilot SITL endpoint"]
async fn sitl_telemetry_groundspeed_available() {
    let vehicle = common::setup_sitl_vehicle().await;
    let result: Result<(), String> = async {
        let sample = vehicle
            .telemetry()
            .position()
            .groundspeed_mps()
            .wait_timeout(Duration::from_secs(10))
            .await
            .map_err(|e| e.to_string())?;

        if !sample.value.is_finite() {
            return Err(String::from("groundspeed is not finite"));
        }
        if sample.value > 1.0 {
            return Err(format!(
                "groundspeed unexpectedly high for stationary vehicle: {} m/s",
                sample.value
            ));
        }

        Ok(())
    }
    .await;

    let _ = vehicle.disconnect().await;
    if let Err(err) = result {
        panic!("{err}");
    }
}

#[tokio::test]
#[ignore = "requires ArduPilot SITL endpoint"]
async fn sitl_telemetry_heading_available() {
    let vehicle = common::setup_sitl_vehicle().await;
    let result: Result<(), String> = async {
        let sample = vehicle
            .telemetry()
            .position()
            .heading_deg()
            .wait_timeout(Duration::from_secs(10))
            .await
            .map_err(|e| e.to_string())?;

        if sample.value < 0.0 || sample.value > 360.0 {
            return Err(format!("heading out of range: {}", sample.value));
        }

        Ok(())
    }
    .await;

    let _ = vehicle.disconnect().await;
    if let Err(err) = result {
        panic!("{err}");
    }
}

#[tokio::test]
#[ignore = "requires ArduPilot SITL endpoint"]
async fn sitl_telemetry_sensor_health_available() {
    let vehicle = common::setup_sitl_vehicle().await;
    let result: Result<(), String> = async {
        let sample = vehicle
            .telemetry()
            .sensor_health()
            .wait_timeout(Duration::from_secs(10))
            .await
            .map_err(|e| e.to_string())?;

        if sample.value.gyro != SensorHealthState::Healthy {
            return Err(format!("gyro not healthy: {:?}", sample.value.gyro));
        }
        if sample.value.accel != SensorHealthState::Healthy {
            return Err(format!("accel not healthy: {:?}", sample.value.accel));
        }

        Ok(())
    }
    .await;

    let _ = vehicle.disconnect().await;
    if let Err(err) = result {
        panic!("{err}");
    }
}

#[tokio::test]
#[ignore = "requires ArduPilot SITL endpoint"]
async fn sitl_telemetry_position_near_sitl_home() {
    let vehicle = common::setup_sitl_vehicle().await;
    let result: Result<(), String> = async {
        let sample = vehicle
            .telemetry()
            .position()
            .global()
            .wait_timeout(Duration::from_secs(10))
            .await
            .map_err(|e| e.to_string())?;

        // SITL home is 42.3898, -71.1476 — position should be within ~0.01 deg
        let lat_diff = (sample.value.latitude_deg - 42.3898).abs();
        let lon_diff = (sample.value.longitude_deg - (-71.1476)).abs();
        if lat_diff > 0.01 || lon_diff > 0.01 {
            return Err(format!(
                "position too far from SITL home: lat={}, lon={} (expected ~42.39, ~-71.15)",
                sample.value.latitude_deg, sample.value.longitude_deg
            ));
        }

        Ok(())
    }
    .await;

    let _ = vehicle.disconnect().await;
    if let Err(err) = result {
        panic!("{err}");
    }
}
