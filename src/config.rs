use crate::mission::RetryPolicy;
use std::time::Duration;

#[derive(Clone, Debug, PartialEq, Eq)]
/// Startup policy for one initialization domain.
///
/// Use this to bound how aggressively MAVKit probes optional data at connect time.
pub struct InitDomainPolicy {
    pub enabled: bool,
    pub max_attempts: u8,
    pub budget: Duration,
}

#[derive(Clone, Debug, PartialEq, Eq)]
/// Per-domain initialization policy used during connect bootstrap.
pub struct InitPolicyConfig {
    pub autopilot_version: InitDomainPolicy,
    pub available_modes: InitDomainPolicy,
    pub home: InitDomainPolicy,
    pub origin: InitDomainPolicy,
}

/// Configuration for a [`Vehicle`](crate::Vehicle) connection.
#[derive(Clone, Debug)]
pub struct VehicleConfig {
    pub connect_timeout: Duration,
    pub command_timeout: Duration,
    pub command_completion_timeout: Duration,
    pub transfer_timeout: Duration,
    pub init_policy: InitPolicyConfig,

    /// MAVLink system ID used by this GCS (default 255).
    pub gcs_system_id: u8,
    /// MAVLink component ID used by this GCS (default 190).
    pub gcs_component_id: u8,
    /// Retry policy for mission and parameter transfers.
    pub retry_policy: RetryPolicy,
    /// Automatically request home position after connecting (default `true`).
    pub auto_request_home: bool,
    /// Capacity of the internal command channel (default 32).
    pub command_buffer_size: usize,
}

impl Default for InitPolicyConfig {
    fn default() -> Self {
        Self {
            autopilot_version: InitDomainPolicy {
                enabled: true,
                max_attempts: 3,
                budget: Duration::from_secs(5),
            },
            available_modes: InitDomainPolicy {
                enabled: true,
                max_attempts: 2,
                budget: Duration::from_secs(5),
            },
            home: InitDomainPolicy {
                enabled: true,
                max_attempts: 1,
                budget: Duration::from_secs(5),
            },
            origin: InitDomainPolicy {
                enabled: true,
                max_attempts: 1,
                budget: Duration::from_secs(5),
            },
        }
    }
}

impl Default for VehicleConfig {
    fn default() -> Self {
        Self {
            connect_timeout: Duration::from_secs(10),
            command_timeout: Duration::from_secs(5),
            command_completion_timeout: Duration::from_secs(10),
            transfer_timeout: Duration::from_secs(30),
            init_policy: InitPolicyConfig::default(),
            gcs_system_id: 255,
            gcs_component_id: 190,
            retry_policy: RetryPolicy::default(),
            auto_request_home: true,
            command_buffer_size: 32,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{InitDomainPolicy, InitPolicyConfig, VehicleConfig};
    use std::time::Duration;

    #[test]
    fn defaults_use_connect_time_config_and_init_policy() {
        let cfg = VehicleConfig::default();

        assert_eq!(cfg.connect_timeout, Duration::from_secs(10));
        assert_eq!(cfg.command_timeout, Duration::from_secs(5));
        assert_eq!(cfg.command_completion_timeout, Duration::from_secs(10));
        assert_eq!(cfg.transfer_timeout, Duration::from_secs(30));

        assert_eq!(cfg.init_policy.autopilot_version.max_attempts, 3);
        assert_eq!(cfg.init_policy.available_modes.max_attempts, 2);
        assert!(cfg.init_policy.home.enabled);
        assert!(cfg.init_policy.origin.enabled);
    }

    #[test]
    fn init_policy_can_override_per_domain() {
        let policy = InitPolicyConfig {
            autopilot_version: InitDomainPolicy {
                enabled: false,
                max_attempts: 0,
                budget: Duration::from_secs(1),
            },
            ..InitPolicyConfig::default()
        };

        assert!(!policy.autopilot_version.enabled);
        assert_eq!(policy.autopilot_version.max_attempts, 0);
    }
}
