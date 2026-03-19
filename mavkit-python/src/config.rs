use std::time::Duration;

use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;

use crate::mission::PyRetryPolicy;

fn validate_timeout(name: &str, value: f64) -> PyResult<Duration> {
    if !value.is_finite() || value < 0.0 {
        return Err(PyValueError::new_err(format!(
            "{name} must be a finite non-negative number",
        )));
    }
    Ok(Duration::from_secs_f64(value))
}

#[pyclass(name = "VehicleConfig", frozen, from_py_object)]
#[derive(Clone)]
pub struct PyVehicleConfig {
    gcs_system_id: u8,
    gcs_component_id: u8,
    auto_request_home: bool,
    command_buffer_size: usize,
    connect_timeout: Duration,
    command_timeout: Duration,
    command_completion_timeout: Duration,
    transfer_timeout: Duration,
    retry: mavkit::RetryPolicy,
}

impl PyVehicleConfig {
    pub(crate) fn into_inner(self) -> mavkit::VehicleConfig {
        mavkit::VehicleConfig {
            gcs_system_id: self.gcs_system_id,
            gcs_component_id: self.gcs_component_id,
            retry_policy: self.retry,
            auto_request_home: self.auto_request_home,
            command_buffer_size: self.command_buffer_size,
            connect_timeout: self.connect_timeout,
            command_timeout: self.command_timeout,
            command_completion_timeout: self.command_completion_timeout,
            transfer_timeout: self.transfer_timeout,
            ..mavkit::VehicleConfig::default()
        }
    }
}

#[pymethods]
impl PyVehicleConfig {
    #[new]
    #[pyo3(signature = (
        *,
        gcs_system_id = 255,
        gcs_component_id = 190,
        auto_request_home = true,
        command_buffer_size = 32,
        connect_timeout_secs = 10.0,
        command_timeout_secs = 5.0,
        command_completion_timeout_secs = 10.0,
        transfer_timeout_secs = 30.0,
        retry_policy = None,
    ))]
    fn new(
        gcs_system_id: u8,
        gcs_component_id: u8,
        auto_request_home: bool,
        command_buffer_size: usize,
        connect_timeout_secs: f64,
        command_timeout_secs: f64,
        command_completion_timeout_secs: f64,
        transfer_timeout_secs: f64,
        retry_policy: Option<PyRetryPolicy>,
    ) -> PyResult<Self> {
        if command_buffer_size == 0 {
            return Err(PyValueError::new_err(
                "command_buffer_size must be at least 1",
            ));
        }

        Ok(Self {
            gcs_system_id,
            gcs_component_id,
            auto_request_home,
            command_buffer_size,
            connect_timeout: validate_timeout("connect_timeout_secs", connect_timeout_secs)?,
            command_timeout: validate_timeout("command_timeout_secs", command_timeout_secs)?,
            command_completion_timeout: validate_timeout(
                "command_completion_timeout_secs",
                command_completion_timeout_secs,
            )?,
            transfer_timeout: validate_timeout("transfer_timeout_secs", transfer_timeout_secs)?,
            retry: retry_policy.map(|r| r.inner).unwrap_or_default(),
        })
    }

    #[getter]
    fn gcs_system_id(&self) -> u8 {
        self.gcs_system_id
    }
    #[getter]
    fn gcs_component_id(&self) -> u8 {
        self.gcs_component_id
    }
    #[getter]
    fn auto_request_home(&self) -> bool {
        self.auto_request_home
    }
    #[getter]
    fn command_buffer_size(&self) -> usize {
        self.command_buffer_size
    }
    #[getter]
    fn connect_timeout_secs(&self) -> f64 {
        self.connect_timeout.as_secs_f64()
    }
    #[getter]
    fn command_timeout_secs(&self) -> f64 {
        self.command_timeout.as_secs_f64()
    }
    #[getter]
    fn command_completion_timeout_secs(&self) -> f64 {
        self.command_completion_timeout.as_secs_f64()
    }
    #[getter]
    fn transfer_timeout_secs(&self) -> f64 {
        self.transfer_timeout.as_secs_f64()
    }
    #[getter]
    fn retry_policy(&self) -> PyRetryPolicy {
        PyRetryPolicy { inner: self.retry }
    }

    fn __repr__(&self) -> String {
        format!(
            "VehicleConfig(sys_id={}, comp_id={}, connect_timeout={:.1}s, \
             command_timeout={:.1}s, transfer_timeout={:.1}s)",
            self.gcs_system_id,
            self.gcs_component_id,
            self.connect_timeout.as_secs_f64(),
            self.command_timeout.as_secs_f64(),
            self.transfer_timeout.as_secs_f64(),
        )
    }
}
