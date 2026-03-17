use pyo3::prelude::*;

use crate::error::to_py_err;
use crate::guided::PyArduGuidedSession;
use crate::vehicle::vehicle_label;

macro_rules! define_vehicle_handle {
    ($rust_name:ident, $py_name:literal) => {
        #[pyclass(name = $py_name, frozen, skip_from_py_object)]
        #[derive(Clone)]
        pub struct $rust_name {
            inner: mavkit::Vehicle,
        }

        impl $rust_name {
            fn new(inner: mavkit::Vehicle) -> Self {
                Self { inner }
            }
        }

        #[pymethods]
        impl $rust_name {
            fn __repr__(&self) -> String {
                format!(concat!($py_name, "({})"), vehicle_label(&self.inner))
            }
        }
    };
}

define_vehicle_handle!(PyArduCopterHandle, "ArduCopterHandle");
define_vehicle_handle!(PyArduPlaneVtolHandle, "ArduPlaneVtolHandle");
define_vehicle_handle!(PyArduRoverHandle, "ArduRoverHandle");
define_vehicle_handle!(PyArduSubHandle, "ArduSubHandle");

#[pyclass(name = "ArduPlaneHandle", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyArduPlaneHandle {
    inner: mavkit::Vehicle,
}

impl PyArduPlaneHandle {
    fn new(inner: mavkit::Vehicle) -> Self {
        Self { inner }
    }
}

#[pymethods]
impl PyArduPlaneHandle {
    fn vtol(&self) -> Option<PyArduPlaneVtolHandle> {
        self.inner
            .ardupilot()
            .plane()
            .and_then(|plane| plane.vtol())
            .map(|_| PyArduPlaneVtolHandle::new(self.inner.clone()))
    }

    fn __repr__(&self) -> String {
        format!("ArduPlaneHandle({})", vehicle_label(&self.inner))
    }
}

#[pyclass(name = "ArduPilotHandle", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyArduPilotHandle {
    inner: mavkit::Vehicle,
}

impl PyArduPilotHandle {
    pub(crate) fn new(inner: mavkit::Vehicle) -> Self {
        Self { inner }
    }
}

#[pymethods]
impl PyArduPilotHandle {
    fn copter(&self) -> Option<PyArduCopterHandle> {
        self.inner
            .ardupilot()
            .copter()
            .map(|_| PyArduCopterHandle::new(self.inner.clone()))
    }

    fn plane(&self) -> Option<PyArduPlaneHandle> {
        self.inner
            .ardupilot()
            .plane()
            .map(|_| PyArduPlaneHandle::new(self.inner.clone()))
    }

    fn rover(&self) -> Option<PyArduRoverHandle> {
        self.inner
            .ardupilot()
            .rover()
            .map(|_| PyArduRoverHandle::new(self.inner.clone()))
    }

    fn sub(&self) -> Option<PyArduSubHandle> {
        self.inner
            .ardupilot()
            .sub()
            .map(|_| PyArduSubHandle::new(self.inner.clone()))
    }

    fn guided<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        let vehicle = self.inner.clone();
        let label = vehicle_label(&vehicle);
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            let session = vehicle.ardupilot().guided().await.map_err(to_py_err)?;
            Ok(PyArduGuidedSession::new(session, label))
        })
    }

    fn request_prearm_checks<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        let vehicle = self.inner.clone();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            vehicle
                .ardupilot()
                .request_prearm_checks()
                .await
                .map_err(to_py_err)?;
            Ok(())
        })
    }

    fn motor_test<'py>(
        &self,
        py: Python<'py>,
        instance: u8,
        throttle_pct: f32,
        duration_s: u16,
    ) -> PyResult<Bound<'py, PyAny>> {
        let vehicle = self.inner.clone();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            vehicle
                .ardupilot()
                .motor_test(instance, throttle_pct, duration_s)
                .await
                .map_err(to_py_err)?;
            Ok(())
        })
    }

    fn preflight_calibration<'py>(
        &self,
        py: Python<'py>,
        gyro: bool,
        accel: bool,
        baro: bool,
        accel_trim: bool,
    ) -> PyResult<Bound<'py, PyAny>> {
        let vehicle = self.inner.clone();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            vehicle
                .ardupilot()
                .preflight_calibration(gyro, accel, baro, accel_trim)
                .await
                .map_err(to_py_err)?;
            Ok(())
        })
    }

    fn start_mag_cal<'py>(&self, py: Python<'py>, compass_mask: u8) -> PyResult<Bound<'py, PyAny>> {
        let vehicle = self.inner.clone();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            vehicle
                .ardupilot()
                .start_mag_cal(compass_mask)
                .await
                .map_err(to_py_err)?;
            Ok(())
        })
    }

    fn accept_mag_cal<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        let vehicle = self.inner.clone();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            vehicle
                .ardupilot()
                .accept_mag_cal()
                .await
                .map_err(to_py_err)?;
            Ok(())
        })
    }

    fn cancel_mag_cal<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        let vehicle = self.inner.clone();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            vehicle
                .ardupilot()
                .cancel_mag_cal()
                .await
                .map_err(to_py_err)?;
            Ok(())
        })
    }

    fn reboot<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        let vehicle = self.inner.clone();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            vehicle.ardupilot().reboot().await.map_err(to_py_err)?;
            Ok(())
        })
    }

    fn reboot_to_bootloader<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        let vehicle = self.inner.clone();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            vehicle
                .ardupilot()
                .reboot_to_bootloader()
                .await
                .map_err(to_py_err)?;
            Ok(())
        })
    }

    fn __repr__(&self) -> String {
        format!("ArduPilotHandle({})", vehicle_label(&self.inner))
    }
}
