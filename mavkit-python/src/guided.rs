use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

use pyo3::prelude::*;
use tokio::sync::Mutex;

use crate::error::to_py_err;
use crate::macros::py_async_unit;

fn geo_point_2d(latitude_deg: f64, longitude_deg: f64) -> mavkit::GeoPoint2d {
    mavkit::GeoPoint2d {
        latitude_deg,
        longitude_deg,
    }
}

fn geo_point_rel_home(
    latitude_deg: f64,
    longitude_deg: f64,
    relative_alt_m: f64,
) -> mavkit::GeoPoint3dRelHome {
    mavkit::GeoPoint3dRelHome {
        latitude_deg,
        longitude_deg,
        relative_alt_m,
    }
}

fn geo_point_msl(
    latitude_deg: f64,
    longitude_deg: f64,
    altitude_msl_m: f64,
) -> mavkit::GeoPoint3dMsl {
    mavkit::GeoPoint3dMsl {
        latitude_deg,
        longitude_deg,
        altitude_msl_m,
    }
}

fn relative_climb_target(relative_climb_m: f32) -> mavkit::RelativeClimbTarget {
    mavkit::RelativeClimbTarget { relative_climb_m }
}

fn sub_goto_depth_target(
    latitude_deg: f64,
    longitude_deg: f64,
    depth_m: f32,
) -> mavkit::SubGotoDepthTarget {
    mavkit::SubGotoDepthTarget {
        point: geo_point_2d(latitude_deg, longitude_deg),
        depth_m,
    }
}

fn session_closed_error() -> mavkit::VehicleError {
    mavkit::VehicleError::OperationConflict {
        conflicting_domain: "ardupilot_guided".to_string(),
        conflicting_op: "session_closed".to_string(),
    }
}

struct PyGuidedSessionShared {
    session: Mutex<Option<mavkit::ArduGuidedSession>>,
    kind: mavkit::ArduGuidedKind,
    plane_kind: Option<mavkit::ArduPlaneKind>,
    closed: AtomicBool,
    label: String,
}

impl PyGuidedSessionShared {
    fn new(
        session: mavkit::ArduGuidedSession,
        kind: mavkit::ArduGuidedKind,
        plane_kind: Option<mavkit::ArduPlaneKind>,
        label: String,
    ) -> Self {
        Self {
            session: Mutex::new(Some(session)),
            kind,
            plane_kind,
            closed: AtomicBool::new(false),
            label,
        }
    }

    fn ensure_python_open(&self) -> Result<(), mavkit::VehicleError> {
        if self.closed.load(Ordering::Acquire) {
            return Err(session_closed_error());
        }
        Ok(())
    }

    async fn session_clone(&self) -> Result<mavkit::ArduGuidedSession, mavkit::VehicleError> {
        self.ensure_python_open()?;
        let guard = self.session.lock().await;
        guard.as_ref().cloned().ok_or_else(session_closed_error)
    }

    async fn close(&self) -> Result<(), mavkit::VehicleError> {
        if self.closed.swap(true, Ordering::AcqRel) {
            return Ok(());
        }

        let session = self.session.lock().await.take();
        if let Some(session) = session {
            session.close().await?;
        }
        Ok(())
    }

    fn kind(&self) -> mavkit::ArduGuidedKind {
        self.kind
    }

    fn plane_kind(&self) -> Option<mavkit::ArduPlaneKind> {
        self.plane_kind
    }

    fn label(&self) -> &str {
        &self.label
    }

    fn is_closed(&self) -> bool {
        self.closed.load(Ordering::Acquire)
    }
}

#[pyclass(name = "ArduGuidedSession", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyArduGuidedSession {
    shared: Arc<PyGuidedSessionShared>,
}

impl PyArduGuidedSession {
    pub(crate) fn new(session: mavkit::ArduGuidedSession, label: String) -> Self {
        let kind = session.kind();
        let plane_kind = match session.specific() {
            mavkit::GuidedSpecific::Plane(plane) => Some(plane.kind()),
            _ => None,
        };

        Self {
            shared: Arc::new(PyGuidedSessionShared::new(session, kind, plane_kind, label)),
        }
    }

    #[cfg_attr(not(test), allow(dead_code))]
    pub(crate) async fn close_impl(&self) -> Result<(), mavkit::VehicleError> {
        self.shared.close().await
    }

    fn status(&self) -> &'static str {
        if self.shared.is_closed() {
            "closed"
        } else {
            "open"
        }
    }

    fn copter_impl(&self) -> Result<Option<PyArduCopterGuidedHandle>, mavkit::VehicleError> {
        self.shared.ensure_python_open()?;
        Ok((self.shared.kind() == mavkit::ArduGuidedKind::Copter)
            .then_some(PyArduCopterGuidedHandle::new(self.shared.clone())))
    }

    fn plane_impl(&self) -> Result<Option<PyArduPlaneGuidedHandle>, mavkit::VehicleError> {
        self.shared.ensure_python_open()?;
        Ok((self.shared.kind() == mavkit::ArduGuidedKind::Plane)
            .then_some(PyArduPlaneGuidedHandle::new(self.shared.clone())))
    }

    fn rover_impl(&self) -> Result<Option<PyArduRoverGuidedHandle>, mavkit::VehicleError> {
        self.shared.ensure_python_open()?;
        Ok((self.shared.kind() == mavkit::ArduGuidedKind::Rover)
            .then_some(PyArduRoverGuidedHandle::new(self.shared.clone())))
    }

    fn sub_impl(&self) -> Result<Option<PyArduSubGuidedHandle>, mavkit::VehicleError> {
        self.shared.ensure_python_open()?;
        Ok((self.shared.kind() == mavkit::ArduGuidedKind::Sub)
            .then_some(PyArduSubGuidedHandle::new(self.shared.clone())))
    }
}

#[pymethods]
impl PyArduGuidedSession {
    fn close<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        py_async_unit!(py, shared = self.shared.clone(); shared.close())
    }

    fn copter(&self) -> PyResult<Option<PyArduCopterGuidedHandle>> {
        self.copter_impl().map_err(to_py_err)
    }

    fn plane(&self) -> PyResult<Option<PyArduPlaneGuidedHandle>> {
        self.plane_impl().map_err(to_py_err)
    }

    fn rover(&self) -> PyResult<Option<PyArduRoverGuidedHandle>> {
        self.rover_impl().map_err(to_py_err)
    }

    fn sub(&self) -> PyResult<Option<PyArduSubGuidedHandle>> {
        self.sub_impl().map_err(to_py_err)
    }

    fn __aenter__<'py>(slf: &Bound<'py, Self>, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        let obj = slf.clone().into_any().unbind();
        pyo3_async_runtimes::tokio::future_into_py(py, async move { Ok(obj) })
    }

    #[pyo3(signature = (_exc_type=None, _exc_val=None, _exc_tb=None))]
    fn __aexit__<'py>(
        &self,
        py: Python<'py>,
        _exc_type: Option<&Bound<'py, PyAny>>,
        _exc_val: Option<&Bound<'py, PyAny>>,
        _exc_tb: Option<&Bound<'py, PyAny>>,
    ) -> PyResult<Bound<'py, PyAny>> {
        let shared = self.shared.clone();
        pyo3_async_runtimes::tokio::future_into_py(py, async move {
            shared.close().await.map_err(to_py_err)?;
            Ok(false)
        })
    }

    fn __repr__(&self) -> String {
        format!(
            "ArduGuidedSession({}, status={})",
            self.shared.label(),
            self.status()
        )
    }
}

#[pyclass(name = "ArduCopterGuidedHandle", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyArduCopterGuidedHandle {
    shared: Arc<PyGuidedSessionShared>,
}

impl PyArduCopterGuidedHandle {
    fn new(shared: Arc<PyGuidedSessionShared>) -> Self {
        Self { shared }
    }

    async fn takeoff_impl(&self, relative_climb_m: f32) -> Result<(), mavkit::VehicleError> {
        let session = self.shared.session_clone().await?;
        let mavkit::GuidedSpecific::Copter(copter) = session.specific() else {
            unreachable!("copter handle should only be constructed for copter guided sessions")
        };
        copter
            .takeoff(relative_climb_target(relative_climb_m))
            .await
    }

    async fn goto_impl(
        &self,
        latitude_deg: f64,
        longitude_deg: f64,
        relative_alt_m: f64,
    ) -> Result<(), mavkit::VehicleError> {
        let session = self.shared.session_clone().await?;
        let mavkit::GuidedSpecific::Copter(copter) = session.specific() else {
            unreachable!("copter handle should only be constructed for copter guided sessions")
        };
        copter
            .goto(geo_point_rel_home(
                latitude_deg,
                longitude_deg,
                relative_alt_m,
            ))
            .await
    }

    async fn goto_msl_impl(
        &self,
        latitude_deg: f64,
        longitude_deg: f64,
        altitude_msl_m: f64,
    ) -> Result<(), mavkit::VehicleError> {
        let session = self.shared.session_clone().await?;
        let mavkit::GuidedSpecific::Copter(copter) = session.specific() else {
            unreachable!("copter handle should only be constructed for copter guided sessions")
        };
        copter
            .goto_msl(geo_point_msl(latitude_deg, longitude_deg, altitude_msl_m))
            .await
    }

    async fn set_velocity_ned_impl(
        &self,
        north_mps: f32,
        east_mps: f32,
        down_mps: f32,
    ) -> Result<(), mavkit::VehicleError> {
        let session = self.shared.session_clone().await?;
        let mavkit::GuidedSpecific::Copter(copter) = session.specific() else {
            unreachable!("copter handle should only be constructed for copter guided sessions")
        };
        copter.set_velocity_ned(north_mps, east_mps, down_mps).await
    }

    async fn hold_impl(&self) -> Result<(), mavkit::VehicleError> {
        let session = self.shared.session_clone().await?;
        let mavkit::GuidedSpecific::Copter(copter) = session.specific() else {
            unreachable!("copter handle should only be constructed for copter guided sessions")
        };
        copter.hold().await
    }
}

#[pymethods]
impl PyArduCopterGuidedHandle {
    fn takeoff<'py>(&self, py: Python<'py>, relative_climb_m: f32) -> PyResult<Bound<'py, PyAny>> {
        py_async_unit!(py, handle = self.clone(); handle.takeoff_impl(relative_climb_m))
    }

    #[pyo3(signature = (*, latitude_deg, longitude_deg, relative_alt_m))]
    fn goto<'py>(
        &self,
        py: Python<'py>,
        latitude_deg: f64,
        longitude_deg: f64,
        relative_alt_m: f64,
    ) -> PyResult<Bound<'py, PyAny>> {
        py_async_unit!(
            py,
            handle = self.clone();
            handle.goto_impl(latitude_deg, longitude_deg, relative_alt_m)
        )
    }

    #[pyo3(signature = (*, latitude_deg, longitude_deg, altitude_msl_m))]
    fn goto_msl<'py>(
        &self,
        py: Python<'py>,
        latitude_deg: f64,
        longitude_deg: f64,
        altitude_msl_m: f64,
    ) -> PyResult<Bound<'py, PyAny>> {
        py_async_unit!(
            py,
            handle = self.clone();
            handle.goto_msl_impl(latitude_deg, longitude_deg, altitude_msl_m)
        )
    }

    fn set_velocity_ned<'py>(
        &self,
        py: Python<'py>,
        north_mps: f32,
        east_mps: f32,
        down_mps: f32,
    ) -> PyResult<Bound<'py, PyAny>> {
        py_async_unit!(
            py,
            handle = self.clone();
            handle.set_velocity_ned_impl(north_mps, east_mps, down_mps)
        )
    }

    fn hold<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        py_async_unit!(py, handle = self.clone(); handle.hold_impl())
    }

    fn __repr__(&self) -> String {
        format!("ArduCopterGuidedHandle({})", self.shared.label())
    }
}

#[pyclass(name = "ArduPlaneGuidedHandle", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyArduPlaneGuidedHandle {
    shared: Arc<PyGuidedSessionShared>,
}

impl PyArduPlaneGuidedHandle {
    fn new(shared: Arc<PyGuidedSessionShared>) -> Self {
        Self { shared }
    }

    async fn reposition_impl(
        &self,
        latitude_deg: f64,
        longitude_deg: f64,
        altitude_msl_m: f64,
    ) -> Result<(), mavkit::VehicleError> {
        let session = self.shared.session_clone().await?;
        let mavkit::GuidedSpecific::Plane(plane) = session.specific() else {
            unreachable!("plane handle should only be constructed for plane guided sessions")
        };
        plane
            .reposition(geo_point_msl(latitude_deg, longitude_deg, altitude_msl_m))
            .await
    }

    async fn reposition_rel_home_impl(
        &self,
        latitude_deg: f64,
        longitude_deg: f64,
        relative_alt_m: f64,
    ) -> Result<(), mavkit::VehicleError> {
        let session = self.shared.session_clone().await?;
        let mavkit::GuidedSpecific::Plane(plane) = session.specific() else {
            unreachable!("plane handle should only be constructed for plane guided sessions")
        };
        plane
            .reposition_rel_home(geo_point_rel_home(
                latitude_deg,
                longitude_deg,
                relative_alt_m,
            ))
            .await
    }
}

#[pymethods]
impl PyArduPlaneGuidedHandle {
    #[pyo3(signature = (*, latitude_deg, longitude_deg, altitude_msl_m))]
    fn reposition<'py>(
        &self,
        py: Python<'py>,
        latitude_deg: f64,
        longitude_deg: f64,
        altitude_msl_m: f64,
    ) -> PyResult<Bound<'py, PyAny>> {
        py_async_unit!(
            py,
            handle = self.clone();
            handle.reposition_impl(latitude_deg, longitude_deg, altitude_msl_m)
        )
    }

    #[pyo3(signature = (*, latitude_deg, longitude_deg, relative_alt_m))]
    fn reposition_rel_home<'py>(
        &self,
        py: Python<'py>,
        latitude_deg: f64,
        longitude_deg: f64,
        relative_alt_m: f64,
    ) -> PyResult<Bound<'py, PyAny>> {
        py_async_unit!(
            py,
            handle = self.clone();
            handle.reposition_rel_home_impl(latitude_deg, longitude_deg, relative_alt_m)
        )
    }

    fn vtol(&self) -> PyResult<Option<PyArduPlaneVtolGuidedHandle>> {
        self.shared.ensure_python_open().map_err(to_py_err)?;
        Ok(
            (self.shared.plane_kind() == Some(mavkit::ArduPlaneKind::Vtol))
                .then_some(PyArduPlaneVtolGuidedHandle::new(self.shared.clone())),
        )
    }

    fn __repr__(&self) -> String {
        format!("ArduPlaneGuidedHandle({})", self.shared.label())
    }
}

#[pyclass(name = "ArduPlaneVtolGuidedHandle", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyArduPlaneVtolGuidedHandle {
    shared: Arc<PyGuidedSessionShared>,
}

impl PyArduPlaneVtolGuidedHandle {
    fn new(shared: Arc<PyGuidedSessionShared>) -> Self {
        Self { shared }
    }

    async fn takeoff_impl(&self, relative_climb_m: f32) -> Result<(), mavkit::VehicleError> {
        let session = self.shared.session_clone().await?;
        let mavkit::GuidedSpecific::Plane(plane) = session.specific() else {
            unreachable!("vtol guided handle should only be constructed for plane sessions")
        };
        let vtol = plane
            .vtol()
            .expect("vtol guided handle requires a VTOL plane session");
        vtol.takeoff(relative_climb_target(relative_climb_m)).await
    }

    async fn hold_impl(&self) -> Result<(), mavkit::VehicleError> {
        let session = self.shared.session_clone().await?;
        let mavkit::GuidedSpecific::Plane(plane) = session.specific() else {
            unreachable!("vtol guided handle should only be constructed for plane sessions")
        };
        let vtol = plane
            .vtol()
            .expect("vtol guided handle requires a VTOL plane session");
        vtol.hold().await
    }
}

#[pymethods]
impl PyArduPlaneVtolGuidedHandle {
    fn takeoff<'py>(&self, py: Python<'py>, relative_climb_m: f32) -> PyResult<Bound<'py, PyAny>> {
        py_async_unit!(py, handle = self.clone(); handle.takeoff_impl(relative_climb_m))
    }

    fn hold<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        py_async_unit!(py, handle = self.clone(); handle.hold_impl())
    }

    fn __repr__(&self) -> String {
        format!("ArduPlaneVtolGuidedHandle({})", self.shared.label())
    }
}

#[pyclass(name = "ArduRoverGuidedHandle", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyArduRoverGuidedHandle {
    shared: Arc<PyGuidedSessionShared>,
}

impl PyArduRoverGuidedHandle {
    fn new(shared: Arc<PyGuidedSessionShared>) -> Self {
        Self { shared }
    }

    async fn drive_to_impl(
        &self,
        latitude_deg: f64,
        longitude_deg: f64,
    ) -> Result<(), mavkit::VehicleError> {
        let session = self.shared.session_clone().await?;
        let mavkit::GuidedSpecific::Rover(rover) = session.specific() else {
            unreachable!("rover handle should only be constructed for rover guided sessions")
        };
        rover
            .drive_to(geo_point_2d(latitude_deg, longitude_deg))
            .await
    }

    async fn drive_impl(
        &self,
        forward_mps: f32,
        turn_rate_dps: f32,
    ) -> Result<(), mavkit::VehicleError> {
        let session = self.shared.session_clone().await?;
        let mavkit::GuidedSpecific::Rover(rover) = session.specific() else {
            unreachable!("rover handle should only be constructed for rover guided sessions")
        };
        rover.drive(forward_mps, turn_rate_dps).await
    }

    async fn hold_impl(&self) -> Result<(), mavkit::VehicleError> {
        let session = self.shared.session_clone().await?;
        let mavkit::GuidedSpecific::Rover(rover) = session.specific() else {
            unreachable!("rover handle should only be constructed for rover guided sessions")
        };
        rover.hold().await
    }
}

#[pymethods]
impl PyArduRoverGuidedHandle {
    #[pyo3(signature = (*, latitude_deg, longitude_deg))]
    fn drive_to<'py>(
        &self,
        py: Python<'py>,
        latitude_deg: f64,
        longitude_deg: f64,
    ) -> PyResult<Bound<'py, PyAny>> {
        py_async_unit!(
            py,
            handle = self.clone();
            handle.drive_to_impl(latitude_deg, longitude_deg)
        )
    }

    fn drive<'py>(
        &self,
        py: Python<'py>,
        forward_mps: f32,
        turn_rate_dps: f32,
    ) -> PyResult<Bound<'py, PyAny>> {
        py_async_unit!(
            py,
            handle = self.clone();
            handle.drive_impl(forward_mps, turn_rate_dps)
        )
    }

    fn hold<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        py_async_unit!(py, handle = self.clone(); handle.hold_impl())
    }

    fn __repr__(&self) -> String {
        format!("ArduRoverGuidedHandle({})", self.shared.label())
    }
}

#[pyclass(name = "ArduSubGuidedHandle", frozen, skip_from_py_object)]
#[derive(Clone)]
pub struct PyArduSubGuidedHandle {
    shared: Arc<PyGuidedSessionShared>,
}

impl PyArduSubGuidedHandle {
    fn new(shared: Arc<PyGuidedSessionShared>) -> Self {
        Self { shared }
    }

    async fn goto_depth_impl(
        &self,
        latitude_deg: f64,
        longitude_deg: f64,
        depth_m: f32,
    ) -> Result<(), mavkit::VehicleError> {
        let session = self.shared.session_clone().await?;
        let mavkit::GuidedSpecific::Sub(sub) = session.specific() else {
            unreachable!("sub handle should only be constructed for sub guided sessions")
        };
        sub.goto_depth(sub_goto_depth_target(latitude_deg, longitude_deg, depth_m))
            .await
    }

    async fn set_velocity_body_impl(
        &self,
        forward_mps: f32,
        lateral_mps: f32,
        vertical_mps: f32,
        yaw_rate_dps: f32,
    ) -> Result<(), mavkit::VehicleError> {
        let session = self.shared.session_clone().await?;
        let mavkit::GuidedSpecific::Sub(sub) = session.specific() else {
            unreachable!("sub handle should only be constructed for sub guided sessions")
        };
        sub.set_velocity_body(forward_mps, lateral_mps, vertical_mps, yaw_rate_dps)
            .await
    }

    async fn hold_impl(&self) -> Result<(), mavkit::VehicleError> {
        let session = self.shared.session_clone().await?;
        let mavkit::GuidedSpecific::Sub(sub) = session.specific() else {
            unreachable!("sub handle should only be constructed for sub guided sessions")
        };
        sub.hold().await
    }
}

#[pymethods]
impl PyArduSubGuidedHandle {
    #[pyo3(signature = (*, latitude_deg, longitude_deg, depth_m))]
    fn goto_depth<'py>(
        &self,
        py: Python<'py>,
        latitude_deg: f64,
        longitude_deg: f64,
        depth_m: f32,
    ) -> PyResult<Bound<'py, PyAny>> {
        py_async_unit!(
            py,
            handle = self.clone();
            handle.goto_depth_impl(latitude_deg, longitude_deg, depth_m)
        )
    }

    fn set_velocity_body<'py>(
        &self,
        py: Python<'py>,
        forward_mps: f32,
        lateral_mps: f32,
        vertical_mps: f32,
        yaw_rate_dps: f32,
    ) -> PyResult<Bound<'py, PyAny>> {
        py_async_unit!(
            py,
            handle = self.clone();
            handle.set_velocity_body_impl(forward_mps, lateral_mps, vertical_mps, yaw_rate_dps)
        )
    }

    fn hold<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        py_async_unit!(py, handle = self.clone(); handle.hold_impl())
    }

    fn __repr__(&self) -> String {
        format!("ArduSubGuidedHandle({})", self.shared.label())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mavkit::dialect;
    use mavlink::{AsyncMavConnection, MavHeader, MavlinkVersion};
    use std::sync::{Arc, Mutex};
    use std::time::Duration;
    use tokio::sync::mpsc;
    use tokio::time::timeout;

    type SentMessages = Arc<Mutex<Vec<(MavHeader, dialect::MavMessage)>>>;

    struct MockConnection {
        recv_rx: tokio::sync::Mutex<mpsc::Receiver<(MavHeader, dialect::MavMessage)>>,
        sent: SentMessages,
    }

    impl MockConnection {
        fn new(rx: mpsc::Receiver<(MavHeader, dialect::MavMessage)>) -> (Self, SentMessages) {
            let sent = Arc::new(Mutex::new(Vec::new()));
            (
                Self {
                    recv_rx: tokio::sync::Mutex::new(rx),
                    sent: sent.clone(),
                },
                sent,
            )
        }
    }

    impl AsyncMavConnection<dialect::MavMessage> for MockConnection {
        fn recv<'life0, 'async_trait>(
            &'life0 self,
        ) -> std::pin::Pin<
            Box<
                dyn std::future::Future<
                        Output = Result<
                            (MavHeader, dialect::MavMessage),
                            mavlink::error::MessageReadError,
                        >,
                    > + Send
                    + 'async_trait,
            >,
        >
        where
            'life0: 'async_trait,
            Self: 'async_trait,
        {
            Box::pin(async move {
                let mut rx = self.recv_rx.lock().await;
                match rx.recv().await {
                    Some(message) => Ok(message),
                    None => Err(mavlink::error::MessageReadError::Io(std::io::Error::new(
                        std::io::ErrorKind::ConnectionReset,
                        "mock connection closed",
                    ))),
                }
            })
        }

        fn recv_raw<'life0, 'async_trait>(
            &'life0 self,
        ) -> std::pin::Pin<
            Box<
                dyn std::future::Future<
                        Output = Result<
                            mavlink::MAVLinkMessageRaw,
                            mavlink::error::MessageReadError,
                        >,
                    > + Send
                    + 'async_trait,
            >,
        >
        where
            'life0: 'async_trait,
            Self: 'async_trait,
        {
            Box::pin(async move {
                let (header, message) = self.recv().await?;
                let mut raw = mavlink::MAVLinkV2MessageRaw::new();
                raw.serialize_message(header, &message);
                Ok(mavlink::MAVLinkMessageRaw::V2(raw))
            })
        }

        fn send<'life0, 'life1, 'life2, 'async_trait>(
            &'life0 self,
            header: &'life1 MavHeader,
            data: &'life2 dialect::MavMessage,
        ) -> std::pin::Pin<
            Box<
                dyn std::future::Future<Output = Result<usize, mavlink::error::MessageWriteError>>
                    + Send
                    + 'async_trait,
            >,
        >
        where
            'life0: 'async_trait,
            'life1: 'async_trait,
            'life2: 'async_trait,
            Self: 'async_trait,
        {
            let header = *header;
            let data = data.clone();
            Box::pin(async move {
                self.sent.lock().unwrap().push((header, data));
                Ok(0)
            })
        }

        fn set_protocol_version(&mut self, _version: MavlinkVersion) {}

        fn protocol_version(&self) -> MavlinkVersion {
            MavlinkVersion::V2
        }

        fn set_allow_recv_any_version(&mut self, _allow: bool) {}

        fn allow_recv_any_version(&self) -> bool {
            true
        }
    }

    fn default_header() -> MavHeader {
        MavHeader {
            system_id: 1,
            component_id: 1,
            sequence: 0,
        }
    }

    fn heartbeat_msg_with_mode(mavtype: dialect::MavType, custom_mode: u32) -> dialect::MavMessage {
        dialect::MavMessage::HEARTBEAT(dialect::HEARTBEAT_DATA {
            custom_mode,
            mavtype,
            autopilot: dialect::MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA,
            base_mode: dialect::MavModeFlag::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            system_status: dialect::MavState::MAV_STATE_STANDBY,
            mavlink_version: 3,
        })
    }

    fn fast_config() -> mavkit::VehicleConfig {
        mavkit::VehicleConfig {
            connect_timeout: Duration::from_millis(150),
            command_timeout: Duration::from_millis(50),
            command_completion_timeout: Duration::from_millis(150),
            auto_request_home: false,
            ..mavkit::VehicleConfig::default()
        }
    }

    async fn connect_mock_vehicle(mavtype: dialect::MavType, custom_mode: u32) -> mavkit::Vehicle {
        let (msg_tx, msg_rx) = mpsc::channel(16);
        let (conn, _sent) = MockConnection::new(msg_rx);
        let connect_task = tokio::spawn(async move {
            mavkit::Vehicle::from_connection(Box::new(conn), fast_config()).await
        });

        msg_tx
            .send((
                default_header(),
                heartbeat_msg_with_mode(mavtype, custom_mode),
            ))
            .await
            .expect("heartbeat should be delivered");

        timeout(Duration::from_millis(250), connect_task)
            .await
            .expect("connect should complete")
            .expect("connect task should join")
            .expect("mock vehicle should connect")
    }

    #[tokio::test(flavor = "current_thread")]
    async fn close_is_idempotent_and_blocks_future_narrowing() {
        let vehicle = connect_mock_vehicle(dialect::MavType::MAV_TYPE_QUADROTOR, 4).await;
        let session = vehicle
            .ardupilot()
            .guided()
            .await
            .expect("guided session should acquire");
        let py_session = PyArduGuidedSession::new(session, "sys=1, comp=1".to_string());

        py_session
            .close_impl()
            .await
            .expect("first close should succeed");
        py_session
            .close_impl()
            .await
            .expect("second close should also succeed");

        assert!(matches!(
            py_session.copter_impl(),
            Err(mavkit::VehicleError::OperationConflict {
                conflicting_domain,
                conflicting_op,
            }) if conflicting_domain == "ardupilot_guided" && conflicting_op == "session_closed"
        ));
    }

    #[tokio::test(flavor = "current_thread")]
    async fn retained_guided_handle_operations_fail_after_python_close() {
        let vehicle = connect_mock_vehicle(dialect::MavType::MAV_TYPE_GROUND_ROVER, 15).await;
        let session = vehicle
            .ardupilot()
            .guided()
            .await
            .expect("guided session should acquire");
        let py_session = PyArduGuidedSession::new(session, "sys=1, comp=1".to_string());
        let rover = py_session
            .rover_impl()
            .expect("rover narrowing should succeed before close")
            .expect("rover session should expose rover handle");

        py_session.close_impl().await.expect("close should succeed");

        assert!(matches!(
            rover.drive_impl(1.0, 0.0).await,
            Err(mavkit::VehicleError::OperationConflict {
                conflicting_domain,
                conflicting_op,
            }) if conflicting_domain == "ardupilot_guided" && conflicting_op == "session_closed"
        ));
    }
}
