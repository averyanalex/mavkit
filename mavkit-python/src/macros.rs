/// Generates a PyO3 subscription class for an `ObservationSubscription<$rust_ty>`.
///
/// The generated struct exposes `recv()`, `__aiter__()`, and `__anext__()` so it can
/// be iterated with `async for` in Python.  Every call to `recv` (or `__anext__`) locks
/// the inner subscription, awaits the next value, converts it with `<$py_ty>::from`,
/// and raises `StopAsyncIteration` when the sender side is dropped.
macro_rules! py_subscription {
    ($struct_name:ident, $rust_ty:ty, $py_ty:ty, $py_name:literal, $closed_msg:literal) => {
        #[pyo3::pyclass(name = $py_name, frozen, skip_from_py_object)]
        pub struct $struct_name {
            pub(crate) inner:
                std::sync::Arc<tokio::sync::Mutex<mavkit::ObservationSubscription<$rust_ty>>>,
        }

        #[pyo3::pymethods]
        impl $struct_name {
            fn recv<'py>(
                &self,
                py: pyo3::Python<'py>,
            ) -> pyo3::PyResult<pyo3::Bound<'py, pyo3::types::PyAny>> {
                let inner = self.inner.clone();
                pyo3_async_runtimes::tokio::future_into_py(py, async move {
                    let mut guard = inner.lock().await;
                    match guard.recv().await {
                        Some(value) => Ok(<$py_ty>::from(value)),
                        None => Err(pyo3::exceptions::PyStopAsyncIteration::new_err($closed_msg)),
                    }
                })
            }

            fn __aiter__(slf: pyo3::PyRef<'_, Self>) -> pyo3::PyRef<'_, Self> {
                slf
            }

            fn __anext__<'py>(
                slf: pyo3::PyRef<'py, Self>,
                py: pyo3::Python<'py>,
            ) -> pyo3::PyResult<pyo3::Bound<'py, pyo3::types::PyAny>> {
                slf.recv(py)
            }
        }
    };
}

pub(crate) use py_subscription;
