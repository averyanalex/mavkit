use crate::error::VehicleError;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use tokio_util::sync::CancellationToken;

#[derive(Clone)]
pub(crate) struct MissionProtocolScope {
    inner: Arc<MissionProtocolScopeInner>,
}

struct MissionProtocolScopeInner {
    active_operation: Mutex<Option<ActiveOperation>>,
    operation_id: AtomicU64,
}

struct ActiveOperation {
    id: u64,
    domain: &'static str,
    op_name: &'static str,
}

pub(crate) struct OperationReservation {
    pub(crate) id: u64,
    pub(crate) cancel: CancellationToken,
}

impl MissionProtocolScope {
    pub(crate) fn new() -> Self {
        Self {
            inner: Arc::new(MissionProtocolScopeInner {
                active_operation: Mutex::new(None),
                operation_id: AtomicU64::new(1),
            }),
        }
    }

    pub(crate) fn begin_operation(
        &self,
        domain: &'static str,
        op_name: &'static str,
    ) -> Result<OperationReservation, VehicleError> {
        let mut active = self.inner.active_operation.lock().unwrap();
        if let Some(conflict) = active.as_ref() {
            return Err(VehicleError::OperationConflict {
                conflicting_domain: conflict.domain.to_string(),
                conflicting_op: conflict.op_name.to_string(),
            });
        }

        let id = self.inner.operation_id.fetch_add(1, Ordering::Relaxed);
        let cancel = CancellationToken::new();
        *active = Some(ActiveOperation {
            id,
            domain,
            op_name,
        });

        Ok(OperationReservation { id, cancel })
    }

    pub(crate) fn finish_operation(&self, op_id: u64) {
        let mut active = self.inner.active_operation.lock().unwrap();
        if active
            .as_ref()
            .is_some_and(|active_op| active_op.id == op_id)
        {
            *active = None;
        }
    }
}
