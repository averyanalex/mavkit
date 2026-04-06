use crate::error::VehicleError;
use crate::shared_state::recover_lock;
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

#[derive(Debug)]
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
        let mut active = recover_lock(&self.inner.active_operation);
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
        let mut active = recover_lock(&self.inner.active_operation);
        if active
            .as_ref()
            .is_some_and(|active_op| active_op.id == op_id)
        {
            *active = None;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn finish_operation_clears_only_matching_reservation() {
        let scope = MissionProtocolScope::new();
        let reservation = scope
            .begin_operation("mission", "upload")
            .expect("first reservation should succeed");

        let conflict = scope
            .begin_operation("params", "download_all")
            .expect_err("second reservation should conflict while mission upload is active");
        assert!(matches!(
            conflict,
            VehicleError::OperationConflict {
                conflicting_domain,
                conflicting_op,
            } if conflicting_domain == "mission" && conflicting_op == "upload"
        ));

        scope.finish_operation(reservation.id + 1);

        let stale_id_conflict = scope
            .begin_operation("params", "download_all")
            .expect_err("stale finish should not clear the active mission reservation");
        assert!(matches!(
            stale_id_conflict,
            VehicleError::OperationConflict {
                conflicting_domain,
                conflicting_op,
            } if conflicting_domain == "mission" && conflicting_op == "upload"
        ));

        scope.finish_operation(reservation.id);

        let next = scope
            .begin_operation("params", "download_all")
            .expect("matching finish should clear the reservation");
        assert_eq!(next.id, reservation.id + 1);
    }
}
