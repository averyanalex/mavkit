use crate::error::VehicleError;
use crate::mission::{MissionProtocolScope, OperationReservation};
use crate::observation::{ObservationHandle, ObservationWriter};
use crate::types::SyncState;
use std::sync::{Arc, Mutex};

pub(crate) trait StoredPlanState:
    Clone + Default + PartialEq + Send + Sync + 'static
{
    type Plan;
    type OperationKind;

    fn set_active_op(&mut self, kind: Option<Self::OperationKind>);
    fn set_plan(&mut self, plan: Self::Plan);
    fn set_sync(&mut self, sync: SyncState);
    fn cleared_plan() -> Self::Plan;
}

#[derive(Clone)]
pub(crate) struct StoredPlanDomain<S: StoredPlanState> {
    inner: Arc<StoredPlanDomainInner<S>>,
}

struct StoredPlanDomainInner<S: StoredPlanState> {
    state_writer: ObservationWriter<S>,
    state: ObservationHandle<S>,
    latest_state: Mutex<S>,
}

impl<S: StoredPlanState> StoredPlanDomain<S> {
    pub(crate) fn new() -> Self {
        let (state_writer, state) = ObservationHandle::watch();
        let latest = S::default();
        let _ = state_writer.publish(latest.clone());

        Self {
            inner: Arc::new(StoredPlanDomainInner {
                state_writer,
                state,
                latest_state: Mutex::new(latest),
            }),
        }
    }

    pub(crate) fn state(&self) -> ObservationHandle<S> {
        self.inner.state.clone()
    }

    pub(crate) fn begin_operation(
        &self,
        scope: &MissionProtocolScope,
        domain: &'static str,
        kind: S::OperationKind,
        op_name: &'static str,
    ) -> Result<OperationReservation, VehicleError> {
        let reservation = scope.begin_operation(domain, op_name)?;
        self.update_state(|state| {
            state.set_active_op(Some(kind));
        });
        Ok(reservation)
    }

    pub(crate) fn finish_operation(&self, scope: &MissionProtocolScope, op_id: u64) {
        scope.finish_operation(op_id);
        self.update_state(|state| {
            state.set_active_op(None);
        });
    }

    pub(crate) fn note_operation_error(&self) {
        self.update_state(|state| {
            state.set_sync(SyncState::PossiblyStale);
        });
    }

    pub(crate) fn note_upload_success(&self, plan: S::Plan) {
        self.update_state(|state| {
            state.set_plan(plan);
            state.set_sync(SyncState::Current);
        });
    }

    pub(crate) fn note_download_success(&self, plan: S::Plan) {
        self.update_state(|state| {
            state.set_plan(plan);
            state.set_sync(SyncState::Current);
        });
    }

    pub(crate) fn note_clear_success(&self) {
        self.update_state(|state| {
            state.set_plan(S::cleared_plan());
            state.set_sync(SyncState::Current);
        });
    }

    pub(crate) fn update_state(&self, edit: impl FnOnce(&mut S)) {
        let mut latest = self.inner.latest_state.lock().unwrap();
        let mut next = latest.clone();
        edit(&mut next);
        if *latest != next {
            *latest = next.clone();
            let _ = self.inner.state_writer.publish(next);
        }
    }
}
