use std::time::Duration;

use mavlink::MavHeader;
use tokio::sync::mpsc;
use tokio::task::JoinHandle;
use tokio_util::sync::CancellationToken;

use crate::command::Command;
use crate::config::VehicleConfig;
use crate::dialect;
use crate::event_loop::run_event_loop;
use crate::state::{self, StateChannels};
use crate::test_support::{MockConnection, SentMessages};

pub(super) fn fast_event_loop_test_config() -> VehicleConfig {
    VehicleConfig {
        command_timeout: Duration::from_millis(50),
        command_completion_timeout: Duration::from_millis(250),
        retry_policy: crate::mission::RetryPolicy {
            request_timeout_ms: 50,
            item_timeout_ms: 50,
            max_retries: 2,
        },
        auto_request_home: false,
        ..VehicleConfig::default()
    }
}

#[derive(Clone)]
pub(super) struct EventLoopOptions {
    pub(super) config: VehicleConfig,
    pub(super) message_capacity: usize,
    pub(super) command_capacity: usize,
}

impl Default for EventLoopOptions {
    fn default() -> Self {
        Self {
            config: fast_event_loop_test_config(),
            message_capacity: 64,
            command_capacity: 16,
        }
    }
}

pub(super) struct EventLoopHarness {
    pub(super) msg_tx: mpsc::Sender<(MavHeader, dialect::MavMessage)>,
    pub(super) cmd_tx: mpsc::Sender<Command>,
    pub(super) channels: StateChannels,
    pub(super) sent: SentMessages,
    handle: JoinHandle<()>,
}

impl EventLoopHarness {
    pub(super) async fn start(options: EventLoopOptions) -> Self {
        let (msg_tx, msg_rx) = mpsc::channel(options.message_capacity);
        let (conn, sent) = MockConnection::new(msg_rx);
        let (cmd_tx, cmd_rx) = mpsc::channel(options.command_capacity);
        let (writers, channels) = state::create_channels();
        let cancel = CancellationToken::new();

        let handle = tokio::spawn(async move {
            run_event_loop(Box::new(conn), cmd_rx, writers, options.config, cancel).await;
        });

        Self {
            msg_tx,
            cmd_tx,
            channels,
            sent,
            handle,
        }
    }

    pub(super) async fn shutdown(self) {
        self.cmd_tx
            .send(Command::Shutdown)
            .await
            .expect("shutdown command should be delivered");
        self.handle
            .await
            .expect("event loop task should shut down cleanly");
    }
}
