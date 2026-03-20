from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]


class TestExamples:
    def test_monitor_link_state_examples_exist_in_both_languages(self):
        assert (REPO_ROOT / "examples" / "monitor_link_state.rs").exists()
        assert (
            REPO_ROOT / "mavkit-python" / "examples" / "monitor_link_state.py"
        ).exists()
