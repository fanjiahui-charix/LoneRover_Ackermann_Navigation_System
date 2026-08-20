import ast
from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
NODE = PACKAGE / 'adaptive_speed_limiter/node.py'


def test_perf_and_reverse_safety_are_debug_only_low_rate_publishers():
    source = NODE.read_text(encoding='utf-8')
    ast.parse(source)
    assert "declare('capture_diagnostics_enabled', False)" in source
    assert "String, '/race/perf_trace', 10" in source
    assert "String, '/race/reverse_safety_state', 10" in source
    assert 'self.create_timer(\n                1.0' in source
    assert 'self.create_timer(\n                0.2' in source
    assert 'if not self._reverse_only:' in source

    safety_body = source.split(
        '    def _publish_reverse_safety_state(self)', 1
    )[1].split('    @staticmethod', 1)[0]
    assert 'get_logger()' not in safety_body


def test_safety_snapshot_reuses_existing_context_and_executor_contract():
    source = NODE.read_text(encoding='utf-8')
    assert 'grid = self._grid' in source
    assert 'odom_pose = self._odom_pose' in source
    assert 'self._footprint_clearance(' in source
    assert "'footprint_safe': (" in source
    assert "'footprint_clearance_m': clearance" in source
    assert 'MultiThreadedExecutor(num_threads=2)' in source
    assert source.count("'/local_costmap/costmap_raw'") == 1


def test_perf_trace_contains_requested_aggregate_fields():
    source = NODE.read_text(encoding='utf-8')
    for field in (
            "'callback_mean_sec':",
            "'callback_max_sec':",
            "'input_message_age_sec':",
            "'input_to_output_delay_sec':",
            "'tf_lookup_mean_sec':",
            "'tf_lookup_max_sec':",
            "'action_wait_mean_sec':",
            "'main_loop_period_max_sec':",
            "'longest_no_response_sec':"):
        assert field in source
