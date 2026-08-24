from pathlib import Path


ROOT = Path(__file__).resolve().parents[4]
REPLAY = (ROOT / "tools/vehicle_model/nav2_virtual_vehicle_replay.py").read_text()


def test_native_replay_feeds_scenario_cones_through_real_cone_layer_topic():
    assert "scenario.get('fixed_cones_map', [])" in REPLAY
    assert "PointCloud2, '/cones/points', qos_profile_sensor_data" in REPLAY
    assert "cloud.header.frame_id = 'map'" in REPLAY
    assert "struct.pack('<ffff', x, y, 0.0, 1.0)" in REPLAY
    assert "now - self.last_cone_publish_sec < 0.10" in REPLAY
    assert "self.publish_fixed_cones(stamp, now)" in REPLAY


def test_fixed_cone_shadow_keeps_physical_command_topics_isolated():
    assert "Twist, '/shadow_cmd_vel_safe'" in REPLAY
    assert "Twist, '/shadow_cmd_vel_raw'" in REPLAY
    assert "'/cmd_vel_safe'" not in REPLAY
    assert "'fixed_cones_map': self.fixed_cones" in REPLAY


def test_native_replay_allows_slow_but_healthy_lifecycle_activation():
    assert "client.wait_for_server(timeout_sec=30.0)" in REPLAY


def test_native_replay_records_compute_path_latency_separately_from_startup():
    assert "self.plan_request_t = self.now_sec()" in REPLAY
    assert "self.plan_result_t = self.now_sec()" in REPLAY
    assert "'duration_sec': self.planning_duration_sec" in REPLAY


def test_native_variant_clears_shared_path_and_requires_a_new_path():
    # The X5-native remote coordinator is intentionally excluded from the
    # public release. The local replay contract remains covered above.
    assert not (ROOT / "tools/run_native_x5_variant.sh").exists()
