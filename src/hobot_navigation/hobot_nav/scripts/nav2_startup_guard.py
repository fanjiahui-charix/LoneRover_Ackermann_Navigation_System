#!/usr/bin/env python3

"""Boundedly complete a partially failed Nav2 lifecycle startup.

The Humble lifecycle manager can abort its whole startup sequence when a single
post-transition get_state response misses its fixed two-second window.  It
does not retry, which can leave one server active and the remaining servers
inactive. This guard starts after the normal manager and only supplies missing
configure/activate transitions. It exits as soon as the configured Nav2 nodes
are active, so it has no task-control role.
"""

import sys
import time

from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
import rclpy
from rclpy.node import Node


class Nav2StartupGuard(Node):
    def __init__(self):
        super().__init__('nav2_startup_guard')
        self.declare_parameter('node_names', [
            '/controller_server', '/planner_server', '/velocity_smoother'])
        self.declare_parameter('repair_timeout_sec', 45.0)
        self.declare_parameter('request_timeout_sec', 3.0)
        self.declare_parameter('retry_period_sec', 1.0)
        self.node_names = list(self.get_parameter('node_names').value)
        self.repair_timeout = max(
            5.0, float(self.get_parameter('repair_timeout_sec').value))
        self.request_timeout = max(
            0.5, float(self.get_parameter('request_timeout_sec').value))
        self.retry_period = max(
            0.1, float(self.get_parameter('retry_period_sec').value))

    def _call(self, service_type, service_name, request):
        client = self.create_client(service_type, service_name)
        try:
            if not client.wait_for_service(timeout_sec=self.request_timeout):
                return None
            future = client.call_async(request)
            rclpy.spin_until_future_complete(
                self, future, timeout_sec=self.request_timeout)
            if not future.done() or future.cancelled() or future.exception() is not None:
                try:
                    client.remove_pending_request(future)
                except (AttributeError, RuntimeError):
                    pass
                return None
            return future.result()
        finally:
            self.destroy_client(client)

    def _state(self, node_name):
        response = self._call(
            GetState, f'{node_name}/get_state', GetState.Request())
        return None if response is None else int(response.current_state.id)

    def _transition(self, node_name, transition_id):
        request = ChangeState.Request()
        request.transition.id = transition_id
        response = self._call(
            ChangeState, f'{node_name}/change_state', request)
        return response is not None and bool(response.success)

    def repair(self):
        deadline = time.monotonic() + self.repair_timeout
        last_report = 0.0
        while rclpy.ok() and time.monotonic() < deadline:
            states = {name: self._state(name) for name in self.node_names}
            if all(state == State.PRIMARY_STATE_ACTIVE for state in states.values()):
                self.get_logger().info(
                    'Nav2 startup complete; guard exits before navigation runtime')
                return True

            now = time.monotonic()
            if now - last_report >= 2.0:
                detail = ','.join(
                    f'{name}={state if state is not None else "unavailable"}'
                    for name, state in states.items())
                self.get_logger().warn(f'Nav2 startup repair pending: {detail}')
                last_report = now

            for name in self.node_names:
                state = states[name]
                if state == State.PRIMARY_STATE_UNCONFIGURED:
                    self.get_logger().warn(f'Configuring missing Nav2 node {name}')
                    self._transition(name, Transition.TRANSITION_CONFIGURE)
                    break
                if state == State.PRIMARY_STATE_INACTIVE:
                    self.get_logger().warn(f'Activating inactive Nav2 node {name}')
                    self._transition(name, Transition.TRANSITION_ACTIVATE)
                    break
                if state == State.PRIMARY_STATE_FINALIZED:
                    self.get_logger().error(
                        f'Cannot repair finalized Nav2 node {name}')
                    return False
            time.sleep(self.retry_period)

        self.get_logger().error(
            'Nav2 startup repair timed out; navigation readiness remains closed')
        return False


def main(args=None):
    rclpy.init(args=args)
    node = Nav2StartupGuard()
    try:
        success = node.repair()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0 if success else 2


if __name__ == '__main__':
    sys.exit(main())
