#!/usr/bin/env python3
"""Activate a lifecycle node once, without depending on transition notifications."""

import argparse
import math
import time

import rclpy
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.utilities import remove_ros_args


def activate(node, target, timeout):
    """Use a wall-clock deadline, including service discovery and responses."""
    deadline = time.monotonic() + timeout

    def remaining():
        if not rclpy.ok(context=node.context):
            raise ExternalShutdownException()
        left = deadline - time.monotonic()
        if left <= 0:
            raise TimeoutError(f'{target}: startup timed out after {timeout:g}s')
        return left

    def call(client, request):
        while not client.wait_for_service(timeout_sec=min(0.1, remaining())):
            pass
        future = client.call_async(request)
        try:
            while not future.done():
                executor.spin_once(timeout_sec=min(0.1, remaining()))
            remaining()
            return future.result()
        finally:
            if not future.done():
                future.cancel()

    remaining()
    executor = SingleThreadedExecutor(context=node.context)
    executor.add_node(node)
    get_state = node.create_client(GetState, f'{target}/get_state')
    change_state = node.create_client(ChangeState, f'{target}/change_state')
    attempted = set()
    transitions = {
        State.PRIMARY_STATE_UNCONFIGURED: Transition.TRANSITION_CONFIGURE,
        State.PRIMARY_STATE_INACTIVE: Transition.TRANSITION_ACTIVATE,
    }
    try:
        while True:
            state = call(get_state, GetState.Request()).current_state
            if state.id == State.PRIMARY_STATE_ACTIVE:
                node.get_logger().info(f'{target}: active')
                return
            transition = transitions.get(state.id)
            if transition is not None:
                # Do not hide callback failures by repeatedly configuring/activating.
                if transition in attempted:
                    raise RuntimeError(f'{target}: returned to {state.label} after transition')
                attempted.add(transition)
                request = ChangeState.Request()
                request.transition.id = transition
                if not call(change_state, request).success:
                    raise RuntimeError(f'{target}: transition {transition} failed')
            elif state.id in (
                State.TRANSITION_STATE_CONFIGURING,
                State.TRANSITION_STATE_ACTIVATING,
            ):
                executor.spin_once(timeout_sec=min(0.1, remaining()))
            else:
                raise RuntimeError(f'{target}: unexpected state {state.label} ({state.id})')
    finally:
        executor.shutdown()
        node.destroy_client(get_state)
        node.destroy_client(change_state)


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('target', help='Lifecycle node name, relative to this node namespace')
    parser.add_argument('--timeout', type=float, default=60.0)
    options = parser.parse_args(remove_ros_args(args=args)[1:])
    if not math.isfinite(options.timeout) or options.timeout <= 0:
        parser.error('--timeout must be finite and positive')
    rclpy.init(args=args)
    node = rclpy.create_node('localization_startup')
    try:
        activate(node, options.target.rstrip('/'), options.timeout)
        return 0
    except (KeyboardInterrupt, ExternalShutdownException):
        return 0
    except (RuntimeError, TimeoutError) as error:
        node.get_logger().error(str(error))
        return 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
