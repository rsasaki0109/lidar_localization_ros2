"""Exercise startup against real ROS services without any transition publisher."""
import sys
import threading
import time
from pathlib import Path

import pytest

rclpy = pytest.importorskip('rclpy')
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.context import Context
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from start_lifecycle_node import activate


@pytest.fixture
def services():
    context = Context()
    rclpy.init(context=context)
    server = rclpy.create_node('fake_lifecycle', namespace='/startup_test', context=context)
    client = rclpy.create_node('startup_client', namespace='/startup_test', context=context)
    state = {'id': 1, 'success': True, 'calls': [], 'delay': 0, 'stuck': False}

    def get_state(request, response):
        time.sleep(state.get('read_delay', 0))
        response.current_state.id = state['id']
        response.current_state.label = str(state['id'])
        return response

    def change_state(request, response):
        state['calls'].append(request.transition.id)
        time.sleep(state['delay'])
        response.success = state['success']
        if response.success and not state['stuck']:
            state['id'] = {1: 2, 3: 3}[request.transition.id]
        return response

    server.create_service(GetState, 'target/get_state', get_state)
    server.create_service(ChangeState, 'target/change_state', change_state)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(server)
    def spin():
        try:
            executor.spin()
        except ExternalShutdownException:
            pass
    thread = threading.Thread(target=spin)
    thread.start()
    yield client, state
    executor.shutdown()
    thread.join(timeout=3)
    assert not thread.is_alive()
    client.destroy_node()
    server.destroy_node()
    context.try_shutdown()


@pytest.mark.parametrize('initial, expected', [(1, [1, 3]), (2, [3]), (3, [])])
def test_no_notifications_and_relative_namespace(services, initial, expected):
    client, state = services
    state['id'] = initial
    activate(client, 'target', 3)
    assert state['id'] == 3
    assert state['calls'] == expected


@pytest.mark.parametrize('initial', [1, 2])
def test_failed_callback_is_not_retried(services, initial):
    client, state = services
    state.update(id=initial, success=False)
    with pytest.raises(RuntimeError, match='failed'):
        activate(client, 'target', 3)
    assert len(state['calls']) == 1


def test_success_response_still_requires_active_state(services):
    client, state = services
    state['stuck'] = True
    with pytest.raises(RuntimeError, match='returned to'):
        activate(client, 'target', 3)
    assert state['calls'] == [1]


@pytest.mark.parametrize('initial', [0, 4, 12, 15])
def test_unexpected_state_fails_without_mutation(services, initial):
    client, state = services
    state['id'] = initial
    with pytest.raises(RuntimeError, match='unexpected state'):
        activate(client, 'target', 3)
    assert state['calls'] == []


@pytest.mark.parametrize('mode', ['missing', 'transitioning', 'response'])
def test_deadline_includes_discovery_transition_and_response(services, mode):
    client, state = services
    if mode == 'transitioning':
        state['id'] = 10
    if mode == 'response':
        state['delay'] = 0.8
    started = time.monotonic()
    with pytest.raises(TimeoutError):
        activate(client, 'missing' if mode == 'missing' else 'target', 0.5)
    assert time.monotonic() - started < 1.2


def test_shutdown_stops_waiting(services):
    client, state = services
    client.context.shutdown()
    with pytest.raises(ExternalShutdownException):
        activate(client, 'missing', 3)
    assert state['calls'] == []


@pytest.mark.parametrize('mode', ['first_read', 'all_reads', 'transition'])
def test_lost_response_is_bounded_and_mutations_are_not_retried(services, monkeypatch, mode):
    from rclpy.task import Future

    client, state = services
    original = client.create_client
    calls = {'read': 0, 'transition': 0}
    dropped = []

    def create_client(service_type, *args, **kwargs):
        service = original(service_type, *args, **kwargs)
        real_call = service.call_async
        kind = 'read' if service_type is GetState else 'transition'

        def call(request):
            calls[kind] += 1
            delivered = real_call(request)
            drop = ((kind == 'read' and
                     (mode == 'all_reads' or (mode == 'first_read' and calls[kind] == 1)))
                    or (kind == 'transition' and mode == 'transition'))
            # Server still processes the request, but the caller never gets its result.
            if drop:
                pending = Future()
                dropped.append(pending)
                return pending
            return delivered

        monkeypatch.setattr(service, 'call_async', call)
        return service

    monkeypatch.setattr(client, 'create_client', create_client)
    started = time.monotonic()
    if mode == 'first_read':
        activate(client, 'target', 3)
        assert state['id'] == 3
        assert state['calls'] == [1, 3]
        assert calls['read'] >= 4
    else:
        with pytest.raises(TimeoutError):
            activate(client, 'target', 2.5)
        assert state['calls'] == ([1] if mode == 'transition' else [])
        assert calls['transition'] == (1 if mode == 'transition' else 0)
        if mode == 'all_reads':
            assert calls['read'] >= 2
    assert time.monotonic() - started < 3.8
    assert all(future.cancelled() for future in dropped)


def test_slow_read_responses_remain_usable(services):
    client, state = services
    state['read_delay'] = 1.2
    activate(client, 'target', 6)
    assert state['id'] == 3
    assert state['calls'] == [1, 3]
