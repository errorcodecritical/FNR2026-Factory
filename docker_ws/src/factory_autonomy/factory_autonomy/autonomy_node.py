#!/usr/bin/env python3
"""RobotAtFactory autonomous production loop.

The node talks to the competition server described in chapter 6 of the rules.
It waits for the run to start by repeatedly asking for the incoming warehouse
state, refreshes warehouse/machine state, chooses the next transport move, and
sends pickup/drop-off goals to Nav2.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
import socket
import time
from typing import List, Optional

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from .crate_protocol import (
    FactorySnapshot,
    PART_EMPTY,
    PART_RESPONSE_REQUESTS,
    REQUEST_INCOMING,
    REQUEST_MACHINE_A,
    REQUEST_MACHINE_B,
    REQUEST_OUTGOING,
    REQUEST_TIME_LEFT,
    RESPONSE_STOP,
    parse_server_response,
)
from .scheduler import FactoryPlanner, TransportTask
from .waypoints import Waypoint, WaypointStore, load_waypoints_file


STATE_WAITING_FOR_START = 'waiting_for_start'
STATE_REFRESHING_FACTORY = 'refreshing_factory'
STATE_IDLE = 'idle'
STATE_WAITING_FOR_NAV2 = 'waiting_for_nav2'
STATE_NAVIGATING_TO_PICKUP = 'navigating_to_pickup'
STATE_PICKUP_PAUSE = 'pickup_pause'
STATE_NAVIGATING_TO_DROPOFF = 'navigating_to_dropoff'
STATE_DROPOFF_PAUSE = 'dropoff_pause'


@dataclass
class PendingServerRequest:
    """Track the one UDP request that may be in flight at a time.

    The competition protocol does not include request ids, so the node keeps
    the interaction strictly serialized: send one request, wait for the
    matching response, then send the next request.
    """

    code: str
    attempts: int
    deadline: float


class FactoryAutonomyNode(Node):
    """ROS 2 node that bridges three concerns:

    1. Poll the competition UDP server for factory state.
    2. Ask the planner which transport should happen next.
    3. Dispatch pickup/drop-off goals to Nav2 and toggle the electromagnet.
    """

    def __init__(self):
        super().__init__('factory_autonomy')

        self._declare_params()
        self._load_params()

        # Static collaborators loaded once at startup.
        self._waypoints = self._load_waypoints()
        self._planner = FactoryPlanner()
        self._snapshot = FactorySnapshot()
        self._eletro_qos = _latched_bool_qos()

        # ROS interfaces for navigation, status, and the electromagnet output.
        self._nav_client = ActionClient(self, NavigateToPose, self._nav_action_name)
        self._status_pub = self.create_publisher(String, self._status_topic, 10)
        self._goal_pub = self.create_publisher(PoseStamped, self._selected_goal_topic, 10)
        self._eletro_pub = self.create_publisher(Bool, self._eletro_topic, self._eletro_qos)

        # Raw UDP transport used by the competition protocol.
        self._udp_sock = self._open_udp_socket()
        self._server_address = (self._server_host, self._server_port)

        # Global run state.
        self._enabled = self._autostart
        self._competition_started = False
        self._state = STATE_WAITING_FOR_START
        self._last_event = 'waiting for competition server'
        self._last_status_publish = 0.0
        self._last_refresh_time = 0.0
        self._last_time_left_request = 0.0
        self._next_start_request_time = 0.0
        self._competition_time_left: Optional[int] = None

        # UDP request bookkeeping.
        self._pending_request: Optional[PendingServerRequest] = None
        self._refresh_queue: List[str] = []

        # Navigation/task bookkeeping.
        self._current_task: Optional[TransportTask] = None
        self._current_leg: Optional[str] = None
        self._goal_handle = None
        self._goal_future = None
        self._pause_until: Optional[float] = None
        self._next_dispatch_time = 0.0
        self._nav_attempts = 0
        self._last_feedback_distance: Optional[float] = None

        # The node is driven by two lightweight loops:
        # - a fast UDP poller that only receives/parses packets
        # - a slower state-machine tick that decides what to do next
        self.create_timer(1.0 / self._udp_poll_hz, self._poll_udp)
        self.create_timer(1.0 / self._tick_hz, self._tick)

        self.get_logger().info(
            f'Factory autonomy ready; server={self._server_host}:{self._server_port}; '
            f'client={self._client_bind_host}:{self._client_bind_port}; '
            f'loaded {len(self._waypoints.names())} waypoints'
        )
        self._publish_eletro(False, reason='startup')
        self._publish_status(force=True)

    def destroy_node(self):
        try:
            self._udp_sock.close()
            self._publish_eletro(False, reason='shutdown')
        finally:
            super().destroy_node()

    def _declare_params(self) -> None:
        """Declare ROS parameters and their default values."""
        self.declare_parameter('server_host', '127.0.0.1')
        self.declare_parameter('server_port', 5005)
        self.declare_parameter('client_bind_host', '0.0.0.0')
        self.declare_parameter('client_bind_port', 0)
        self.declare_parameter('udp_poll_hz', 20.0)
        self.declare_parameter('tick_hz', 5.0)
        self.declare_parameter('server_request_timeout_seconds', 0.5)
        self.declare_parameter('server_request_retries', 3)
        self.declare_parameter('start_request_period_seconds', 1.0)
        self.declare_parameter('factory_refresh_period_seconds', 2.0)
        self.declare_parameter('time_left_period_seconds', 10.0)
        self.declare_parameter('waypoints_file', '/shared/factory_waypoints.yaml')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('nav_action_name', 'navigate_to_pose')
        self.declare_parameter('nav_server_wait_seconds', 1.0)
        self.declare_parameter('nav_retry_delay_seconds', 2.0)
        self.declare_parameter('max_nav_retries', 1)
        self.declare_parameter('pickup_pause_seconds', 2.0)
        self.declare_parameter('dropoff_pause_seconds', 1.0)
        self.declare_parameter('autostart', True)
        self.declare_parameter('status_topic', '/factory_autonomy/status')
        self.declare_parameter('selected_goal_topic', '/factory_autonomy/selected_goal')
        self.declare_parameter('eletro_topic', 'eletro')
        self.declare_parameter('status_period_seconds', 1.0)

    def _load_params(self) -> None:
        """Read declared ROS parameters into plain Python attributes."""
        get = self.get_parameter
        self._server_host = str(get('server_host').value)
        self._server_port = int(get('server_port').value)
        self._client_bind_host = str(get('client_bind_host').value)
        self._client_bind_port = int(get('client_bind_port').value)
        self._udp_poll_hz = float(get('udp_poll_hz').value)
        self._tick_hz = float(get('tick_hz').value)
        self._request_timeout = float(get('server_request_timeout_seconds').value)
        self._request_retries = int(get('server_request_retries').value)
        self._start_request_period = float(get('start_request_period_seconds').value)
        self._refresh_period = float(get('factory_refresh_period_seconds').value)
        self._time_left_period = float(get('time_left_period_seconds').value)
        self._waypoints_file = str(get('waypoints_file').value)
        self._map_frame = str(get('map_frame').value)
        self._nav_action_name = str(get('nav_action_name').value)
        self._nav_server_wait_seconds = float(get('nav_server_wait_seconds').value)
        self._nav_retry_delay_seconds = float(get('nav_retry_delay_seconds').value)
        self._max_nav_retries = int(get('max_nav_retries').value)
        self._pickup_pause_seconds = float(get('pickup_pause_seconds').value)
        self._dropoff_pause_seconds = float(get('dropoff_pause_seconds').value)
        self._autostart = bool(get('autostart').value)
        self._status_topic = str(get('status_topic').value)
        self._selected_goal_topic = str(get('selected_goal_topic').value)
        self._eletro_topic = str(get('eletro_topic').value)
        self._status_period_seconds = float(get('status_period_seconds').value)

    def _load_waypoints(self) -> WaypointStore:
        return load_waypoints_file(self._waypoints_file)

    def _open_udp_socket(self) -> socket.socket:
        """Create a non-blocking UDP socket for the competition server."""
        udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        udp_sock.bind((self._client_bind_host, self._client_bind_port))
        udp_sock.setblocking(False)
        return udp_sock

    def _poll_udp(self) -> None:
        """Drain all currently available UDP packets.

        This method never decides what should happen next. Its job is only to
        receive data, validate that it matches the outstanding request, and
        hand the parsed response to the state machine.
        """
        while True:
            try:
                payload, address = self._udp_sock.recvfrom(4096)
            except BlockingIOError:
                break
            except OSError as exc:
                self.get_logger().error(f'UDP receive failed: {exc}')
                break

            if self._pending_request is None:
                # Without an outstanding request we cannot safely correlate the
                # packet to any expected response, so ignore it.
                self.get_logger().debug(f'Ignoring unsolicited UDP packet from {address}: {payload!r}')
                continue

            request_code = self._pending_request.code
            try:
                response = parse_server_response(request_code, payload)
            except Exception as exc:
                self.get_logger().warning(
                    f'Invalid response for {request_code} from {address}: {exc}'
                )
                continue

            self._pending_request = None
            self._handle_server_response(request_code, response)

    def _tick(self) -> None:
        """Advance the high-level autonomy state machine by one step."""
        now = time.time()
        self._handle_request_timeout(now)

        if self._state == STATE_WAITING_FOR_START:
            # Before the run starts, repeatedly ask for incoming parts. A
            # non-STOP response doubles as the "start" signal in this protocol.
            if self._enabled and self._pending_request is None and now >= self._next_start_request_time:
                self._send_server_request(REQUEST_INCOMING)

        elif self._state == STATE_IDLE and self._enabled:
            # When idle, first refresh stale factory state, otherwise try to
            # dispatch the next movement.
            if self._pending_request is None and now - self._last_refresh_time >= self._refresh_period:
                self._start_factory_refresh()
            elif self._pending_request is None:
                self._dispatch_next_task()

        elif self._state == STATE_PICKUP_PAUSE and self._pause_until is not None:
            # The robot simulates the pickup dwell time before driving away.
            if now >= self._pause_until and self._current_task is not None:
                self._send_navigation_goal(self._current_task.dropoff_waypoint, leg='dropoff')

        elif self._state == STATE_DROPOFF_PAUSE and self._pause_until is not None:
            # After the drop-off dwell time, commit the move in the local model.
            if now >= self._pause_until:
                self._finish_current_task()

        elif self._state == STATE_WAITING_FOR_NAV2 and now >= self._next_dispatch_time:
            # Re-attempt goal dispatch after a short backoff while waiting for
            # Nav2 or retrying after a failed goal attempt.
            if self._current_task is not None and self._current_leg is not None:
                waypoint = (
                    self._current_task.pickup_waypoint
                    if self._current_leg == 'pickup'
                    else self._current_task.dropoff_waypoint
                )
                self._send_navigation_goal(waypoint, leg=self._current_leg)

        self._publish_status()

    def _send_server_request(self, code: str) -> None:
        """Send one competition-server request if no other request is pending."""
        if self._pending_request is not None:
            return

        try:
            self._udp_sock.sendto(code.encode('ascii'), self._server_address)
        except OSError as exc:
            self._last_event = f'failed to send {code}: {exc}'
            self.get_logger().warning(self._last_event)
            return

        self._pending_request = PendingServerRequest(
            code=code,
            attempts=1,
            deadline=time.time() + self._request_timeout,
        )
        self._last_event = f'sent server request {code}'

    def _handle_request_timeout(self, now: float) -> None:
        """Retry or abandon the current UDP request when its deadline expires."""
        if self._pending_request is None or now < self._pending_request.deadline:
            return

        request = self._pending_request
        if request.attempts < self._request_retries:
            try:
                self._udp_sock.sendto(request.code.encode('ascii'), self._server_address)
                request.attempts += 1
                request.deadline = now + self._request_timeout
                self._last_event = f'retrying server request {request.code}'
            except OSError as exc:
                self._last_event = f'failed to retry {request.code}: {exc}'
                self.get_logger().warning(self._last_event)
            return

        self._last_event = f'server request {request.code} timed out'
        self.get_logger().warning(self._last_event)
        self._pending_request = None

        if self._state == STATE_WAITING_FOR_START:
            self._next_start_request_time = now + self._start_request_period
        elif self._state == STATE_REFRESHING_FACTORY:
            self._state = STATE_IDLE
            self._last_refresh_time = now

    def _handle_server_response(self, request_code: str, response) -> None:
        """Merge one parsed server response into the node state."""
        now = time.time()

        if response == RESPONSE_STOP:
            # STOP means "competition has not started yet" when polled before
            # the run, so return to the start-wait loop.
            self._competition_started = False
            self._state = STATE_WAITING_FOR_START
            self._next_start_request_time = now + self._start_request_period
            self._last_event = 'server returned STOP; robot remains stopped'
            self._publish_status(force=True)
            return

        if request_code in PART_RESPONSE_REQUESTS:
            self._snapshot = self._snapshot.update(request_code, response)
            if request_code == REQUEST_INCOMING and not self._competition_started:
                # The first non-STOP incoming response is treated as the start
                # of the run. We already know incoming, so refresh the rest.
                self._competition_started = True
                self._last_event = f'run started; incoming parts={"".join(response)}'
                self._start_factory_refresh(include_incoming=False)
            elif self._state == STATE_REFRESHING_FACTORY:
                # Refreshes are serialized: once one response arrives, ask for
                # the next item in the queue.
                self._request_next_refresh_item()
            else:
                self._last_event = f'updated {request_code}={"".join(response)}'
            self._publish_status(force=True)
            return

        if request_code == REQUEST_TIME_LEFT:
            self._competition_time_left = int(response)
            self._last_event = f'time left: {self._competition_time_left}s'
            self._publish_status(force=True)

    def _start_factory_refresh(self, include_incoming: bool = True) -> None:
        """Queue a full snapshot refresh from the server."""
        self._state = STATE_REFRESHING_FACTORY
        self._refresh_queue = [
            REQUEST_OUTGOING,
            REQUEST_MACHINE_A,
            REQUEST_MACHINE_B,
        ]
        if include_incoming:
            self._refresh_queue.insert(0, REQUEST_INCOMING)
        self._request_next_refresh_item()

    def _request_next_refresh_item(self) -> None:
        """Send the next request needed to complete a queued refresh."""
        if self._pending_request is not None:
            return

        if not self._refresh_queue:
            self._last_refresh_time = time.time()
            self._state = STATE_IDLE
            self._last_event = 'factory state refreshed'
            self._maybe_request_time_left()
            return

        self._send_server_request(self._refresh_queue.pop(0))

    def _maybe_request_time_left(self) -> None:
        """Optionally request the remaining competition time at a slower cadence."""
        if self._time_left_period <= 0.0:
            return
        if self._pending_request is not None:
            return
        now = time.time()
        if now - self._last_time_left_request >= self._time_left_period:
            self._last_time_left_request = now
            self._send_server_request(REQUEST_TIME_LEFT)

    def _dispatch_next_task(self) -> None:
        """Ask the planner for the next move and start its pickup leg."""
        if self._current_task is not None:
            return

        task = self._planner.choose_next(self._snapshot)
        if task is None:
            self._last_event = 'no transport task available; waiting for machine/output changes'
            return

        self._current_task = task
        self._current_leg = 'pickup'
        self._nav_attempts = 0
        self._send_navigation_goal(task.pickup_waypoint, leg='pickup')

    def _send_navigation_goal(self, waypoint_name: str, leg: str) -> None:
        """Resolve one waypoint name and forward it to Nav2."""
        if self._current_task is None:
            return

        waypoint = self._waypoints.resolve(waypoint_name)
        if waypoint is None:
            self._fail_current_task(f'missing waypoint "{waypoint_name}"')
            return

        if self._goal_handle is not None or self._goal_future is not None:
            return

        self._current_leg = leg

        if not self._nav_client.wait_for_server(timeout_sec=self._nav_server_wait_seconds):
            self._state = STATE_WAITING_FOR_NAV2
            self._next_dispatch_time = time.time() + self._nav_retry_delay_seconds
            self._last_event = 'waiting for Nav2 NavigateToPose action server'
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self._pose_for_waypoint(waypoint)

        self._goal_pub.publish(goal_msg.pose)
        self._state = STATE_NAVIGATING_TO_PICKUP if leg == 'pickup' else STATE_NAVIGATING_TO_DROPOFF
        self._last_event = (
            f'navigating to {leg} waypoint {waypoint_name}; '
            f'task={self._current_task.to_dict()}'
        )
        self.get_logger().info(self._last_event)

        self._goal_future = self._nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self._nav_feedback_cb,
        )
        self._goal_future.add_done_callback(self._goal_response_cb)
        self._publish_status(force=True)

    def _pose_for_waypoint(self, waypoint: Waypoint) -> PoseStamped:
        """Convert a logical waypoint into a Nav2 goal pose."""
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self._map_frame
        pose.pose.position.x = waypoint.x
        pose.pose.position.y = waypoint.y
        pose.pose.position.z = 0.0
        # Nav2 expects a quaternion. The waypoint stores yaw in radians, so we
        # construct the planar quaternion directly from that heading.
        pose.pose.orientation.z = math.sin(waypoint.yaw / 2.0)
        pose.pose.orientation.w = math.cos(waypoint.yaw / 2.0)
        return pose

    def _goal_response_cb(self, future) -> None:
        """Handle the asynchronous accept/reject response from Nav2."""
        self._goal_future = None
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._handle_nav_failure(f'goal request failed: {exc}')
            return

        if not goal_handle.accepted:
            self._handle_nav_failure('goal rejected by Nav2')
            return

        self._goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._nav_result_cb)

    def _nav_feedback_cb(self, feedback_msg) -> None:
        """Keep the latest distance-to-goal for status publishing."""
        feedback = feedback_msg.feedback
        self._last_feedback_distance = getattr(feedback, 'distance_remaining', None)

    def _nav_result_cb(self, future) -> None:
        """Handle the final result of a Nav2 goal."""
        self._goal_handle = None
        try:
            result = future.result()
        except Exception as exc:
            self._handle_nav_failure(f'goal result failed: {exc}')
            return

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self._handle_arrival()
            return

        self._handle_nav_failure(f'Nav2 finished with status {_goal_status_name(result.status)}')

    def _handle_arrival(self) -> None:
        """Advance from a successful navigation result into the dwell phase."""
        if self._current_task is None or self._current_leg is None:
            self._state = STATE_IDLE
            return

        if self._current_leg == 'pickup':
            self._state = STATE_PICKUP_PAUSE
            self._pause_until = time.time() + self._pickup_pause_seconds
            self._publish_eletro(True, reason='pickup arrival')
            self._last_event = f'arrived at pickup {self._current_task.source.label()}'
            # TODO: publish the "loaded" footprint here when that interface exists.
        else:
            self._state = STATE_DROPOFF_PAUSE
            self._pause_until = time.time() + self._dropoff_pause_seconds
            self._publish_eletro(False, reason='dropoff arrival')
            self._last_event = f'arrived at dropoff {self._current_task.target.label()}'
            # TODO: publish the "unloaded" footprint here when that interface exists.

        self._nav_attempts = 0
        self.get_logger().info(self._last_event)
        self._publish_status(force=True)

    def _handle_nav_failure(self, detail: str) -> None:
        """Retry a navigation leg when allowed, otherwise fail the task."""
        if self._current_task is None:
            self._state = STATE_IDLE
            return

        if self._nav_attempts < self._max_nav_retries:
            self._nav_attempts += 1
            self._state = STATE_WAITING_FOR_NAV2
            self._next_dispatch_time = time.time() + self._nav_retry_delay_seconds
            self._last_event = f'{detail}; retry {self._nav_attempts}/{self._max_nav_retries}'
            self.get_logger().warning(self._last_event)
            self._publish_status(force=True)
            return

        self._fail_current_task(detail)

    def _finish_current_task(self) -> None:
        """Commit the completed move into the local snapshot model.

        The local model is updated optimistically here, then a full refresh is
        requested so the node converges back to the authoritative server state.
        """
        if self._current_task is None:
            self._state = STATE_IDLE
            return

        task = self._current_task
        self._snapshot = self._snapshot.with_slot(task.source.zone, task.source.slot, PART_EMPTY)
        self._snapshot = self._snapshot.with_slot(task.target.zone, task.target.slot, task.part_type)
        self._last_event = f'completed transport {task.to_dict()}'
        self.get_logger().info(self._last_event)
        self._clear_current_task()
        self._start_factory_refresh()
        self._publish_status(force=True)

    def _fail_current_task(self, detail: str) -> None:
        """Drop the current task and return to idle after an unrecoverable error."""
        task = self._current_task.to_dict() if self._current_task else None
        self._last_event = f'failed task {task}: {detail}'
        self.get_logger().error(self._last_event)
        self._clear_current_task()
        self._state = STATE_IDLE
        self._publish_status(force=True)

    def _clear_current_task(self) -> None:
        """Reset all task- and navigation-specific runtime state."""
        self._current_task = None
        self._current_leg = None
        self._goal_handle = None
        self._goal_future = None
        self._pause_until = None
        self._next_dispatch_time = 0.0
        self._nav_attempts = 0
        self._last_feedback_distance = None
        self._state = STATE_IDLE

    def _publish_status(self, force: bool = False) -> None:
        """Publish a compact JSON snapshot for debugging and dashboards."""
        now = time.time()
        if not force and now - self._last_status_publish < self._status_period_seconds:
            return

        self._last_status_publish = now
        msg = String()
        msg.data = json.dumps(
            {
                'enabled': self._enabled,
                'competition_started': self._competition_started,
                'competition_time_left': self._competition_time_left,
                'state': self._state,
                'snapshot': self._snapshot.to_dict(),
                'pending_server_request': (
                    self._pending_request.code if self._pending_request else None
                ),
                'current_task': self._current_task.to_dict() if self._current_task else None,
                'current_leg': self._current_leg,
                'last_event': self._last_event,
                'last_feedback_distance': self._last_feedback_distance,
            },
            sort_keys=True,
        )
        self._status_pub.publish(msg)

    def _publish_eletro(self, enabled: bool, reason: str) -> None:
        """Publish the electromagnet command and log why it changed."""
        msg = Bool()
        msg.data = enabled
        self._eletro_pub.publish(msg)
        self.get_logger().info(
            f'published {enabled} on {self._eletro_topic} ({reason})'
        )


def _goal_status_name(status: int) -> str:
    """Convert Nav2 status codes into readable log text."""
    names = {
        GoalStatus.STATUS_UNKNOWN: 'UNKNOWN',
        GoalStatus.STATUS_ACCEPTED: 'ACCEPTED',
        GoalStatus.STATUS_EXECUTING: 'EXECUTING',
        GoalStatus.STATUS_CANCELING: 'CANCELING',
        GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
        GoalStatus.STATUS_CANCELED: 'CANCELED',
        GoalStatus.STATUS_ABORTED: 'ABORTED',
    }
    return names.get(status, str(status))


def _latched_bool_qos() -> QoSProfile:
    """QoS profile used for the electromagnet command topic.

    Transient-local durability makes the last command available to late joiners,
    which is useful when the GPIO node starts after the autonomy node.
    """
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def main(args=None):
    """ROS 2 entry point."""
    rclpy.init(args=args)
    node = FactoryAutonomyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
