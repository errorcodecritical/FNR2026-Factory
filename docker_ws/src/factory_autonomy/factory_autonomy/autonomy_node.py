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
from typing import Iterable
from typing import List, Optional

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ComputePathToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener

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
    code: str
    attempts: int
    deadline: float


class FactoryAutonomyNode(Node):
    def __init__(self):
        super().__init__('factory_autonomy')

        self._declare_params()
        self._load_params()

        self._waypoints = self._load_waypoints()
        self._planner = FactoryPlanner()
        self._snapshot = FactorySnapshot()
        self._eletro_qos = _latched_bool_qos()
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._nav_client = ActionClient(self, NavigateToPose, self._nav_action_name)
        self._compute_path_client = self.create_client(ComputePathToPose, self._compute_path_service)
        self._status_pub = self.create_publisher(String, self._status_topic, 10)
        self._goal_pub = self.create_publisher(PoseStamped, self._selected_goal_topic, 10)
        self._eletro_pub = self.create_publisher(Bool, self._eletro_topic, self._eletro_qos)
        self._near_goal_pub = self.create_publisher(Bool, self._near_goal_topic, self._eletro_qos)

        self._udp_sock = self._open_udp_socket()
        self._server_address = (self._server_host, self._server_port)

        self._enabled = self._autostart
        self._competition_started = False
        self._state = STATE_WAITING_FOR_START
        self._last_event = 'waiting for competition server'
        self._last_status_publish = 0.0
        self._last_refresh_time = 0.0
        self._last_time_left_request = 0.0
        self._next_start_request_time = 0.0
        self._competition_time_left: Optional[int] = None

        self._pending_request: Optional[PendingServerRequest] = None
        self._refresh_queue: List[str] = []

        self._current_task: Optional[TransportTask] = None
        self._current_leg: Optional[str] = None
        self._goal_handle = None
        self._goal_future = None
        self._pause_until: Optional[float] = None
        self._next_dispatch_time = 0.0
        self._nav_attempts = 0
        self._last_feedback_distance: Optional[float] = None
        self._near_goal_active = False

        self.create_timer(1.0 / self._udp_poll_hz, self._poll_udp)
        self.create_timer(1.0 / self._tick_hz, self._tick)

        self.get_logger().info(
            f'Factory autonomy ready; server={self._server_host}:{self._server_port}; '
            f'client={self._client_bind_host}:{self._client_bind_port}; '
            f'loaded {len(self._waypoints.names())} waypoints'
        )
        self._publish_eletro(False, reason='startup')
        self._publish_near_goal(False, force=True)
        self._publish_status(force=True)

    def destroy_node(self):
        try:
            self._udp_sock.close()
            self._publish_eletro(False, reason='shutdown')
        finally:
            super().destroy_node()

    def _declare_params(self) -> None:
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
        self.declare_parameter('compute_path_service', '/compute_path_to_pose')
        self.declare_parameter('compute_path_timeout_seconds', 0.35)
        self.declare_parameter('path_cost_weight_pickup', 1.0)
        self.declare_parameter('path_cost_weight_dropoff', 1.0)
        self.declare_parameter('path_cost_max_candidates', 8)
        self.declare_parameter('nav_server_wait_seconds', 1.0)
        self.declare_parameter('nav_retry_delay_seconds', 2.0)
        self.declare_parameter('max_nav_retries', 1)
        self.declare_parameter('near_goal_topic', '/factory_autonomy/near_goal')
        self.declare_parameter('near_goal_distance_m', 0.55)
        self.declare_parameter('near_goal_hysteresis_m', 0.10)
        self.declare_parameter('pickup_pause_seconds', 2.0)
        self.declare_parameter('dropoff_pause_seconds', 1.0)
        self.declare_parameter('autostart', True)
        self.declare_parameter('status_topic', '/factory_autonomy/status')
        self.declare_parameter('selected_goal_topic', '/factory_autonomy/selected_goal')
        self.declare_parameter('eletro_topic', 'eletro')
        self.declare_parameter('status_period_seconds', 1.0)

    def _load_params(self) -> None:
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
        self._compute_path_service = str(get('compute_path_service').value)
        self._compute_path_timeout_seconds = float(get('compute_path_timeout_seconds').value)
        self._path_cost_weight_pickup = float(get('path_cost_weight_pickup').value)
        self._path_cost_weight_dropoff = float(get('path_cost_weight_dropoff').value)
        self._path_cost_max_candidates = int(get('path_cost_max_candidates').value)
        self._nav_server_wait_seconds = float(get('nav_server_wait_seconds').value)
        self._nav_retry_delay_seconds = float(get('nav_retry_delay_seconds').value)
        self._max_nav_retries = int(get('max_nav_retries').value)
        self._near_goal_topic = str(get('near_goal_topic').value)
        self._near_goal_distance_m = float(get('near_goal_distance_m').value)
        self._near_goal_hysteresis_m = float(get('near_goal_hysteresis_m').value)
        self._pickup_pause_seconds = float(get('pickup_pause_seconds').value)
        self._dropoff_pause_seconds = float(get('dropoff_pause_seconds').value)
        self._autostart = bool(get('autostart').value)
        self._status_topic = str(get('status_topic').value)
        self._selected_goal_topic = str(get('selected_goal_topic').value)
        self._eletro_topic = str(get('eletro_topic').value)
        self._status_period_seconds = float(get('status_period_seconds').value)

    def _load_waypoints(self) -> WaypointStore:
        return load_waypoints_file(self._waypoints_file, default_frame=self._map_frame)

    def _open_udp_socket(self) -> socket.socket:
        udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        udp_sock.bind((self._client_bind_host, self._client_bind_port))
        udp_sock.setblocking(False)
        return udp_sock

    def _poll_udp(self) -> None:
        while True:
            try:
                payload, address = self._udp_sock.recvfrom(4096)
            except BlockingIOError:
                break
            except OSError as exc:
                self.get_logger().error(f'UDP receive failed: {exc}')
                break

            if self._pending_request is None:
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
        now = time.time()
        self._handle_request_timeout(now)

        if self._state == STATE_WAITING_FOR_START:
            if self._enabled and self._pending_request is None and now >= self._next_start_request_time:
                self._send_server_request(REQUEST_INCOMING)

        elif self._state == STATE_IDLE and self._enabled:
            if self._pending_request is None and now - self._last_refresh_time >= self._refresh_period:
                self._start_factory_refresh()
            elif self._pending_request is None:
                self._dispatch_next_task()

        elif self._state == STATE_PICKUP_PAUSE and self._pause_until is not None:
            if now >= self._pause_until and self._current_task is not None:
                self._send_navigation_goal(self._current_task.dropoff_waypoint, leg='dropoff')

        elif self._state == STATE_DROPOFF_PAUSE and self._pause_until is not None:
            if now >= self._pause_until:
                self._finish_current_task()

        elif self._state == STATE_WAITING_FOR_NAV2 and now >= self._next_dispatch_time:
            if self._current_task is not None and self._current_leg is not None:
                waypoint = (
                    self._current_task.pickup_waypoint
                    if self._current_leg == 'pickup'
                    else self._current_task.dropoff_waypoint
                )
                self._send_navigation_goal(waypoint, leg=self._current_leg)

        self._publish_status()

    def _send_server_request(self, code: str) -> None:
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
        now = time.time()

        if response == RESPONSE_STOP:
            self._competition_started = False
            self._state = STATE_WAITING_FOR_START
            self._next_start_request_time = now + self._start_request_period
            self._last_event = 'server returned STOP; robot remains stopped'
            self._publish_status(force=True)
            return

        if request_code in PART_RESPONSE_REQUESTS:
            self._snapshot = self._snapshot.update(request_code, response)
            if request_code == REQUEST_INCOMING and not self._competition_started:
                self._competition_started = True
                self._last_event = f'run started; incoming parts={"".join(response)}'
                self._start_factory_refresh(include_incoming=False)
            elif self._state == STATE_REFRESHING_FACTORY:
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
        if self._time_left_period <= 0.0:
            return
        if self._pending_request is not None:
            return
        now = time.time()
        if now - self._last_time_left_request >= self._time_left_period:
            self._last_time_left_request = now
            self._send_server_request(REQUEST_TIME_LEFT)

    def _dispatch_next_task(self) -> None:
        if self._current_task is not None:
            return

        task = self._select_next_task()
        if task is None:
            self._last_event = 'no transport task available; waiting for machine/output changes'
            return

        self._current_task = task
        self._current_leg = 'pickup'
        self._nav_attempts = 0
        self._send_navigation_goal(task.pickup_waypoint, leg='pickup')

    def _select_next_task(self) -> Optional[TransportTask]:
        candidates = self._planner.candidate_tasks(self._snapshot)
        if not candidates:
            return None

        if len(candidates) == 1:
            return candidates[0]

        limited = candidates[: max(1, self._path_cost_max_candidates)]

        start_pose = self._current_robot_pose()
        if start_pose is None:
            self.get_logger().warning('could not get current pose for path-cost tie-break; falling back to scheduler order')
            return limited[0]

        best_task: Optional[TransportTask] = None
        best_score = float('inf')
        used_path_cost = False

        for task in limited:
            pickup_waypoint = self._waypoints.resolve(task.pickup_waypoint)
            dropoff_waypoint = self._waypoints.resolve(task.dropoff_waypoint)
            if pickup_waypoint is None or dropoff_waypoint is None:
                continue

            pickup_pose = self._pose_for_waypoint(pickup_waypoint)
            dropoff_pose = self._pose_for_waypoint(dropoff_waypoint)

            path_cost_to_pickup = self._path_cost(start_pose, pickup_pose)
            path_cost_pickup_to_dropoff = self._path_cost(pickup_pose, dropoff_pose)

            if path_cost_to_pickup is None:
                path_cost_to_pickup = self._euclidean_distance(start_pose, pickup_pose)
            else:
                used_path_cost = True

            if path_cost_pickup_to_dropoff is None:
                path_cost_pickup_to_dropoff = self._euclidean_distance(pickup_pose, dropoff_pose)
            else:
                used_path_cost = True

            score = (
                self._path_cost_weight_pickup * path_cost_to_pickup
                + self._path_cost_weight_dropoff * path_cost_pickup_to_dropoff
            )
            if score < best_score:
                best_score = score
                best_task = task

        if best_task is None:
            return limited[0]

        if used_path_cost:
            self.get_logger().info(
                f'path-cost selected task {best_task.to_dict()} (score={best_score:.3f})'
            )

        return best_task

    def _send_navigation_goal(self, waypoint_name: str, leg: str) -> None:
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
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = waypoint.frame_id or self._map_frame
        pose.pose.position.x = waypoint.x
        pose.pose.position.y = waypoint.y
        pose.pose.position.z = 0.0
        pose.pose.orientation.z = math.sin(waypoint.yaw / 2.0)
        pose.pose.orientation.w = math.cos(waypoint.yaw / 2.0)
        return pose

    def _goal_response_cb(self, future) -> None:
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
        feedback = feedback_msg.feedback
        self._last_feedback_distance = getattr(feedback, 'distance_remaining', None)
        self._update_near_goal_mode(self._last_feedback_distance)

    def _nav_result_cb(self, future) -> None:
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
        if self._current_task is None or self._current_leg is None:
            self._state = STATE_IDLE
            return

        if self._current_leg == 'pickup':
            self._state = STATE_PICKUP_PAUSE
            self._pause_until = time.time() + self._pickup_pause_seconds
            self._publish_eletro(True, reason='pickup arrival')
            self._last_event = f'arrived at pickup {self._current_task.source.label()}'
        else:
            self._state = STATE_DROPOFF_PAUSE
            self._pause_until = time.time() + self._dropoff_pause_seconds
            self._publish_eletro(False, reason='dropoff arrival')
            self._last_event = f'arrived at dropoff {self._current_task.target.label()}'

        self._nav_attempts = 0
        self._publish_near_goal(False)
        self.get_logger().info(self._last_event)
        self._publish_status(force=True)

    def _handle_nav_failure(self, detail: str) -> None:
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
        task = self._current_task.to_dict() if self._current_task else None
        self._last_event = f'failed task {task}: {detail}'
        self.get_logger().error(self._last_event)
        self._clear_current_task()
        self._state = STATE_IDLE
        self._publish_status(force=True)

    def _clear_current_task(self) -> None:
        self._current_task = None
        self._current_leg = None
        self._goal_handle = None
        self._goal_future = None
        self._pause_until = None
        self._next_dispatch_time = 0.0
        self._nav_attempts = 0
        self._last_feedback_distance = None
        self._state = STATE_IDLE
        self._publish_near_goal(False)

    def _current_robot_pose(self) -> Optional[PoseStamped]:
        try:
            transform = self._tf_buffer.lookup_transform(
                self._map_frame,
                'base_link',
                rclpy.time.Time(),
                timeout=Duration(seconds=0.15),
            )
        except TransformException:
            return None

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self._map_frame
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = 0.0
        pose.pose.orientation = transform.transform.rotation
        return pose

    def _path_cost(self, start: PoseStamped, goal: PoseStamped) -> Optional[float]:
        if not self._compute_path_client.wait_for_service(timeout_sec=self._compute_path_timeout_seconds):
            return None

        request = ComputePathToPose.Request()
        request.start = start
        request.goal = goal
        request.planner_id = ''
        request.use_start = True

        future = self._compute_path_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self._compute_path_timeout_seconds)

        if not future.done() or future.result() is None:
            return None

        response = future.result()
        path = response.path
        if not path.poses:
            return None
        return self._path_length(path.poses)

    def _path_length(self, poses: Iterable[PoseStamped]) -> float:
        total = 0.0
        previous = None
        for pose in poses:
            if previous is not None:
                dx = pose.pose.position.x - previous.pose.position.x
                dy = pose.pose.position.y - previous.pose.position.y
                total += math.hypot(dx, dy)
            previous = pose
        return total

    def _euclidean_distance(self, pose_a: PoseStamped, pose_b: PoseStamped) -> float:
        dx = pose_a.pose.position.x - pose_b.pose.position.x
        dy = pose_a.pose.position.y - pose_b.pose.position.y
        return math.hypot(dx, dy)

    def _update_near_goal_mode(self, distance_remaining: Optional[float]) -> None:
        if distance_remaining is None:
            return

        enter_threshold = self._near_goal_distance_m
        exit_threshold = self._near_goal_distance_m + self._near_goal_hysteresis_m

        desired_state = self._near_goal_active
        if self._near_goal_active:
            if distance_remaining > exit_threshold:
                desired_state = False
        else:
            if distance_remaining <= enter_threshold:
                desired_state = True

        self._publish_near_goal(desired_state)

    def _publish_near_goal(self, active: bool, force: bool = False) -> None:
        if not force and active == self._near_goal_active:
            return

        self._near_goal_active = active
        msg = Bool()
        msg.data = active
        self._near_goal_pub.publish(msg)

    def _publish_status(self, force: bool = False) -> None:
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
        msg = Bool()
        msg.data = enabled
        self._eletro_pub.publish(msg)
        self.get_logger().info(
            f'published {enabled} on {self._eletro_topic} ({reason})'
        )


def _goal_status_name(status: int) -> str:
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
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def main(args=None):
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
