#!/usr/bin/env python3

from __future__ import annotations

from datetime import datetime
import math
from pathlib import Path
import signal
import subprocess
from typing import List

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
import yaml


class SessionControl(Node):
    def __init__(self) -> None:
        super().__init__('session_control')

        self.declare_parameter('mode', 'record')
        self.declare_parameter('joy_topic', '/joy')

        self.declare_parameter('start_button_idx', 7)
        self.declare_parameter('stop_button_idx', 6)
        self.declare_parameter('waypoint_button_idx', 2)

        self.declare_parameter('recordings_dir', '/shared/recordings')
        self.declare_parameter('record_all', False)
        self.declare_parameter(
            'record_topics',
            ['/scan', '/imu/data', '/odometry/filtered', '/tf', '/tf_static', '/cmd_vel'],
        )

        self.declare_parameter('waypoint_file', '/shared/factory_waypoints.yaml')
        self.declare_parameter('waypoint_name_prefix', 'wp_')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('waypoint_debounce_seconds', 0.6)
        self.declare_parameter('tf_lookup_timeout_seconds', 0.25)

        self.mode = str(self.get_parameter('mode').value).strip().lower()
        self.joy_topic = str(self.get_parameter('joy_topic').value)

        self.start_button_idx = int(self.get_parameter('start_button_idx').value)
        self.stop_button_idx = int(self.get_parameter('stop_button_idx').value)
        self.waypoint_button_idx = int(self.get_parameter('waypoint_button_idx').value)

        self.recordings_dir = Path(str(self.get_parameter('recordings_dir').value))
        self.record_all = bool(self.get_parameter('record_all').value)
        self.record_topics = [str(topic) for topic in self.get_parameter('record_topics').value]

        self.waypoint_file = Path(str(self.get_parameter('waypoint_file').value))
        self.waypoint_name_prefix = str(self.get_parameter('waypoint_name_prefix').value)
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.waypoint_debounce_seconds = float(self.get_parameter('waypoint_debounce_seconds').value)
        self.tf_lookup_timeout_seconds = float(self.get_parameter('tf_lookup_timeout_seconds').value)

        self._previous_buttons: List[int] = []
        self._bag_proc: subprocess.Popen | None = None
        self._bag_output_dir: Path | None = None
        self._last_waypoint_time = 0.0
        self._started_time = self.get_clock().now().nanoseconds / 1e9
        self._last_joy_time: float | None = None
        self._joy_warning_active = False

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._status_pub = self.create_publisher(String, '/session_control/status', 10)
        self._joy_sub = self.create_subscription(Joy, self.joy_topic, self._on_joy, 20)
        self._joy_watchdog_timer = self.create_timer(2.0, self._check_joy_stream)

        self.get_logger().info(
            f'session_control started in mode={self.mode}, joy_topic={self.joy_topic}, '
            f'start_button={self.start_button_idx}, stop_button={self.stop_button_idx}, '
            f'waypoint_button={self.waypoint_button_idx}'
        )

    def destroy_node(self) -> bool:
        self._stop_recording('node shutdown')
        return super().destroy_node()

    def _publish_status(self, text: str) -> None:
        message = String()
        message.data = text
        self._status_pub.publish(message)

    def _on_joy(self, message: Joy) -> None:
        self._last_joy_time = self.get_clock().now().nanoseconds / 1e9
        if self._joy_warning_active:
            self.get_logger().info(f'joy stream detected on {self.joy_topic}')
            self._publish_status('joy_stream_detected')
            self._joy_warning_active = False

        buttons = [int(value) for value in message.buttons]
        if not self._previous_buttons:
            self._previous_buttons = buttons
            return

        start_edge = self._rising_edge(buttons, self.start_button_idx)
        stop_edge = self._rising_edge(buttons, self.stop_button_idx)
        waypoint_edge = self._rising_edge(buttons, self.waypoint_button_idx)

        if self.mode == 'record':
            if start_edge:
                self._start_recording()
            if stop_edge:
                self._stop_recording('button stop')

        if self.mode == 'waypoint' and waypoint_edge:
            now = self.get_clock().now().nanoseconds / 1e9
            if now - self._last_waypoint_time >= self.waypoint_debounce_seconds:
                self._save_waypoint_from_tf()
                self._last_waypoint_time = now

        self._previous_buttons = buttons

    def _rising_edge(self, buttons: List[int], index: int) -> bool:
        if index < 0 or index >= len(buttons) or index >= len(self._previous_buttons):
            return False
        return self._previous_buttons[index] == 0 and buttons[index] == 1

    def _check_joy_stream(self) -> None:
        now = self.get_clock().now().nanoseconds / 1e9
        if self._last_joy_time is None:
            elapsed = now - self._started_time
            if elapsed >= 6.0 and not self._joy_warning_active:
                self.get_logger().warning(
                    f'no Joy messages received on {self.joy_topic}; '
                    'record/waypoint buttons will not work until joystick input is available'
                )
                self._publish_status('waiting_for_joy_stream')
                self._joy_warning_active = True
            return

        elapsed_since_joy = now - self._last_joy_time
        if elapsed_since_joy >= 6.0 and not self._joy_warning_active:
            self.get_logger().warning(
                f'Joy stream on {self.joy_topic} timed out ({elapsed_since_joy:.1f}s since last message)'
            )
            self._publish_status('joy_stream_timeout')
            self._joy_warning_active = True

    def _start_recording(self) -> None:
        if self._bag_proc is not None and self._bag_proc.poll() is None:
            self.get_logger().warning('recording already running')
            self._publish_status('recording_already_running')
            return

        self.recordings_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._bag_output_dir = self.recordings_dir / f'session_{stamp}'

        if self.record_all:
            cmd = ['ros2', 'bag', 'record', '-a', '-o', str(self._bag_output_dir)]
        else:
            cmd = ['ros2', 'bag', 'record', '-o', str(self._bag_output_dir)] + self.record_topics
        self.get_logger().info(f'starting rosbag record: {" ".join(cmd)}')
        self._bag_proc = subprocess.Popen(cmd)
        self._publish_status(f'recording_started:{self._bag_output_dir}')

    def _stop_recording(self, reason: str) -> None:
        if self._bag_proc is None:
            return

        if self._bag_proc.poll() is None:
            self.get_logger().info(f'stopping recording ({reason})')
            self._bag_proc.send_signal(signal.SIGINT)
            try:
                self._bag_proc.wait(timeout=12.0)
            except subprocess.TimeoutExpired:
                self.get_logger().warning('rosbag did not stop in time; terminating')
                self._bag_proc.terminate()
                self._bag_proc.wait(timeout=4.0)

        output_dir = self._bag_output_dir
        self._bag_proc = None
        self._bag_output_dir = None

        if output_dir is not None:
            latest = self.recordings_dir / 'latest'
            if latest.exists() or latest.is_symlink():
                latest.unlink()
            latest.symlink_to(output_dir.name)
            self._publish_status(f'recording_stopped:{output_dir}')

    def _save_waypoint_from_tf(self) -> None:
        try:
            transform = self._tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=self.tf_lookup_timeout_seconds),
            )
        except TransformException as exc:
            self.get_logger().warning(f'could not lookup {self.map_frame}->{self.base_frame}: {exc}')
            self._publish_status('waypoint_failed_tf_lookup')
            return

        translation = transform.transform.translation
        rotation = transform.transform.rotation

        yaw = math.atan2(
            2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
        )

        self.waypoint_file.parent.mkdir(parents=True, exist_ok=True)
        if self.waypoint_file.exists():
            with self.waypoint_file.open('r', encoding='utf-8') as stream:
                data = yaml.safe_load(stream) or {}
        else:
            data = {}

        waypoints = data.get('waypoints')
        if not isinstance(waypoints, dict):
            waypoints = {}
            data['waypoints'] = waypoints

        next_index = self._next_waypoint_index(waypoints)
        name = f'{self.waypoint_name_prefix}{next_index:03d}'
        waypoints[name] = {
            'x': float(translation.x),
            'y': float(translation.y),
            'yaw': float(yaw),
            'frame_id': self.map_frame,
        }

        with self.waypoint_file.open('w', encoding='utf-8') as stream:
            yaml.safe_dump(data, stream, sort_keys=True)

        self.get_logger().info(
            f'saved waypoint {name} at x={translation.x:.3f}, y={translation.y:.3f}, yaw={yaw:.3f}'
        )
        self._publish_status(f'waypoint_saved:{name}')

    def _next_waypoint_index(self, waypoints: dict) -> int:
        max_index = 0
        for key in waypoints:
            if not isinstance(key, str):
                continue
            if key.startswith(self.waypoint_name_prefix):
                suffix = key[len(self.waypoint_name_prefix):]
                if suffix.isdigit():
                    max_index = max(max_index, int(suffix))
        return max_index + 1


def main() -> None:
    rclpy.init()
    node = SessionControl()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
