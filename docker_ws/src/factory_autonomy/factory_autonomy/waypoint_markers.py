#!/usr/bin/env python3

from __future__ import annotations

from pathlib import Path
from typing import Dict

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

from .waypoints import load_waypoints_file


class WaypointMarkersNode(Node):
    def __init__(self) -> None:
        super().__init__('waypoint_markers')

        self.declare_parameter('waypoints_file', '/shared/factory_waypoints.yaml')
        self.declare_parameter('markers_topic', '/factory_autonomy/waypoint_markers')
        self.declare_parameter('publish_period_seconds', 1.0)
        self.declare_parameter('marker_scale', 0.14)
        self.declare_parameter('label_scale', 0.10)
        self.declare_parameter('marker_alpha', 1.0)
        self.declare_parameter('show_labels', True)

        self._waypoints_file = str(self.get_parameter('waypoints_file').value)
        self._markers_topic = str(self.get_parameter('markers_topic').value)
        self._publish_period = float(self.get_parameter('publish_period_seconds').value)
        self._marker_scale = float(self.get_parameter('marker_scale').value)
        self._label_scale = float(self.get_parameter('label_scale').value)
        self._marker_alpha = float(self.get_parameter('marker_alpha').value)
        self._show_labels = bool(self.get_parameter('show_labels').value)

        self._publisher = self.create_publisher(MarkerArray, self._markers_topic, 10)
        self._last_ids: Dict[int, bool] = {}
        self._last_mtime_ns: int | None = None
        self._cached_msg = MarkerArray()

        self.create_timer(self._publish_period, self._publish)
        self.get_logger().info(
            f'publishing waypoint markers from {self._waypoints_file} to {self._markers_topic}'
        )

    def _publish(self) -> None:
        path = Path(self._waypoints_file)
        if not path.exists():
            self._publish_delete_all()
            self.get_logger().warning(f'waypoint file not found: {self._waypoints_file}')
            return

        stat = path.stat()
        if self._last_mtime_ns != stat.st_mtime_ns:
            self._cached_msg = self._build_markers()
            self._last_mtime_ns = stat.st_mtime_ns

        self._publisher.publish(self._cached_msg)

    def _publish_delete_all(self) -> None:
        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        self._publisher.publish(MarkerArray(markers=[delete_all]))
        self._cached_msg = MarkerArray()
        self._last_mtime_ns = None

    def _build_markers(self) -> MarkerArray:
        marker_array = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        store = load_waypoints_file(self._waypoints_file)
        names = store.names()

        active_ids: Dict[int, bool] = {}
        marker_id = 0

        for name in names:
            waypoint = store.resolve(name)
            if waypoint is None:
                continue

            point_marker = Marker()
            point_marker.header.frame_id = waypoint.frame_id
            point_marker.header.stamp = stamp
            point_marker.ns = 'waypoints'
            point_marker.id = marker_id
            point_marker.type = Marker.SPHERE
            point_marker.action = Marker.ADD
            point_marker.pose.position.x = waypoint.x
            point_marker.pose.position.y = waypoint.y
            point_marker.pose.position.z = 0.05
            point_marker.pose.orientation.w = 1.0
            point_marker.scale.x = self._marker_scale
            point_marker.scale.y = self._marker_scale
            point_marker.scale.z = self._marker_scale
            point_marker.color.r = 0.1
            point_marker.color.g = 0.8
            point_marker.color.b = 0.95
            point_marker.color.a = self._marker_alpha
            point_marker.lifetime = Duration(seconds=0.0).to_msg()
            marker_array.markers.append(point_marker)
            active_ids[marker_id] = True
            marker_id += 1

            if self._show_labels:
                text_marker = Marker()
                text_marker.header.frame_id = waypoint.frame_id
                text_marker.header.stamp = stamp
                text_marker.ns = 'waypoint_labels'
                text_marker.id = marker_id
                text_marker.type = Marker.TEXT_VIEW_FACING
                text_marker.action = Marker.ADD
                text_marker.pose.position.x = waypoint.x
                text_marker.pose.position.y = waypoint.y
                text_marker.pose.position.z = 0.22
                text_marker.pose.orientation.w = 1.0
                text_marker.scale.z = self._label_scale
                text_marker.color.r = 1.0
                text_marker.color.g = 1.0
                text_marker.color.b = 1.0
                text_marker.color.a = self._marker_alpha
                text_marker.text = name
                text_marker.lifetime = Duration(seconds=0.0).to_msg()
                marker_array.markers.append(text_marker)
                active_ids[marker_id] = True
                marker_id += 1

        for old_id in self._last_ids:
            if old_id in active_ids:
                continue
            delete_marker = Marker()
            delete_marker.header.stamp = stamp
            delete_marker.ns = 'waypoints' if old_id % 2 == 0 else 'waypoint_labels'
            delete_marker.id = old_id
            delete_marker.action = Marker.DELETE
            marker_array.markers.append(delete_marker)

        self._last_ids = active_ids
        return marker_array


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WaypointMarkersNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
