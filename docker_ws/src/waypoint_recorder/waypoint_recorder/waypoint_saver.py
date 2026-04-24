import math
from pathlib import Path
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener
import yaml


class WaypointSaver(Node):
    def __init__(self, name: str, file_path: str):
        super().__init__('waypoint_saver')

        self.name = name
        self.file_path = Path(file_path)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

    def _read_robot_pose(self):
        timeout = Duration(seconds=2.0)
        transform = self._tf_buffer.lookup_transform(
            'map',
            'base_link',
            rclpy.time.Time(),
            timeout=timeout,
        )
        x = transform.transform.translation.x
        y = transform.transform.translation.y
        q = transform.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return x, y, yaw

    def save(self):
        self.get_logger().info("Waiting for map->base_link transform...")
        while rclpy.ok():
            try:
                x_m, y_m, yaw = self._read_robot_pose()
                break
            except TransformException:
                rclpy.spin_once(self, timeout_sec=0.1)

        # load existing YAML if it exists
        if self.file_path.exists():
            with self.file_path.open('r', encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}
        else:
            data = {}

        if 'waypoints' not in data:
            data['waypoints'] = {}

        # add/update waypoint
        data['waypoints'][self.name] = {
            'x': float(x_m),
            'y': float(y_m),
            'yaw': float(yaw),
            'frame_id': 'map',
        }

        # save back
        with self.file_path.open('w', encoding='utf-8') as f:
            yaml.dump(data, f, sort_keys=True)

        self.get_logger().info(
            f"Saved waypoint '{self.name}' at ({x_m:.3f}, {y_m:.3f}, yaw={yaw:.3f})"
        )


def main():
    import sys

    if len(sys.argv) < 3:
        print("Usage: ros2 run <pkg> waypoint_saver <name> <file.yaml>")
        return

    name = sys.argv[1]
    file_path = sys.argv[2]

    rclpy.init()
    node = WaypointSaver(name, file_path)

    node.save()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
