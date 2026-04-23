import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import yaml
import math
from pathlib import Path


class WaypointSaver(Node):
    def __init__(self, name: str, file_path: str):
        super().__init__('waypoint_saver')

        self.name = name
        self.file_path = Path(file_path)
        self.pose = None

        self.sub = self.create_subscription(
            PoseStamped,
            '/amcl_pose',
            self.callback,
            10
        )

    def callback(self, msg):
        self.pose = msg.pose

    def wait_for_pose(self):
        self.get_logger().info("Waiting for /amcl_pose...")
        while rclpy.ok() and self.pose is None:
            rclpy.spin_once(self)

    def quaternion_to_yaw(self, z, w):
        return 2 * math.atan2(z, w)

    def save(self):
        self.wait_for_pose()

        x_m = self.pose.position.x
        y_m = self.pose.position.y
        z = self.pose.orientation.z
        w = self.pose.orientation.w

        yaw = self.quaternion_to_yaw(z, w)

        # convert meters → millimeters
        x_mm = int(round(x_m * 1000))
        y_mm = int(round(y_m * 1000))

        # load existing YAML if it exists
        if self.file_path.exists():
            with self.file_path.open('r') as f:
                data = yaml.safe_load(f) or {}
        else:
            data = {}

        if 'waypoints' not in data:
            data['waypoints'] = {}

        # add/update waypoint
        data['waypoints'][self.name] = {
            'x': x_mm,
            'y': y_mm,
            'yaw': float(yaw)
        }

        # save back
        with self.file_path.open('w') as f:
            yaml.dump(data, f, sort_keys=True)

        self.get_logger().info(
            f"Saved waypoint '{self.name}' at ({x_mm}, {y_mm}, yaw={yaw:.3f})"
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
