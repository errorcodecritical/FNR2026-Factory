#!/usr/bin/env python3
"""
footprint_publisher_node.py
──────────────────────────
Subscribes to the bool topic /enabled and publishes the robot footprint
to the costmap topic /local_costmap/footprint (and /global_costmap/footprint).

  /enabled == False  →  25 cm × 25 cm box  (compact mode)
  /enabled == True   →  25 cm × 35 cm box  (extended mode, width × length)

The footprint is published as geometry_msgs/msg/Polygon and also as a
parameter update via the ROS2 parameter service so Nav2 costmap layers
pick up the change at runtime.
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Bool
from geometry_msgs.msg import Polygon, Point32

# ── Footprint helpers ────────────────────────────────────────────────────────

def box_footprint(width_m: float, length_m: float) -> Polygon:
    """
    Return a centred rectangular Polygon footprint.

    width_m  – dimension along the Y axis  (side-to-side)
    length_m – dimension along the X axis  (front-to-back)
    """
    hw = width_m  / 2.0   # half-width
    hl = length_m / 2.0   # half-length

    poly = Polygon()
    poly.points = [
        Point32(x= hl, y= hw, z=0.0),   # front-left
        Point32(x= hl, y=-hw, z=0.0),   # front-right
        Point32(x=-hl, y=-hw, z=0.0),   # rear-right
        Point32(x=-hl, y= hw, z=0.0),   # rear-left
    ]
    return poly


def polygon_to_string(poly: Polygon) -> str:
    """Serialise a Polygon to the footprint string format Nav2 expects."""
    pts = ", ".join(f"[{p.x:.4f}, {p.y:.4f}]" for p in poly.points)
    return f"[{pts}]"


# ── Node ─────────────────────────────────────────────────────────────────────

class FootprintPublisherNode(Node):

    # Footprint dimensions (metres)
    COMPACT  = (0.25, 0.25)   # width × length when /enabled is False
    EXTENDED = (0.25, 0.35)   # width × length when /enabled is True

    # Topics / parameter paths
    ENABLED_TOPIC        = "/eletro"
    LOCAL_FP_TOPIC       = "/local_costmap/footprint"
    GLOBAL_FP_TOPIC      = "/global_costmap/footprint"
    LOCAL_FP_PARAM_NODE  = "/local_costmap/local_costmap"
    GLOBAL_FP_PARAM_NODE = "/global_costmap/global_costmap"
    FOOTPRINT_PARAM      = "footprint"

    def __init__(self):
        super().__init__("footprint_publisher")

        # ── Publishers ────────────────────────────────────────────────────
        qos = rclpy.qos.QoSProfile(
            depth=1,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
        )
        self._local_pub  = self.create_publisher(Polygon, self.LOCAL_FP_TOPIC,  qos)
        self._global_pub = self.create_publisher(Polygon, self.GLOBAL_FP_TOPIC, qos)

        # ── Parameter clients (for live Nav2 costmap updates) ─────────────
        from rcl_interfaces.srv import SetParameters
        self._local_param_client  = self.create_client(
            SetParameters, f"{self.LOCAL_FP_PARAM_NODE}/set_parameters"
        )
        self._global_param_client = self.create_client(
            SetParameters, f"{self.GLOBAL_FP_PARAM_NODE}/set_parameters"
        )

        # ── State ─────────────────────────────────────────────────────────
        self._current_enabled: bool | None = None  # track last state

        # ── Subscriber ────────────────────────────────────────────────────
        self._sub = self.create_subscription(
            Bool,
            self.ENABLED_TOPIC,
            self._enabled_callback,
            10,
        )

        self.get_logger().info("FootprintPublisherNode started – waiting for /enabled …")

        # Publish the compact footprint at startup (safe default)
        self._apply_footprint(enabled=False)

    # ── Callback ─────────────────────────────────────────────────────────────

    def _enabled_callback(self, msg: Bool) -> None:
        if msg.data == self._current_enabled:
            return  # no change – skip

        self._current_enabled = msg.data
        self.get_logger().info(
            f"/enabled changed → {msg.data}  "
            f"({'25×35 cm extended' if msg.data else '25×25 cm compact'} footprint)"
        )
        self._apply_footprint(enabled=msg.data)

    # ── Footprint application ─────────────────────────────────────────────────

    def _apply_footprint(self, enabled: bool) -> None:
        width, length = self.EXTENDED if enabled else self.COMPACT
        poly           = box_footprint(width, length)
        fp_string      = polygon_to_string(poly)

        # 1) Publish to footprint topics
        self._local_pub.publish(poly)
        self._global_pub.publish(poly)

        # 2) Update Nav2 costmap parameters (best-effort – nodes may not exist yet)
        self._set_footprint_param(self._local_param_client,  fp_string, "local")
        self._set_footprint_param(self._global_param_client, fp_string, "global")

        self.get_logger().debug(f"Footprint applied: {fp_string}")

    def _set_footprint_param(self, client, fp_string: str, label: str) -> None:
        from rcl_interfaces.msg import Parameter as RclParameter, ParameterType, ParameterValue

        if not client.service_is_ready():
            self.get_logger().warn(
                f"{label} costmap parameter service not available – "
                "footprint topic updated only."
            )
            return

        param_value = ParameterValue(
            type=ParameterType.PARAMETER_STRING,
            string_value=fp_string,
        )
        param = RclParameter(name=self.FOOTPRINT_PARAM, value=param_value)

        from rcl_interfaces.srv import SetParameters
        req = SetParameters.Request()
        req.parameters = [param]

        future = client.call_async(req)
        future.add_done_callback(
            lambda f: self._param_set_done(f, label)
        )

    def _param_set_done(self, future, label: str) -> None:
        try:
            result = future.result()
            if result.results and result.results[0].successful:
                self.get_logger().info(f"{label} costmap footprint parameter updated.")
            else:
                reason = result.results[0].reason if result.results else "unknown"
                self.get_logger().warn(
                    f"{label} costmap parameter update failed: {reason}"
                )
        except Exception as exc:
            self.get_logger().error(f"{label} costmap parameter call raised: {exc}")


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = FootprintPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()