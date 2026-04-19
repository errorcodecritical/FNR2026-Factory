#!/usr/bin/env python3
"""
mecanum_driver.py
=================
ROS 2 Jazzy node for a 4-wheel mecanum robot.

Subscriptions
-------------
  /cmd_vel              geometry_msgs/Twist   – desired robot velocity
  /motor{0-3}/encoder   std_msgs/Int16        – accumulated encoder ticks

Publications
------------
  /motor{0-3}/cmd       std_msgs/Int16        – PWM command  [-255, 255]
  /odom                 nav_msgs/Odometry     – dead-reckoning odometry
  /tf                   (odom → base_link)    – TF broadcast
  /wheel_debug          std_msgs/Float32MultiArray – PI debug data (see below)

Wheel layout (top view)
-----------------------
        FRONT
   FL(2)     FR(3)
   RL(0)     RR(1)
        REAR

Mecanum roller convention: front-left & rear-right have rollers at +45°,
front-right & rear-left have rollers at −45°.

Motor control
-------------
Each wheel has an independent PI velocity controller running at `control_rate`
Hz (default 50 Hz).  The inverse-kinematics setpoint [rad/s] is tracked using:

  feedforward  =  ω_setpoint / max_motor_speed × 255
  PI output    =  Kp × e  +  Ki × ∫e dt          (integrator clamped)
  PWM          =  clamp(feedforward + PI output, -255, 255)

Setting Ki = 0.0 degrades to pure feedforward (original open-loop behaviour).

/wheel_debug layout  (16 floats, published at control_rate)
-----------------------------------------------------------
  Index  Field
  -----  ------------------------------------------------
   0     wheel 0 (RL) – setpoint  [rad/s]
   1     wheel 0 (RL) – measured  [rad/s]
   2     wheel 0 (RL) – error     [rad/s]  (setpoint − measured)
   3     wheel 0 (RL) – PWM       [-255..255]
   4     wheel 1 (RR) – setpoint  [rad/s]
   5     wheel 1 (RR) – measured  [rad/s]
   6     wheel 1 (RR) – error     [rad/s]
   7     wheel 1 (RR) – PWM       [-255..255]
   8     wheel 2 (FL) – setpoint  [rad/s]
   9     wheel 2 (FL) – measured  [rad/s]
  10     wheel 2 (FL) – error     [rad/s]
  11     wheel 2 (FL) – PWM       [-255..255]
  12     wheel 3 (FR) – setpoint  [rad/s]
  13     wheel 3 (FR) – measured  [rad/s]
  14     wheel 3 (FR) – error     [rad/s]
  15     wheel 3 (FR) – PWM       [-255..255]

rqt_plot examples
-----------------
  # All four setpoints vs measured on one chart:
  /wheel_debug/data[0] /wheel_debug/data[1]   # RL
  /wheel_debug/data[4] /wheel_debug/data[5]   # RR
  /wheel_debug/data[8] /wheel_debug/data[9]   # FL
  /wheel_debug/data[12] /wheel_debug/data[13] # FR

  # All four tracking errors:
  /wheel_debug/data[2] /wheel_debug/data[6] /wheel_debug/data[10] /wheel_debug/data[14]

  # All four PWM outputs:
  /wheel_debug/data[3] /wheel_debug/data[7] /wheel_debug/data[11] /wheel_debug/data[15]

Parameters (all settable via ROS 2 parameter server)
-----------------------------------------------------
  wheel_radius        [m]      0.0325   (Ø65 mm wheel)
  wheel_separation_x  [m]      0.160    half track-width  (left ↔ right)
  wheel_separation_y  [m]      0.140    half wheelbase    (front ↔ rear)
  max_motor_speed     [rad/s]  10.0     speed that maps to PWM ±255
  motor_ppr           [array]  [881.0, 899.0, 1495.0, 900.0]  pulses per revolution per motor
                                        Motor 0 (RL): 881 PPR – Rear Left
                                        Motor 1 (RR): 899 PPR – Rear Right
                                        Motor 2 (FL): 1495 PPR (different gearing) – Front Left
                                        Motor 3 (FR): 900 PPR – Front Right
  odom_topic          [str]    "odom_wheel"   topic name for odometry publication
  odom_frame_id       [str]    "odom_wheel"   frame_id written into the Odometry message header
  base_frame_id       [str]    "base_link"
  publish_odom_tf     [bool]   False    broadcast odom→base_link TF (disable when using EKF)
  publish_wheel_tf    [bool]   False    broadcast base_link→wheel_* TFs for joint states

  control_rate        [Hz]     50.0     PI control loop rate
  pid_kp              [–]      22.0     proportional gain  (PWM per rad/s error)
  pid_ki              [–]      45.0     integral gain      (PWM per rad error)
  pid_i_clamp         [–]      100.0    integrator anti-windup clamp  [PWM units]

  odom_rate           [Hz]     20.0     odometry publish rate
  publish_wheel_debug [bool]   True     publish /wheel_debug topic
"""

import math
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16, Float32MultiArray, MultiArrayDimension
from tf2_ros import TransformBroadcaster


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


class WheelPI:
    """Lightweight PI controller for one wheel."""

    def __init__(self, kp: float = 0.0, ki: float = 0.0, i_clamp: float = 80.0):
        self.kp        = kp
        self.ki        = ki
        self.i_clamp   = i_clamp
        self._integral = 0.0

    def reset(self):
        self._integral = 0.0

    def update(self, setpoint: float, measured: float, dt: float) -> float:
        if dt <= 0.0:
            return 0.0
        error = setpoint - measured
        self._integral = clamp(
            self._integral + error * dt,
            -self.i_clamp,
            self.i_clamp,
        )
        return self.kp * error + self.ki * self._integral


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class MecanumDriverNode(Node):

    TWO_PI            = 2.0 * math.pi
    _WHEEL_NAMES  = ('RL', 'RR', 'FL', 'FR')
    _DEBUG_FIELDS = ('setpoint', 'measured', 'error', 'pwm')
    _DEBUG_LEN    = len(_WHEEL_NAMES) * len(_DEBUG_FIELDS)   # 16

    def __init__(self):
        super().__init__('mecanum_driver')

        # ------------------------------------------------------------------ #
        # Parameters                                                           #
        # ------------------------------------------------------------------ #
        self._declare_params()
        self._load_params()

        self.get_logger().info(
            f"Mecanum driver initialised  r={self._r:.4f} m  "
            f"lx={self._lx:.3f} m  ly={self._ly:.3f} m  "
            f"Motor PPR={self._motor_ppr}  "
            f"Kp={self._kp:.2f}  Ki={self._ki:.2f}  "
            f"control_rate={self._control_rate:.0f} Hz  "
            f"odom_topic={self._odom_topic}  "
            f"odom_frame_id={self._odom_frame}"
        )

        # ------------------------------------------------------------------ #
        # PI controllers                                                       #
        # ------------------------------------------------------------------ #
        self._pi = [
            WheelPI(self._kp, self._ki, self._i_clamp)
            for _ in range(4)
        ]

        # ------------------------------------------------------------------ #
        # State                                                                #
        # ------------------------------------------------------------------ #
        self._enc_last       = [None] * 4
        self._enc_last_time  = [None] * 4
        self._omega_meas     = [0.0]  * 4
        self._omega_setpoint = [0.0]  * 4
        self._enc_delta_odom = [0.0]  * 4

        self._x   = 0.0
        self._y   = 0.0
        self._yaw = 0.0

        # ------------------------------------------------------------------ #
        # Publishers                                                           #
        # ------------------------------------------------------------------ #
        self._motor_pubs = [
            self.create_publisher(Int16, f'/motor{i}/cmd', 10)
            for i in range(4)
        ]

        # Odometry topic name is now driven by the 'odom_topic' parameter.
        self._odom_pub = self.create_publisher(Odometry, self._odom_topic, 10)

        if self._pub_odom_tf or self._pub_wheel_tf:
            self._tf_broadcaster = TransformBroadcaster(self)

        if self._publish_wheel_debug:
            self._debug_pub = self.create_publisher(
                Float32MultiArray, '/wheel_debug', 10
            )
            self._debug_msg = Float32MultiArray()
            self._debug_msg.layout.data_offset = 0
            for wi, wname in enumerate(self._WHEEL_NAMES):
                for fi, fname in enumerate(self._DEBUG_FIELDS):
                    dim = MultiArrayDimension()
                    dim.label  = f'{wname}_{fname}'
                    dim.size   = 1
                    dim.stride = 1
                    self._debug_msg.layout.dim.append(dim)
            self._debug_msg.data = [0.0] * self._DEBUG_LEN

        # ------------------------------------------------------------------ #
        # Subscribers                                                          #
        # ------------------------------------------------------------------ #
        self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_cb, 10)

        for i in range(4):
            self.create_subscription(
                Int16,
                f'/motor{i}/encoder',
                lambda msg, idx=i: self._encoder_cb(msg, idx),
                10,
            )

        # ------------------------------------------------------------------ #
        # Timers                                                               #
        # ------------------------------------------------------------------ #
        self._control_timer = self.create_timer(
            1.0 / self._control_rate, self._control_loop
        )
        self._odom_timer = self.create_timer(
            1.0 / self._odom_rate, self._publish_odom
        )

        self.add_on_set_parameters_callback(self._on_params_changed)

        self.get_logger().info("mecanum_driver node ready.")

    # ======================================================================= #
    # Parameter helpers                                                        #
    # ======================================================================= #

    def _declare_params(self):
        def _d(desc):
            return ParameterDescriptor(description=desc)

        self.declare_parameter('wheel_radius',        0.0325, _d('Wheel radius [m]'))
        self.declare_parameter('wheel_separation_x',  0.160,  _d('Half track-width – left to right centre [m]'))
        self.declare_parameter('wheel_separation_y',  0.140,  _d('Half wheelbase – front to rear centre [m]'))
        self.declare_parameter('max_motor_speed',     10.0,   _d('Wheel angular speed [rad/s] that maps to PWM 255'))
        self.declare_parameter('motor_ppr', [881.0, 899.0, 1495.0, 900.0],
                               _d('Motor PPR per index [Motor0=RL, Motor1=RR, Motor2=FL, Motor3=FR]'))
        self.declare_parameter('encoder_ppr',         234.3,  _d('(Deprecated) Use motor_ppr instead'))
        # odom_topic controls both the ROS topic name and, independently,
        # odom_frame_id controls the frame_id written into the message header.
        # Default to "odom_wheel" for both so this source does not conflict
        # with rf2o (odom_rf2o) or a fused EKF output (odom).
        self.declare_parameter('odom_topic',    'odom_wheel', _d('Odometry topic name'))
        self.declare_parameter('odom_frame_id', 'odom_wheel', _d('Odometry header frame_id'))
        self.declare_parameter('base_frame_id', 'base_link',  _d('Robot base frame id'))
        # publish_odom_tf: broadcast the odom->base_link transform.
        # Keep False when robot_localization EKF is the authoritative TF source.
        self.declare_parameter('publish_odom_tf',  False, _d('Broadcast odom->base_link TF (disable when using EKF)'))
        # publish_wheel_tf: broadcast base_link->wheel_* transforms.
        # Needed when no joint_state_publisher / robot_state_publisher is running.
        self.declare_parameter('publish_wheel_tf', False, _d('Broadcast base_link->wheel_* TFs'))
        self.declare_parameter('control_rate',        50.0,  _d('PI control loop rate [Hz]'))
        self.declare_parameter('odom_rate',           20.0,  _d('Odometry publish rate [Hz]'))
        self.declare_parameter('pid_kp',              22.0,  _d('Proportional gain [PWM / (rad/s)]'))
        self.declare_parameter('pid_ki',              45.0,  _d('Integral gain [PWM / rad]'))
        self.declare_parameter('pid_i_clamp',         100.0, _d('Integrator anti-windup clamp [PWM units]'))
        self.declare_parameter('publish_wheel_debug', True,  _d('Publish /wheel_debug Float32MultiArray for rqt_plot'))

    def _load_params(self):
        g = self.get_parameter
        self._r                   = g('wheel_radius').value
        self._lx                  = g('wheel_separation_x').value
        self._ly                  = g('wheel_separation_y').value
        self._l                   = self._lx + self._ly
        self._max_speed           = g('max_motor_speed').value
        self._motor_ppr           = g('motor_ppr').value
        self._odom_topic          = g('odom_topic').value       # ← new
        self._odom_frame          = g('odom_frame_id').value
        self._base_frame          = g('base_frame_id').value
        self._pub_odom_tf         = g('publish_odom_tf').value
        self._pub_wheel_tf        = g('publish_wheel_tf').value
        self._control_rate        = g('control_rate').value
        self._odom_rate           = g('odom_rate').value
        self._kp                  = g('pid_kp').value
        self._ki                  = g('pid_ki').value
        self._i_clamp             = g('pid_i_clamp').value
        self._publish_wheel_debug = g('publish_wheel_debug').value

    def _on_params_changed(self, params):
        """Allow live tuning of PI gains without restarting the node."""
        gain_params = {'pid_kp', 'pid_ki', 'pid_i_clamp'}
        for p in params:
            if p.name == 'pid_kp':
                self._kp = p.value
            elif p.name == 'pid_ki':
                self._ki = p.value
            elif p.name == 'pid_i_clamp':
                self._i_clamp = p.value

        if any(p.name in gain_params for p in params):
            for pi in self._pi:
                pi.kp      = self._kp
                pi.ki      = self._ki
                pi.i_clamp = self._i_clamp
                pi.reset()
            self.get_logger().info(
                f"PI gains updated  Kp={self._kp:.3f}  "
                f"Ki={self._ki:.3f}  i_clamp={self._i_clamp:.1f}"
            )

        return SetParametersResult(successful=True)

    # ======================================================================= #
    # cmd_vel callback  ->  IK setpoints                                       #
    # ======================================================================= #

    def _cmd_vel_cb(self, msg: Twist):
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z
        r  = self._r
        l  = self._l

        w_fl = (vx - vy - l * wz) / r
        w_fr = (vx + vy + l * wz) / r
        w_rl = (vx + vy - l * wz) / r
        w_rr = (vx - vy + l * wz) / r

        self._omega_setpoint = [w_rl, w_rr, w_fl, w_fr]

        if vx == 0.0 and vy == 0.0 and wz == 0.0:
            for pi in self._pi:
                pi.reset()

    # ======================================================================= #
    # Encoder callbacks                                                        #
    # ======================================================================= #

    def _encoder_cb(self, msg: Int16, idx: int):
        now = self.get_clock().now()
        raw = msg.data

        if self._enc_last[idx] is None:
            self._enc_last[idx]      = raw
            self._enc_last_time[idx] = now
            return

        diff = raw - self._enc_last[idx]
        if diff >  32767:
            diff -= 65536
        elif diff < -32768:
            diff += 65536

        self._enc_last[idx] = raw

        ppr = self._motor_ppr[idx] if idx < len(self._motor_ppr) else self._motor_ppr[0]
        delta_rad = (diff / ppr) * self.TWO_PI

        dt_ns = (now - self._enc_last_time[idx]).nanoseconds
        if dt_ns > 0:
            self._omega_meas[idx] = delta_rad / (dt_ns * 1e-9)

        self._enc_last_time[idx] = now
        self._enc_delta_odom[idx] += delta_rad

    # ======================================================================= #
    # Control loop                                                             #
    # ======================================================================= #

    def _control_loop(self):
        dt = 1.0 / self._control_rate
        debug_data = [0.0] * self._DEBUG_LEN

        for i in range(4):
            sp   = self._omega_setpoint[i]
            meas = self._omega_meas[i]
            err  = sp - meas

            ff         = sp / self._max_speed * 255.0
            correction = self._pi[i].update(sp, meas, dt)
            pwm        = int(clamp(round(ff + correction), -255.0, 255.0))

            cmd = Int16()
            cmd.data = pwm
            self._motor_pubs[i].publish(cmd)

            base = i * len(self._DEBUG_FIELDS)
            debug_data[base + 0] = float(sp)
            debug_data[base + 1] = float(meas)
            debug_data[base + 2] = float(err)
            debug_data[base + 3] = float(pwm)

        if self._publish_wheel_debug:
            self._debug_msg.data = debug_data
            self._debug_pub.publish(self._debug_msg)

    # ======================================================================= #
    # Odometry publish                                                         #
    # ======================================================================= #

    def _publish_odom(self):
        dw = self._enc_delta_odom[:]
        self._enc_delta_odom = [0.0] * 4

        rl, rr, fl, fr = dw

        r  = self._r
        l  = self._l
        dt = 1.0 / self._odom_rate

        d_x_body = r / 4.0       * ( fl + fr + rl + rr)
        d_y_body = r / 4.0       * (-fl + fr + rl - rr)
        d_yaw    = r / (4.0 * l) * (-fl + fr - rl + rr)

        cos_yaw = math.cos(self._yaw + d_yaw / 2.0)
        sin_yaw = math.sin(self._yaw + d_yaw / 2.0)

        self._x   += d_x_body * cos_yaw - d_y_body * sin_yaw
        self._y   += d_x_body * sin_yaw + d_y_body * cos_yaw
        self._yaw += d_yaw
        self._yaw  = math.atan2(math.sin(self._yaw), math.cos(self._yaw))

        now      = self.get_clock().now().to_msg()
        half_yaw = self._yaw / 2.0

        odom = Odometry()
        odom.header.stamp    = now
        odom.header.frame_id = self._odom_frame   # driven by odom_frame_id param
        odom.child_frame_id  = self._base_frame

        odom.pose.pose.position.x    = self._x
        odom.pose.pose.position.y    = self._y
        odom.pose.pose.position.z    = 0.0
        odom.pose.pose.orientation.z = math.sin(half_yaw)
        odom.pose.pose.orientation.w = math.cos(half_yaw)

        odom.twist.twist.linear.x  = d_x_body / dt
        odom.twist.twist.linear.y  = d_y_body / dt
        odom.twist.twist.angular.z = d_yaw    / dt

        pose_cov     = [0.0] * 36
        pose_cov[0]  = 1e-3
        pose_cov[7]  = 1e-3
        pose_cov[35] = 1e-2
        odom.pose.covariance = pose_cov

        twist_cov     = [0.0] * 36
        twist_cov[0]  = 1e-3
        twist_cov[7]  = 1e-3
        twist_cov[35] = 1e-2
        odom.twist.covariance = twist_cov

        self._odom_pub.publish(odom)

        # --- odom -> base_link TF -------------------------------------------
        if self._pub_odom_tf:
            tf = TransformStamped()
            tf.header.stamp            = now
            tf.header.frame_id         = self._odom_frame
            tf.child_frame_id          = self._base_frame
            tf.transform.translation.x = self._x
            tf.transform.translation.y = self._y
            tf.transform.translation.z = 0.0
            tf.transform.rotation.z    = math.sin(half_yaw)
            tf.transform.rotation.w    = math.cos(half_yaw)
            self._tf_broadcaster.sendTransform(tf)

        # --- base_link -> wheel_* TFs ---------------------------------------
        # These are static-ish transforms that reflect each wheel's joint
        # position relative to the base.  Useful when robot_state_publisher
        # is not running (e.g. no URDF loaded).  The rotation about Y encodes
        # accumulated wheel angle from odometry (visualisation only — not used
        # for localisation).
        if self._pub_wheel_tf:
            wheel_joints = [
                # (child_frame,          x,                    y,                     z)
                ('wheel_rear_left',   -self._lx,  self._ly,  0.0),
                ('wheel_rear_right',  -self._lx, -self._ly,  0.0),
                ('wheel_front_left',   self._lx,  self._ly,  0.0),
                ('wheel_front_right',  self._lx, -self._ly,  0.0),
            ]
            wheel_tfs = []
            for i, (child, wx, wy, wz) in enumerate(wheel_joints):
                # Accumulated angle for this wheel [rad] from odometry deltas.
                # We reuse _enc_delta_odom *before* it was zeroed above (dw).
                wheel_angle = dw[i] / self._r if self._r > 0 else 0.0
                half_wa = wheel_angle / 2.0
                wtf = TransformStamped()
                wtf.header.stamp    = now
                wtf.header.frame_id = self._base_frame
                wtf.child_frame_id  = child
                wtf.transform.translation.x = wx
                wtf.transform.translation.y = wy
                wtf.transform.translation.z = wz
                # Rotation about Y axis (wheel spin)
                wtf.transform.rotation.y = math.sin(half_wa)
                wtf.transform.rotation.w = math.cos(half_wa)
                wheel_tfs.append(wtf)
            self._tf_broadcaster.sendTransform(wheel_tfs)


# ---------------------------------------------------------------------------
# Entry-point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = MecanumDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()