#!/usr/bin/env python3
"""
motor_driver.py
===============
ROS 2 Jazzy node that bridges the serial link to an Arduino motor controller.

Serial protocol (Arduino side)
-------------------------------
  TX (ROS → Arduino):
    "M<id> <speed>\n"   – set motor <id> (0-3) to PWM speed [-255, 255]
    "EA\n"              – request all four encoder values in one shot
    "S\n"               – stop all motors (sent on shutdown)

  RX (Arduino → ROS):
    "OK: Motor <id> set to <speed>\n"  – ack for every M command  (drained, not parsed)
    "ENCODERS: <v0> <v1> <v2> <v3>\n" – reply to EA

  Why EA instead of individual E<id> requests
  --------------------------------------------
  Each M<id> command causes the Arduino to emit an "OK:..." line.  If we use
  individual E<id> requests the OK replies pile up in the OS serial buffer and
  the next readline() consumes a stale OK instead of the expected ENCODER<id>
  reply, shifting every subsequent read by one slot.
  A single EA command (plus a buffer flush before the request) avoids this.

Subscriptions
-------------
  /motor{0-3}/cmd      std_msgs/Int16   – PWM command FROM mecanum_driver

Publications
------------
  /motor{0-3}/encoder  std_msgs/Int16   – accumulated encoder tick TO mecanum_driver

Parameters
----------
  serial_port          [str]   '/dev/ttyACM0'   – USB device path
  baud_rate            [int]   115200
  encoder_poll_rate    [float] 10.0  Hz         – how often encoders are polled
  cmd_timeout          [float] 0.2   s           – serial read timeout per line
  reconnect_delay      [float] 2.0   s           – post-open stabilisation delay
"""

import time
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from std_msgs.msg import Int16

try:
    import serial
    import serial.serialutil
except ImportError:
    raise SystemExit(
        "pyserial is not installed. Run:  pip install pyserial --break-system-packages"
    )


class MotorDriverNode(Node):

    def __init__(self):
        super().__init__('motor_driver')

        # ------------------------------------------------------------------ #
        # Parameters                                                           #
        # ------------------------------------------------------------------ #
        self._declare_params()

        self._port          = self.get_parameter('serial_port').value
        self._baud          = self.get_parameter('baud_rate').value
        self._timeout       = self.get_parameter('cmd_timeout').value
        self._reconnect_dly = self.get_parameter('reconnect_delay').value
        poll_hz             = self.get_parameter('encoder_poll_rate').value

        # ------------------------------------------------------------------ #
        # Serial connection                                                    #
        # ------------------------------------------------------------------ #
        self._serial: serial.Serial | None = None
        self._open_serial()

        # ------------------------------------------------------------------ #
        # Subscribers – motor commands                                         #
        # ------------------------------------------------------------------ #
        self._cmd_subs = []
        for i in range(4):
            sub = self.create_subscription(
                Int16,
                f'/motor{i}/cmd',
                lambda msg, idx=i: self._motor_cmd_cb(msg, idx),
                10,
            )
            self._cmd_subs.append(sub)

        # ------------------------------------------------------------------ #
        # Publishers – encoder values                                          #
        # ------------------------------------------------------------------ #
        self._enc_pubs = [
            self.create_publisher(Int16, f'/motor{i}/encoder', 10)
            for i in range(4)
        ]

        # ------------------------------------------------------------------ #
        # Encoder poll timer                                                   #
        # ------------------------------------------------------------------ #
        self._poll_timer = self.create_timer(1.0 / poll_hz, self._poll_encoders)

        self.get_logger().info(
            f"motor_driver ready  port={self._port}  baud={self._baud}  "
            f"poll={poll_hz:.1f} Hz"
        )

    # ======================================================================= #
    # Parameter declaration                                                    #
    # ======================================================================= #

    def _declare_params(self):
        def _d(desc, ptype=ParameterType.PARAMETER_NOT_SET):
            return ParameterDescriptor(description=desc)

        self.declare_parameter('serial_port',       '/dev/ttyACM0',
                               _d('USB serial device path, e.g. /dev/ttyACM0 or /dev/ttyUSB0'))
        self.declare_parameter('baud_rate',          115200,
                               _d('Serial baud rate'))
        self.declare_parameter('cmd_timeout',        0.2,
                               _d('Serial read timeout per line [s]'))
        self.declare_parameter('reconnect_delay',    2.0,
                               _d('Seconds to wait after opening serial port'))
        self.declare_parameter('encoder_poll_rate',  10.0,
                               _d('Encoder polling frequency [Hz]'))

    # ======================================================================= #
    # Serial helpers                                                           #
    # ======================================================================= #

    def _open_serial(self):
        try:
            self._serial = serial.Serial(
                self._port,
                self._baud,
                timeout=self._timeout,
            )
            time.sleep(self._reconnect_dly)
            self.get_logger().info(f"Opened serial port {self._port} @ {self._baud} baud.")
        except serial.serialutil.SerialException as exc:
            self._serial = None
            self.get_logger().error(
                f"Could not open serial port '{self._port}': {exc}. "
                "Motor commands will be silently dropped until the port is available."
            )

    def _write(self, data: bytes) -> bool:
        """Write to serial; attempt reconnect on failure. Returns True on success."""
        if self._serial is None or not self._serial.is_open:
            self._open_serial()

        if self._serial is None:
            return False

        try:
            self._serial.write(data)
            return True
        except serial.serialutil.SerialException as exc:
            self.get_logger().warn(f"Serial write error: {exc} – attempting reconnect.")
            try:
                self._serial.close()
            except Exception:
                pass
            self._serial = None
            return False

    def _readline(self) -> str | None:
        """Read a line from serial; returns None on error."""
        if self._serial is None or not self._serial.is_open:
            return None
        try:
            return self._serial.readline().decode('utf-8').strip()
        except (serial.serialutil.SerialException, UnicodeDecodeError) as exc:
            self.get_logger().warn(f"Serial read error: {exc}")
            return None

    def _flush_input(self):
        """Drain any pending bytes in the OS RX buffer (e.g. queued OK: replies)."""
        if self._serial and self._serial.is_open:
            try:
                self._serial.reset_input_buffer()
            except serial.serialutil.SerialException:
                pass

    # ======================================================================= #
    # Motor command callback                                                   #
    # ======================================================================= #

    def _motor_cmd_cb(self, msg: Int16, motor_id: int):
        speed = int(max(-100, min(100, msg.data)))
        cmd = f"M{motor_id} {speed}\n".encode('utf-8')
        if self._write(cmd):
            self.get_logger().debug(f"Motor {motor_id} → PWM {speed}")
        # Note: the Arduino will reply "OK: Motor X set to Y\n".
        # We intentionally do NOT read it here; _flush_input() in
        # _poll_encoders will drain these ack lines before the EA request.

    # ======================================================================= #
    # Encoder poll  – uses "EA" (all-encoders) command                        #
    # ======================================================================= #

    def _poll_encoders(self):
        """
        Send a single 'EA' command and parse the one-line response:
            ENCODERS: <v0> <v1> <v2> <v3>

        Flushing the RX buffer beforehand discards any accumulated
        "OK: Motor…" ack lines so we always read a fresh ENCODERS reply.
        """
        # Discard stale OK: ack lines from motor commands
        self._flush_input()

        if not self._write(b"EA\n"):
            return   # serial unavailable; skip cycle

        response = self._readline()
        if response is None:
            self.get_logger().warn("No response to EA command.")
            return

        # Expected: "ENCODERS: <v0> <v1> <v2> <v3>"
        prefix = "ENCODERS:"
        if not response.startswith(prefix):
            self.get_logger().warn(
                f"Unexpected EA response: '{response}'"
            )
            return

        try:
            values = response[len(prefix):].split()
            if len(values) != 4:
                self.get_logger().warn(
                    f"EA response has {len(values)} values, expected 4: '{response}'"
                )
                return

            for i, raw in enumerate(values):
                msg = Int16()
                msg.data = int(raw)
                self._enc_pubs[i].publish(msg)
                self.get_logger().debug(f"Encoder {i} = {raw}")

        except ValueError as exc:
            self.get_logger().warn(f"Could not parse EA response '{response}': {exc}")

    # ======================================================================= #
    # Cleanup                                                                  #
    # ======================================================================= #

    def destroy_node(self):
        if self._serial and self._serial.is_open:
            # Stop all motors before closing
            for i in range(4):
                try:
                    self._serial.write(f"M{i} 0\n".encode('utf-8'))
                except Exception:
                    pass
            self._serial.close()
            self.get_logger().info("Serial port closed.")
        super().destroy_node()


# ---------------------------------------------------------------------------
# Entry-point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = MotorDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()