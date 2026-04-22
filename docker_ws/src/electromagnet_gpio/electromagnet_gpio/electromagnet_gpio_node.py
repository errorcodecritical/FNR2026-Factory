#!/usr/bin/env python3
"""ROS 2 node that drives an electromagnet from Raspberry Pi GPIO."""

from __future__ import annotations

from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

try:
    import RPi.GPIO as GPIO
except Exception:  # pragma: no cover - depends on target hardware image
    # Keep the node alive even when running off-target; it will log actions
    # instead of touching real hardware.
    GPIO = None

# Default BCM GPIO used by the electromagnet when no parameter overrides it.
EletroPin = 13


class ElectromagnetGpioNode(Node):
    def __init__(self):
        super().__init__('electromagnet_gpio')

        self._declare_params()
        self._load_params()

        self._gpio = None
        self._dry_run = True
        self._state: Optional[bool] = None

        # Hardware setup happens before subscribing so the pin is already in a
        # safe state when the first latched message arrives.
        self._configure_gpio()
        self._subscription = self.create_subscription(
            Bool,
            self._eletro_topic,
            self._eletro_cb,
            _latched_bool_qos(),
        )
        self._set_magnet(self._initial_enabled, reason='initial state')

        mode = 'dry-run' if self._dry_run else 'hardware'
        self.get_logger().info(
            f'electromagnet GPIO node ready; topic={self._eletro_topic}; '
            f'pin={self._gpio_pin}; active_high={self._active_high}; mode={mode}'
        )

    def destroy_node(self):
        try:
            # Always release the magnet on shutdown so the robot does not keep
            # holding a part after the node exits.
            self._set_magnet(False, reason='shutdown')
            if self._gpio is not None:
                self._gpio.cleanup(self._gpio_pin)
        finally:
            super().destroy_node()

    def _declare_params(self) -> None:
        self.declare_parameter('eletro_topic', 'eletro')
        self.declare_parameter('gpio_pin', EletroPin)
        self.declare_parameter('active_high', True)
        self.declare_parameter('initial_enabled', False)
        self.declare_parameter('mock_gpio', False)

    def _load_params(self) -> None:
        get = self.get_parameter
        self._eletro_topic = str(get('eletro_topic').value)
        self._gpio_pin = int(get('gpio_pin').value)
        self._active_high = bool(get('active_high').value)
        self._initial_enabled = bool(get('initial_enabled').value)
        self._mock_gpio = bool(get('mock_gpio').value)

    def _configure_gpio(self) -> None:
        if self._mock_gpio:
            self.get_logger().warning('mock_gpio=true; electromagnet outputs will only be logged')
            return

        if GPIO is None:
            self.get_logger().warning(
                'RPi.GPIO is not available; running electromagnet node in dry-run mode'
            )
            return

        try:
            GPIO.setwarnings(False)
            GPIO.setmode(GPIO.BCM)
            # Drive the startup level immediately during setup to avoid a short
            # pulse on the output pin.
            initial_level = GPIO.HIGH if self._pin_high_for(self._initial_enabled) else GPIO.LOW
            GPIO.setup(self._gpio_pin, GPIO.OUT, initial=initial_level)
        except Exception as exc:
            self.get_logger().warning(
                f'Failed to configure BCM GPIO {self._gpio_pin}: {exc}; '
                'running electromagnet node in dry-run mode'
            )
            return

        self._gpio = GPIO
        self._dry_run = False

    def _eletro_cb(self, msg: Bool) -> None:
        # The autonomy node publishes True at pickup and False at drop-off.
        self._set_magnet(bool(msg.data), reason=f'{self._eletro_topic} message')

    def _set_magnet(self, enabled: bool, reason: str) -> None:
        # Skip repeated writes when the requested state is already active.
        if self._state == enabled:
            return

        level_name = 'HIGH' if self._pin_high_for(enabled) else 'LOW'
        state_name = 'enabled' if enabled else 'disabled'
        if self._gpio is not None:
            self._gpio.output(self._gpio_pin, self._level_for(enabled))
            self.get_logger().info(
                f'electromagnet {state_name}; set BCM GPIO {self._gpio_pin} to {level_name} ({reason})'
            )
        else:
            self.get_logger().info(
                f'dry-run: would {state_name} electromagnet on BCM GPIO {self._gpio_pin} '
                f'with {level_name} ({reason})'
            )

        self._state = enabled

    def _pin_high_for(self, enabled: bool) -> bool:
        # Some relay boards energize on LOW instead of HIGH, so this helper
        # centralizes the polarity conversion.
        return enabled if self._active_high else not enabled

    def _level_for(self, enabled: bool):
        pin_high = self._pin_high_for(enabled)
        if self._gpio is not None:
            return self._gpio.HIGH if pin_high else self._gpio.LOW
        return pin_high


def _latched_bool_qos() -> QoSProfile:
    # Transient-local durability makes the last command available to late
    # subscribers, so the GPIO node learns the current magnet state even if it
    # starts after the autonomy node.
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def main(args=None):
    rclpy.init(args=args)
    node = ElectromagnetGpioNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
