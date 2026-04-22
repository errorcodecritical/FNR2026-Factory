# electromagnet_gpio

ROS 2 package that subscribes to the `eletro` topic and drives the Raspberry Pi
GPIO pin used by the robot electromagnet.

- Topic: `eletro` (`std_msgs/Bool`)
- `true`: turn magnet on
- `false`: turn magnet off

The package is intended to run in its own container with GPIO access.
