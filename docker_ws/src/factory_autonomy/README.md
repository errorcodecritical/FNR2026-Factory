# factory_autonomy

ROS2 autonomy node for RobotAtFactory 4.0.

The node follows the chapter 6 competition server protocol. At the start of
the run it repeatedly sends `IWP` to the UDP server. While the server answers
`STOP`, the robot remains stopped. When the server answers a four-letter part
string such as `BGGR`, the robot may start moving.

## Server Requests

- `IWP`: incoming warehouse slots.
- `OWP`: outgoing warehouse slots.
- `MAP`: machine type A slots.
- `MBP`: machine type B slots.
- `CTL`: competition time left.
- `PING`: connection check.

Slot responses are four letters using:

- `B`: final blue part.
- `G`: intermediate green part.
- `R`: raw red part.
- `X`: empty slot.

Machine slots use the server's `1 2 / 3 4` layout. Slots `1` and `3` are
inputs; slots `2` and `4` are outputs.

## Production Logic

- `B` goes to the outgoing warehouse.
- `G` goes to machine type B.
- `R` goes to machine type A.
- Raw `R` parts are moved before intermediate `G` parts.
- Machine output slots are unloaded before final incoming `B` parts are moved.

The robot refreshes `IWP`, `OWP`, `MAP`, and `MBP` between moves. If no move is
available because machines are processing or target slots are full, it waits and
polls again.

## Topics

- Publishes `/factory_autonomy/status` as JSON in `std_msgs/String`.
- Publishes `/factory_autonomy/selected_goal` as the latest `PoseStamped` sent
  to Nav2.
- Publishes `eletro` as `std_msgs/Bool`, where `true` turns the electromagnet
  on at pickup and `false` turns it off at drop-off.
- Uses the Nav2 `navigate_to_pose` action server by default.
