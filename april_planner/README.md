#april_planner

this node is a bit badly named as it doesn't output a planned path. Once your robot has view of an apriltag, you want to move the robot until it's at a specific position relative to the apriltag. This node issues cmd_vel messages based on input goal poses. It moves the robot until the goal is reached or the apriltag is no longer within view.

todo: integrate actionlib


example:

```
<node pkg="april_planner" type="april_planner" name="visual_planning" output="screen">
  <remap from="detections" to="/apriltags/detections" />
  <remap from="cmd_vel" to="/mobile_base/commands/velocity" />
</node>

assuming tag#1 is within view:

rostopic pub /goal geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: "1"}, pose: {position: {x: 0.0, y: 0.32, z: 0.0}}}'
```

this will move the robot until the center of the robot base is aligned with the apriltag, at a distance of 0.32 meters. The use of PoseStamped is a bit of a hack, but this avoids the need to create a custom message type.
