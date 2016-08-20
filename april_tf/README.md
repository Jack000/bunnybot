#april_tf

note: this package depends on apriltags_nodelet for the apriltags detections message type.

This node serves 3 main purposes:

- re-publish an apriltags detections.msg in a global map frame, which remains even after the tag has moved out of view
- persist these global detections between ROS sessions by saving state in a .bag file (so the next time you start up the detections will still be there)
- for each detection, publish a "target" pose for use by move_base. This pose should ensure that the tag detection is visible to the robot

basically this allows us to mark points of interest on a map by apriltags.

usage: 
```
input:
~detections - apriltag detections.msg messages

output: 
~detections_global - all previously observed tags in map frame
~detections_target - target pose for each global pose
~detections_marker - markers for rviz

the persistence filename is hardcoded to /home/ubuntu/april_tf.bag, just change the variable.
```

example launch:
```
<node pkg="april_tf" type="april_tf" name="april_map_broadcaster" output="screen">
  <remap from="detections" to="/apriltags/detections" />
</node>
```
