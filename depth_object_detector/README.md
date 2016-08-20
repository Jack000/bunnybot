#depth_object_detector

a nodelet that publishes a true/false based on whether an object is detected in a specific spot in a depth image. An object is detected when the minimum number of depth points is exceeded in a defined window of the depth image.

usage:

```
<node pkg="nodelet" type="nodelet" name="object_detector" args="load depth_object_detector/DetectorNodelet camera_nodelet_manager">
   <remap from="image" to="/camera/depth/image_rect" />
   <param name="min_x" value="200" />
   <param name="max_x" value="260" />
   <param name="min_y" value="95" />
   <param name="max_y" value="150" />
   <param name="min_z" value="0.30" />
   <param name="max_z" value="0.66" />
   <param name="min_points" value="10" />
</node>
```

this publishes true/false (an std_msgs/Bool) to /detected when more than 10 points are detected in the user defined window.
