#gpu_robot_vision

A ROS nodelet wrapper for OpenCV's gpu functions. This nodelet is useful to reduce CPU usage in rectifying large video streams. The OpenCV install linked by ROS must have the gpu apis (ie. compiled with gpu support or using OpenCV4Tegra)

Because CUDA pointers cannot be shared between nodelets, this one nodelet contains various CV related functions in one package so that upload/download to gpu memory only needs to happen once.

example launch:
this launch file takes raw depth data from an intel realsense, rectifies it according to the input camera info, scales it by 0.5 and outputs the resulting image in image_rect. It also outputs a scaled camera_info for use by other nodes down the chain.

```
<node pkg="nodelet" type="nodelet" name="camera_nodelet_manager"  args="manager" />

<node pkg="nodelet" type="nodelet" name="depth_proc" args="load gpu_robot_vision/VisionNodelet camera_nodelet_manager" output="screen">
  <remap from="camera_info" to="/camera/depth/camera_info" />
  <remap from="image_raw" to="/camera/depth/image_raw" />
  <remap from="image_rect" to="/camera/depth/image_rect" />
  <remap from="camera_info_scaled" to="/camera/depth/camera_info_scaled" />

  <param name="scale" value="true" />
  <param name="rectify" value="true" />
  <param name="grayscale_filter" value="false" />

  <param name="scale_x" value="0.5" />
  <param name="scale_y" value="0.5" />
</node>
```
