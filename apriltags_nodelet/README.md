#Apriltags nodelet

A port of the apriltags node to a nodelet container. Also pulls OpenCV install from cv_bridge to avoid compile problems on Jetson TK1

the original node: https://github.com/personalrobotics/apriltags

##Installation

Install dependencies:  
> $ sudo aptitude install libcgal-dev

Clone this repo into your catkin workspace and build. Note that this library conflicts with the apriltags node, so delete it first then clone this repo.


##Print some AprilTags

Images of the default tag family can be found in `tags/36h11.ps`

To open this postscript file:  
> $ sudo apt-get install gv  
> $ gv 36h11.ps  

Alternatively you can download images of the tags from here:  
https://april.eecs.umich.edu/wiki/index.php/AprilTags  
There are *.tgz files for each family, including a *.ps file with one tag per page.

##Configuration

Use the same nodelet manager as your camera.

```
<node pkg="nodelet" type="nodelet" name="image_proc" args="load apriltags_nodelet/AprilNodelet camera_nodelet_manager">
    <param name="default_tag_size" value="0.15" />

    <param name="viewer" value="false" />
    <param name="publish_detections_image" value="false" />

    <param name="display_type" value="CUBE" />
    <param name="marker_thickness" value="0.02" />

    <rosparam param="tag_data">
      "1":
        size: 0.150
      "2":
        size: 0.100
      "3":
        size: 0.100
    </rosparam>

    <remap from="image" to="/camera/color/image_raw"/>
    <remap from="camera_info" to="/camera/color/camera_info"/>
    <remap from="marker_array" to="/apriltags/marker_array"/>
    <remap from="detections" to="/apriltags/detections"/>
</node>
``` 

