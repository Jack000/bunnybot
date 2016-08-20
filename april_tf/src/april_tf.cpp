#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <apriltags_nodelet/AprilTagDetections.h>
#include <map>
#include <math.h>
#include <fstream>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>

ros::Subscriber sub;
ros::Publisher pub;
ros::Publisher pub_target;
ros::Publisher pub_markers;

tf::TransformListener* tf_listener = NULL;
tf::TransformBroadcaster* tf_broadcaster = NULL;

std::string map_frame = "map";
std::string persistence_file = "/home/ubuntu/april_tf.bag";

// store an exponential average of observed transforms in a map
std::map<int,tf::Transform> avg;

// we need this to persist apriltag messages even after it has disappeared from view
std::map<int,apriltags_nodelet::AprilTagDetection> apriltag_detections;

float pos_ratio = 0.1;
float rot_ratio = 0.02;
bool publish_markers = true;
bool publish_target = true;
float target_distance = 0.65;

void publish_detections(){
  // publish detections
  apriltags_nodelet::AprilTagDetections detections;
  detections.header.frame_id = map_frame;
  detections.header.stamp = ros::Time::now();

  std::map<int,apriltags_nodelet::AprilTagDetection>::iterator at;
  for(at = apriltag_detections.begin(); at != apriltag_detections.end(); at++){
    int id = at->first;
    apriltags_nodelet::AprilTagDetection d = at->second;

    detections.detections.push_back(d);
  }

  pub.publish(detections);

  visualization_msgs::MarkerArray marker_transforms;
  if(publish_markers){
    for(at = apriltag_detections.begin(); at != apriltag_detections.end(); at++){
      int id = at->first;
      apriltags_nodelet::AprilTagDetection d = at->second;
      visualization_msgs::Marker marker_transform;
      marker_transform.header.frame_id = map_frame;
      marker_transform.header.stamp = ros::Time::now();

      marker_transform.ns = boost::to_string(id).c_str();
      marker_transform.id = id;

      marker_transform.type = visualization_msgs::Marker::CUBE;
      marker_transform.scale.x = d.tag_size;
      marker_transform.scale.y = d.tag_size;
      marker_transform.scale.z = 0.01;

      marker_transform.action = visualization_msgs::Marker::ADD;
      marker_transform.pose = d.pose;
      marker_transform.color.r = 1.0;
      marker_transform.color.g = 0.0;
      marker_transform.color.b = 1.0;
      marker_transform.color.a = 1.0;
      marker_transforms.markers.push_back(marker_transform);
    }
  }

  if(publish_target){
    apriltags_nodelet::AprilTagDetections target_detections;
    target_detections.header.frame_id = map_frame;
    target_detections.header.stamp = ros::Time::now();
    for(at = apriltag_detections.begin(); at != apriltag_detections.end(); at++){
      int id = at->first;
      apriltags_nodelet::AprilTagDetection d = at->second;

      tf::Transform t;
      t.setOrigin(tf::Vector3(d.pose.position.x, d.pose.position.y, d.pose.position.z));

      tf::Quaternion q(d.pose.orientation.x, d.pose.orientation.y, d.pose.orientation.z, d.pose.orientation.w);
      t.setRotation(q);

      tf::Vector3 normal(0,0,target_distance);
      tf::Vector3 axis = t.getRotation().getAxis();
      tf::Vector3 ppos = t.getOrigin();

      float angle = t.getRotation().getAngle();
      normal = normal.rotate(axis, angle);

      apriltags_nodelet::AprilTagDetection det;
      det.header = d.header;
      det.id = id;
      det.tag_size = d.tag_size;
      det.corners2d = d.corners2d;

      geometry_msgs::Pose pose;

      pose.position.x = ppos.x()+normal.x();
      pose.position.y = ppos.y()+normal.y();
      pose.position.z = ppos.z();

      // ensure z component of target vector is close to zero, for move_base
      normal.setZ(0);

      // point x-axis toward the tag
      tf::Quaternion aq;
      aq.setEuler(0, 0, atan2(normal.y(), normal.x())+M_PI);

      geometry_msgs::Quaternion arrow_orientation;
      tf::quaternionTFToMsg(aq, arrow_orientation);

      pose.orientation = arrow_orientation;

      // todo: set orientation
      det.pose = pose;

      target_detections.detections.push_back(det);

      // add some arrows for visualization
      if(publish_markers){
        visualization_msgs::Marker marker_transform;
        marker_transform.header.frame_id = map_frame;
        marker_transform.header.stamp = ros::Time::now();

        marker_transform.ns = (boost::to_string(id)+"_target").c_str();
        marker_transform.id = id;

        marker_transform.type = visualization_msgs::Marker::ARROW;
        marker_transform.scale.x = d.tag_size*2;
        marker_transform.scale.y = d.tag_size/2;
        marker_transform.scale.z = d.tag_size/2;

        marker_transform.action = visualization_msgs::Marker::ADD;
        marker_transform.pose = pose;
        marker_transform.color.r = 0.0;
        marker_transform.color.g = 0.0;
        marker_transform.color.b = 1.0;
        marker_transform.color.a = 1.0;
        marker_transforms.markers.push_back(marker_transform);
      }
    }

    pub_target.publish(target_detections);
    if(publish_markers){
      pub_markers.publish(marker_transforms);
    }
  }
}

void detection_cb(const apriltags_nodelet::AprilTagDetectionsConstPtr& d){

  if(d->detections.size() == 0){
    return;
  }

  apriltags_nodelet::AprilTagDetection tag;
  geometry_msgs::PoseStamped camera_pose;
  geometry_msgs::PoseStamped p;

  for(int i=0; i < d->detections.size(); i++){
    tag = d->detections[i];

    if(tag.tag_size < 1 && tag.tag_size > 0.01){
      try{
        camera_pose.header = tag.header;
        camera_pose.pose = tag.pose;

        // use classic tf instead of tf2. there doesn't appear to be a transformpose() in tf2
        tf_listener->transformPose(map_frame, camera_pose, p);
      }
      catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
        return;
      }
std::cout << "Detecting: " << tag.id << " size: " << tag.tag_size << std::endl;

      tf::Transform t;
      t.setOrigin(tf::Vector3(p.pose.position.x, p.pose.position.y, p.pose.position.z));

      tf::Quaternion q(p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w);
      t.setRotation(q);

      // update averages
      std::map<int,tf::Transform>::iterator it;
      it = avg.find(tag.id);
      if(it == avg.end()){
        avg[tag.id] = t;
      }
      else{
        tf::Transform a;
        tf::Vector3 ppos = avg[tag.id].getOrigin();
        tf::Vector3 apos = t.getOrigin();
        tf::Vector3 npos = ppos.lerp(apos, pos_ratio);
        a.setOrigin(npos);

        tf::Quaternion prot = avg[tag.id].getRotation();
        tf::Quaternion arot = t.getRotation();
        tf::Quaternion nrot = prot.slerp(arot, rot_ratio);
        a.setRotation(nrot);

        avg[tag.id] = a;
      }

      // prepare new msg in map frame and store globally for persistence
      apriltags_nodelet::AprilTagDetection det;
      det.header = tag.header;
      det.header.frame_id = map_frame;
      det.id = tag.id;
      det.tag_size = tag.tag_size;
      det.corners2d = tag.corners2d;

      geometry_msgs::Pose pose;
      tf::Transform t2 = avg[tag.id];
      tf::Vector3 tp = t2.getOrigin();
      tf::Quaternion tr = t2.getRotation();
      tf::Vector3 tra = tr.getAxis();

      pose.position.x = tp.x();
      pose.position.y = tp.y();
      pose.position.z = tp.z();

      pose.orientation.x = tra.x();
      pose.orientation.y = tra.y();
      pose.orientation.z = tra.z();
      pose.orientation.w = tr.getW();

      det.pose = pose;

      apriltag_detections[tag.id] = det;
    }
  }

  publish_detections();
}

int main(int argc, char **argv){
  ros::init(argc, argv, "april_tf");
  ros::NodeHandle n;

  sub = n.subscribe("detections", 1, detection_cb);
  pub = n.advertise<apriltags_nodelet::AprilTagDetections>("detections_global", 1, true);
  pub_target = n.advertise<apriltags_nodelet::AprilTagDetections>("detections_target", 1, true);
  pub_markers = n.advertise<visualization_msgs::MarkerArray>("detections_marker", 1, true);
  tf_listener = new (tf::TransformListener);

  // load state from previous session
  std::ifstream file(persistence_file.c_str());
  if(file.good()){
    rosbag::Bag rbag(persistence_file);
    rosbag::View view(rbag, rosbag::TopicQuery("detections_global"));
    BOOST_FOREACH(rosbag::MessageInstance const m, view)
    {
      apriltags_nodelet::AprilTagDetections::ConstPtr saved_detections = m.instantiate<apriltags_nodelet::AprilTagDetections>();
      if (saved_detections != NULL){
        std::cout << "loaded saved data from " << persistence_file << " with " << saved_detections->detections.size() << " detections" << std::endl;
        for(int i=0; i < saved_detections->detections.size(); i++){
           apriltags_nodelet::AprilTagDetection d = saved_detections->detections[i];
           apriltag_detections[d.id] = d;
        }
        publish_detections();
      }
    }
    rbag.close();
  }

  ros::Rate rate(10);
  while (n.ok()){
    // spin
    ros::spinOnce();
    rate.sleep();
  }

  // write to persistence file on shutdown
  apriltags_nodelet::AprilTagDetections detections;
  detections.header.frame_id = map_frame;
  detections.header.stamp = ros::Time::now();

  std::map<int,apriltags_nodelet::AprilTagDetection>::iterator at;
  for(at = apriltag_detections.begin(); at != apriltag_detections.end(); at++){
    int id = at->first;
    apriltags_nodelet::AprilTagDetection d = at->second;

    detections.detections.push_back(d);
  }

  rosbag::Bag wbag;
  wbag.open(persistence_file, rosbag::bagmode::Write);
  wbag.write("detections_global", ros::Time::now(), detections);

  return 0;
}
