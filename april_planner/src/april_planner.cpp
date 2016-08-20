#include "ros/ros.h"
#include <tf/transform_datatypes.h>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include <apriltags_nodelet/AprilTagDetections.h>

ros::Publisher pub;
ros::Subscriber sub;

geometry_msgs::Pose goal;
int goal_id;

// offset between the camera perspective and center of rotation of differential drive robot
double camera_offset_y = 0.205;

// realsense rgb camera is mounted slightly to the right of center
double camera_offset_x = 0.04;

double max_vel = 0.3;
double max_vel_theta = 0.2;

int lost_count = 0;

void control_step(const apriltags_nodelet::AprilTagDetectionsConstPtr& d){
//  std::cout << d->header << std::endl;
//  std::cout << d->detections.size() << std::endl;

  if(d->detections.size() == 0){
    std::cout << "Warning: april planner lost detection" << std::endl;
    lost_count++;
    if(lost_count > 3){
      sub.shutdown();
    }
    return;
  }

  apriltags_nodelet::AprilTagDetection tag;
  bool found = false;
  for(int i=0; i < d->detections.size(); i++){
    if(d->detections[i].id == goal_id && d->detections[i].tag_size < 1 && d->detections[i].tag_size > 0.01){
      tag = d->detections[i];
      found = true;
      break;
    }
  //  std::cout << d->detections[i].id << " | " << d->detections[i].tag_size << std::endl;
  }

  if(found==false){
    return;
  }

  lost_count = 0;

  double x = tag.pose.position.x + camera_offset_x;
  double y = tag.pose.position.z - camera_offset_y;
  double theta = atan((x - goal.position.x)/(y - goal.position.y));

  // stop heading correction in last 5cm
  //if(y < goal.position.y + 0.05){
  //  theta = 0;
  //}
  geometry_msgs::Twist msg;

  double vel_x = std::min(std::max(0.5*(y-goal.position.y), (double)0.05), max_vel);
  msg.linear.x = vel_x;
  /*if(fabs(theta) < 0.05){
    msg.linear.x = max_vel;
  }
  else{
    msg.linear.x = 0;
  }*/

  theta *= 3*std::max(y-goal.position.y-0.05, (double) 0); // scale theta linearly to smooth angular motion
  msg.angular.z = -((theta > 0) - (theta < 0))*std::min(fabs(theta),max_vel_theta); // simple proportional control

  std::cout << theta << "\t" << y << std::endl;

  if(y < goal.position.y){
    // stop subscribing after goal is reached
    sub.shutdown();
    return;
  }
  pub.publish(msg);
}


void goal_init(const geometry_msgs::PoseStampedConstPtr& goal_input){
  goal = goal_input->pose;
  goal_id = atoi(goal_input->header.frame_id.c_str());
  ros::NodeHandle n;
  sub = n.subscribe("detections", 1, control_step);
}


int main(int argc, char **argv){
  ros::init(argc, argv, "april_planner");
  ros::NodeHandle n;
  ros::Subscriber goal_sub = n.subscribe("goal", 1, goal_init);
  pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  ros::spin();
}
