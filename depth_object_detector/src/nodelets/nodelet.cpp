#include "nodelet.h"

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(depth_object_detector::DetectorNodelet, nodelet::Nodelet)

using namespace std;

namespace depth_object_detector
{

void DetectorNodelet::BoolCallback(const std_msgs::BoolConstPtr& msg)
{
    // should probably use actionlib for this instead
    if(running_ && !msg->data){
        subscriber_.shutdown();
        detected_ = false;
    }
    else if(!running_ && msg->data){
        detected_ = false;
        subscriber_ = node_.subscribe("image", 1, &DetectorNodelet::ImageCallback, this);
        std_msgs::Bool b;
        b.data = false;
        publisher_.publish(b);
    }
    running_ = msg->data;
}

// Callback for image data
void DetectorNodelet::ImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    //Number of points observed
    unsigned int n = 0;

    const uint16_t* depth_row = reinterpret_cast<const uint16_t*>(&msg->data[0]);
    int row_step = msg->step / sizeof(uint16_t);
    depth_row += (int)(min_y_*row_step);

//  std::cout << msg->width << "\t" << msg->height << "\t" << min_x_ << "\t" << max_x_ << std::endl;

    for (int v = min_y_; v < max_y_; ++v, depth_row += row_step)
    {
     for (int u = min_x_; u < max_x_; ++u)
     {
       //float depth = depth_image_proc::DepthTraits<float>::toMeters(depth_row[u]);
       float depth = ((float)depth_row[u])/1000; // realsense encodes depth in mm integers
 //      std::cout << depth << std::endl;
       if (!depth_image_proc::DepthTraits<float>::valid(depth) || depth > max_z_ || depth < min_z_) continue;
       n++;
     }
    }

//    std::cout << "detected: " << n << std::endl;

    std_msgs::Bool b;
    if(n > min_points_){
      b.data = true;
      running_ = false;
      subscriber_.shutdown();
    }
    else{
      b.data = false;
    }

    if(detected_ != b.data){
      detected_ = b.data;
      publisher_.publish(b);
    }
}

void DetectorNodelet::GetParameterValues()
{

    ros::NodeHandle &np = getPrivateNodeHandle();

    // Load node-wide configuration values.
    np.param("min_x", min_x_, 140);
    np.param("min_y", min_y_, 100);
    np.param("min_z", min_z_, 0.5f);
    np.param("max_x", max_x_, 180);
    np.param("max_y", max_y_, 140);
    np.param("max_z", max_z_, 0.7f);
    np.param("min_points", min_points_, 400);
}

void DetectorNodelet::onInit(){

    node_ =  getNodeHandle();

    GetParameterValues();
    publisher_ = node_.advertise<std_msgs::Bool>("detected", 1, true);

    running_ = false;
    detected_ = false;

    subscriber_bool_ = node_.subscribe("active", 1, &DetectorNodelet::BoolCallback, this);
}

} // namespace
