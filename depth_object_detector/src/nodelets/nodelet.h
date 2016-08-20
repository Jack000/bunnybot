#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <ros/forwards.h>
#include <ros/single_subscriber_publisher.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <depth_image_proc/depth_traits.h>

namespace depth_object_detector
{

  class DetectorNodelet : public nodelet::Nodelet
  {
      public:
        virtual void onInit();
        void BoolCallback(const std_msgs::BoolConstPtr& msg);
	void ImageCallback(const sensor_msgs::ImageConstPtr& msg);
	void GetParameterValues();

        private:

        ros::NodeHandle node_;
        ros::Publisher publisher_;
        ros::Subscriber subscriber_;
        ros::Subscriber subscriber_bool_;
        int min_x_;
        int min_y_;
        float min_z_;
        int max_x_;
        int max_y_;
        float max_z_;
        int min_points_;
        bool running_;
        bool detected_;
    };

}
