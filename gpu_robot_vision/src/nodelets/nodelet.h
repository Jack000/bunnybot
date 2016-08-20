#include <nodelet/nodelet.h>
#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include "opencv2/opencv.hpp"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/gpu/gpu.hpp"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_geometry/pinhole_camera_model.h>

namespace gpu_robot_vision
{

const bool  DEFAULT_SCALE = false;
const float DEFAULT_SCALE_X = 1;
const float DEFAULT_SCALE_Y = 1;

const bool DEFAULT_RECTIFY = true;
const bool DEFAULT_GRAYSCALE_FILTER = true;
const bool DEFAULT_NOISE_FILTER = false;
const float DEFAULT_NOISE_FILTER_H = 7;
const int DEFAULT_NOISE_FILTER_SEARCH_WINDOW = 21;
const int DEFAULT_NOISE_FILTER_BLOCK_SIZE = 7;

const float EPSILON = 0.000001;

    class VisionNodelet : public nodelet::Nodelet
    {
        public:
            virtual void onInit();
            void process_image(const sensor_msgs::ImageConstPtr& frame);
            void camera_info(const sensor_msgs::CameraInfoConstPtr& info_msg);
        private:
            void load_params();
            ros::Subscriber sub_;
            ros::Subscriber sub_info_;
            ros::Publisher pub_;
            ros::Publisher pub_info_;
            sensor_msgs::CameraInfo info_;
            image_geometry::PinholeCameraModel camera_;
            cv::gpu::GpuMat mapx_;
            cv::gpu::GpuMat mapy_;
            bool camera_set_;
            bool scale_;
            float scale_x_;
            float scale_y_;
            bool rectify_;
            bool grayscale_filter_;
            bool noise_filter_;
            float noise_filter_h_;
            int noise_filter_search_window_;
            int noise_filter_block_size_;
    };

}
