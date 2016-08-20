#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <ros/forwards.h>
#include <ros/single_subscriber_publisher.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <visualization_msgs/Marker.h>
#include "opencv2/opencv.hpp"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"

#include <cv_bridge/cv_bridge.h>

#include <src/TagDetector.h>
#include <src/TagDetection.h>
#include <src/TagFamily.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <visualization_msgs/MarkerArray.h>
#include "yaml-cpp/yaml.h"
#include <sstream>
#include <fstream>

#include <boost/unordered_set.hpp>
#include <boost/unordered_map.hpp>
#include <boost/make_shared.hpp>

#include <apriltags/AprilTagDetections.h>

namespace apriltags_nodelet
{

const double SMALL_TAG_SIZE = 0.0358968;
const double MED_TAG_SIZE = 0.06096;
const double PAGE_TAG_SIZE = 0.165;

const std::string DEFAULT_TAG_FAMILY = "Tag36h11";
const std::string DEFAULT_IMAGE_TOPIC = "image";
const std::string DEFAULT_CAMERA_INFO_TOPIC = "camera_info";
const std::string DEFAULT_MARKER_TOPIC = "marker_array";
const std::string DEFAULT_DETECTIONS_TOPIC = "detections";
const std::string DEFAULT_DETECTIONS_IMAGE_TOPIC = "detections_image";
const double DEFAULT_TAG_SIZE = MED_TAG_SIZE;
const std::string DEFAULT_DISPLAY_TYPE = "CUBE";

  class AprilNodelet : public nodelet::Nodelet
  {
      public:
        virtual void onInit();
        ~AprilNodelet();
        double GetTagSize(int tag_id);
	void GetMarkerTransformUsingOpenCV(const TagDetection& detection, Eigen::Matrix4d& transform, cv::Mat& rvec, cv::Mat& tvec);
	void ArrowLine(cv::Mat& image,
		       const cv::Point& pt1, const cv::Point& pt2, const cv::Scalar& color,
		       const int thickness=1, const int line_type=8, const int shift=0,
		       const double tip_length=0.1);
	void DrawMarkerAxes(const cv::Matx33f& intrinsic_matrix, const cv::Vec4f& distortion_coeffs,
		            const cv::Mat& rvec, const cv::Mat& tvec, const float length, const bool use_arrows,
		            cv::Mat& image);
	void DrawMarkerOutline(const TagDetection& detection, const cv::Scalar outline_color, cv::Mat& image);
	void DrawMarkerEdges(const TagDetection& detection, cv::Mat& image);
	void DrawMarkerID(const TagDetection& detection, const cv::Scalar text_color, cv::Mat& image);
	void InfoCallback(const sensor_msgs::CameraInfoConstPtr& camera_info);
	void ImageCallback(const sensor_msgs::ImageConstPtr& msg);
	void GetParameterValues();
	void SetupPublisher();
	void InitializeTags();

        private:

        ros::NodeHandle node_;
        sensor_msgs::CameraInfo camera_info_;
        ros::Publisher marker_publisher_;
        ros::Publisher apriltag_publisher_;
        ros::Publisher image_publisher_;
        ros::Subscriber info_subscriber;
        ros::Subscriber image_subscriber;

        // AprilTag parts
        TagFamily* family_;
        TagDetector* detector_;

        TagDetectorParams tag_params;
        std::string tag_data;
        std::string tag_family_name_;

        // Settings and local information
        bool viewer_;
        bool publish_detections_image_;
        double default_tag_size_;
        double marker_thickness_;
        boost::unordered_map<size_t, double> tag_sizes_;
        bool running_;
        bool has_camera_info_;
        std::string display_type_;

        bool display_marker_overlay_;
        bool display_marker_outline_;
        bool display_marker_id_;
        bool display_marker_edges_;
        bool display_marker_axes_;

    };

}
