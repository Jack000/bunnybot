#include "ros/ros.h"
#include "nodelet.h"

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(gpu_robot_vision::VisionNodelet, nodelet::Nodelet)

namespace gpu_robot_vision {

  void VisionNodelet::onInit(){
    load_params();
    ros::NodeHandle &nh = getNodeHandle();
    camera_set_=false;
    sub_info_ = nh.subscribe("camera_info", 5, &VisionNodelet::camera_info, this);
    sub_ = nh.subscribe("image_raw", 5, &VisionNodelet::process_image, this);
    pub_ = nh.advertise<sensor_msgs::Image>("image_rect", 10);
    pub_info_ = nh.advertise<sensor_msgs::CameraInfo>("camera_info_scaled", 10, true);
/*    cv::gpu::GpuMat tag_gpu(tag_); 
//std::cout << tag_gpu.rows << tag_gpu.cols << std::endl;
    cv::gpu::ORB_GPU detector(200, 1.3, 4, 9, 0,2,0,9);
    detector.blurForDescriptor = true;
    detector.setFastParams(50); // high contrast corners
    detector(tag_gpu, cv::gpu::GpuMat(), tag_keypoints_, tag_descriptors_);*/
  };

  void VisionNodelet::load_params(){
    ros::NodeHandle &nh = getPrivateNodeHandle();

    nh.param("scale", scale_, DEFAULT_SCALE);
    nh.param("scale_x", scale_x_, DEFAULT_SCALE_X);
    nh.param("scale_y", scale_y_, DEFAULT_SCALE_Y);

    nh.param("rectify", rectify_, DEFAULT_RECTIFY);
    nh.param("grayscale_filter", grayscale_filter_, DEFAULT_GRAYSCALE_FILTER);
    nh.param("noise_filter", noise_filter_, DEFAULT_NOISE_FILTER);
    nh.param("noise_filter_h", noise_filter_h_, DEFAULT_NOISE_FILTER_H);
    nh.param("noise_filter_search_window", noise_filter_search_window_, DEFAULT_NOISE_FILTER_SEARCH_WINDOW);
    nh.param("noise_filter_block_size", noise_filter_block_size_, DEFAULT_NOISE_FILTER_BLOCK_SIZE);
  }

  void VisionNodelet::process_image(const sensor_msgs::ImageConstPtr& frame){
    if(!camera_set_){
      return;
    }
    cv_bridge::CvImageConstPtr image = cv_bridge::toCvShare(frame);
    cv::gpu::GpuMat image_gpu(image->image);

    // grayscale
    cv::gpu::GpuMat image_mono;
    if(grayscale_filter_){
      image_mono = cv::gpu::GpuMat(cv::Size(image->image.rows, image->image.cols), CV_8UC1);
      cv::gpu::cvtColor(image_gpu, image_mono, CV_BGR2GRAY);
      image_gpu.release();
    }
    else{
      image_mono = image_gpu;
    }

    // denoise
    cv::gpu::GpuMat image_clean;
    if(noise_filter_){
      image_clean = cv::gpu::GpuMat(cv::Size(image->image.rows, image->image.cols), image_mono.type());
      cv::gpu::FastNonLocalMeansDenoising denoise;
      denoise.simpleMethod(image_mono, image_clean, noise_filter_h_, noise_filter_search_window_, noise_filter_block_size_);
      image_mono.release();
    }
    else{
      image_clean = image_mono;
    }

    // undistort
    cv::gpu::GpuMat image_rect;
    if(rectify_){
      image_rect = cv::gpu::GpuMat(cv::Size(image->image.rows, image->image.cols), image_clean.type());
      cv::gpu::remap(image_clean, image_rect, mapx_, mapy_, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
      image_clean.release();
    }
    else{
      image_rect = image_clean;
    }

    // scale
    cv::gpu::GpuMat image_resized;
    if(scale_){
      if(fabs(scale_x_-1.0f) > EPSILON || fabs(scale_y_-1.0f) > EPSILON){
        cv::gpu::resize(image_rect, image_resized, cv::Size(), scale_x_, scale_y_, cv::INTER_CUBIC);
      }
    }
    else{
      image_resized = image_rect;
    }

    // gpu adaptiveThreshold and extract markers
    // no gpu::adaptiveThreshold implementation, have to roll our own
/*    cv::gpu::GpuMat image_filtered(cv::Size(image->image.rows, image->image.cols), CV_8UC1);
    cv::gpu::GpuMat image_threshold(cv::Size(image->image.rows, image->image.cols), CV_8UC1);

    cv::gpu::boxFilter(image_rect, image_filtered, -1, cv::Size(5,5));
    cv::gpu::add(image_rect, 9, image_rect);
    cv::gpu::subtract(image_rect, image_filtered, image_rect);
    cv::gpu::threshold(image_rect, image_threshold, 1, 255, CV_THRESH_BINARY_INV);

    cv::Mat cpu_image(image_threshold);

    cv::Point2d opticalCenter = cv::Point2d(0.5*cpu_image.rows, 0.5*cpu_image.cols);

    // Detect AprilTag markers in the image
    TagDetectionArray detections;
    TagFamily* family_;
std::string tag_family_name_ = "Tag36h11";
TagDetector* detector_;

    family_ = new TagFamily(tag_family_name_);
TagDetectorParams tag_params;
tag_params.newQuadAlgorithm = 1;
    detector_ = new TagDetector(*family_, tag_params);
    detector_->process(cpu_image, opticalCenter, detections);

*/

    //cv::gpu::GpuMat image_gpu_output;
    //cv::gpu::cvtColor(image_threshold, image_gpu_output, CV_GRAY2RGB);
    //cv::Mat test(image_gpu_output);

/*    cv::Mat mat = cv::Mat(image_mono);
    std::vector< std::vector< cv::Point > > contours;
    std::vector< cv::Vec4i > hierarchy;

    cv::Mat test;
    cv::Canny(mat, mat, 100, 200);
    //cv::adaptiveThreshold(mat, mat, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY_INV, 9, 5);
    cv::cvtColor(mat, test, CV_GRAY2RGB);
    //cv::findContours(mat, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
/*
    // extract ORB features
    //std::vector<cv::KeyPoint> keypoints;
    cv::gpu::GpuMat keypoints;
    cv::gpu::GpuMat descriptors;

    cv::gpu::ORB_GPU detector(300, 1.3, 4, 15, 0, 2, 0,15);
    detector.blurForDescriptor = true;
    detector.setFastParams(80); // high contrast corners
    detector(image_gpu_rect, cv::gpu::GpuMat(), keypoints, descriptors);

    std::vector<cv::DMatch> matches;
    cv::gpu::BFMatcher_GPU matcher(cv::NORM_HAMMING);
    matcher.match(descriptors, tag_descriptors_, matches, cv::gpu::GpuMat());


    if(matches.size() < 70){
        return;
    }
*/
    // compute homography
    
//std::cout << matches.size() << std::endl;
  /*  cv::Mat image_output = cv::Mat(image_gpu_output);

    cv::Mat keyout;
    //cv::drawKeypoints(image_output, keypoints, keyout);
    std::vector<cv::KeyPoint> cpu_keypoints;
    detector.downloadKeyPoints(keypoints, cpu_keypoints);
    std::vector<cv::KeyPoint> cpu_tag_keypoints;
    detector.downloadKeyPoints(tag_keypoints_, cpu_tag_keypoints);


    cv::drawMatches(image_output, cpu_keypoints, tag_, cpu_tag_keypoints, matches, keyout);*/

    //cv::Mat test;
    cv::gpu::GpuMat image_output;
    if(grayscale_filter_){
      cv::gpu::cvtColor(image_resized, image_output, CV_GRAY2RGB);
    }
    else{
      image_output = image_resized;
    }
    cv::Mat out(image_output);
    cv_bridge::CvImage out_msg;
    out_msg.header   = frame->header;
    out_msg.encoding = frame->encoding;
    out_msg.image  = out;
    pub_.publish(out_msg.toImageMsg());

    if(scale_){
      info_.header.stamp = out_msg.header.stamp;
      pub_info_.publish(info_);
    }
  }

  void VisionNodelet::camera_info(const sensor_msgs::CameraInfoConstPtr& info_msg){

    camera_.fromCameraInfo(info_msg);
    cv::Mat m1;
    cv::Mat m2;
    cv::initUndistortRectifyMap(camera_.intrinsicMatrix(), camera_.distortionCoeffs(), cv::Mat(), camera_.intrinsicMatrix(), camera_.fullResolution(), CV_32FC1, m1, m2);
    mapx_ = cv::gpu::GpuMat(m1);
    mapy_ = cv::gpu::GpuMat(m2);

    // publish scaled camera_info
    //if(fabs(scale_x_-1.0f) > EPSILON || fabs(scale_y_-1.0f) > EPSILON){
    if(scale_){
      sensor_msgs::CameraInfo info;
      cv::Size resolution = camera_.fullResolution();
      info.header.stamp = ros::Time::now();
      info.header.frame_id = info_msg->header.frame_id;
      info.width = (int) (scale_x_*resolution.width);
      info.height = (int) (scale_y_*resolution.height);
      info.distortion_model = info_msg->distortion_model;
      info.R = info_msg->R;

      cv::Mat dc = camera_.distortionCoeffs();
      std::vector<double> D(dc.rows*dc.cols);
      D.assign((double*)dc.datastart, (double*)dc.dataend);;
      info.D = D;

      cv::Matx33d im = camera_.intrinsicMatrix();
      boost::array<double, 9> K;
      int i;
      for(i=0; i<9; i++){
        K[i] = im.val[i];
      }
      K[0] *= scale_x_;
      K[2] *= scale_x_;
      K[4] *= scale_y_;
      K[5] *= scale_y_;

      info.K = K;
      ///std::cout << scale_x_ << scale_y_ << "\t" << K[0] << std::endl;
      cv::Matx34d pm = camera_.projectionMatrix();
      boost::array<double, 12> P;
      for(i=0; i<12; i++){
        P[i] = pm.val[i];
      }

      P[0] *= scale_x_;
      P[2] *= scale_x_;
      P[5] *= scale_y_;
      P[6] *= scale_y_;
      info.P = P;

      info_ = info;
      pub_info_.publish(info);

     }
    sub_info_.shutdown();
    camera_set_ = true;
  }
} // namespace
