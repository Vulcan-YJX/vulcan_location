#ifndef POINT_GENERATION_HPP__
#define POINT_GENERATION_HPP__

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>
#include "camera/camera_info.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

class PointGener
{
private:
  std::shared_ptr<CameraInfo> camera_info_; 
  CameraCalibT stereo_calib_;

public:
  PointGener(/* args */);
  ~PointGener();
  bool mono_point(const Eigen::Vector2d & keypoint, Eigen::Vector3d & output);
  bool stereo_point(const Eigen::Vector3d & keypoint, Eigen::Vector3d & output);
  bool stereo_point_cloud(const cv::Mat & depth_img,pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cv_color);
  std::vector<cv::DMatch> match_point(cv::Mat des1, cv::Mat des2);
};

#endif /*POINT_GENERATION_HPP__*/
