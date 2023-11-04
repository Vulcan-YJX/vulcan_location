#ifndef POINT_GENERATION_HPP__
#define POINT_GENERATION_HPP__

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>

class PointGener
{
private:
  /* data */
public:
  PointGener(/* args */);
  ~PointGener();
  bool mono_point(const Eigen::Vector2d & keypoint, Eigen::Vector3d & output);
  bool stereo_point(const Eigen::Vector3d & keypoint, Eigen::Vector3d & output);
  std::vector<cv::DMatch> match_point(cv::Mat des1, cv::Mat des2);
};

#endif /*POINT_GENERATION_HPP__*/
