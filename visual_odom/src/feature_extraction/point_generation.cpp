/**
* This file is part of vulcan_location.
*
* Copyright (C) 2023 Vulcan YJX <vulcanyjx@163.com>
* For more information see <https://github.com/Vulcan-YJX/vulcan_location>
*
* vulcan_location is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* vulcan_location is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with stairs_detection. If not, see <http://www.gnu.org/licenses/>.
*/

#include "feature_extraction/point_generation.hpp"

PointGener::PointGener(CameraCalibT stereo_calib) {
  stereo_calib_ = stereo_calib;
  camera_info_ = std::make_shared<CameraInfo>();
}

PointGener::~PointGener() {}

bool PointGener::mono_point(const Eigen::Vector2d & keypoint, Eigen::Vector3d & output)
{
  output(0) = (keypoint(0) - stereo_calib_.K[2]) / stereo_calib_.K[0];
  output(1) = (keypoint(1) - stereo_calib_.K[5]) / stereo_calib_.K[4];
  output(2) = 1.0;
  return true;
}

bool PointGener::stereo_point(const Eigen::Vector3d & keypoint, Eigen::Vector3d & output)
{
  BackProjectMono(keypoint.head(2), output);
  double d = stereo_calib_.K[0] * stereo_calib_.P[3] / (keypoint(0) - keypoint(2));
  output = output * d;
  return true;
}

bool PointGener::stereo_point_cloud(const cv::Mat & depth_img,pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cv_color)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  for (int v = 0; v < depth_img.rows; v++) {
    for (int u = 0; u < depth_img.cols; u++) {
      if (depth_img.at<float>(v, u) == 0) continue;
      pcl::PointXYZRGB p;
      double x = (u - stereo_calib_.K[2]) / stereo_calib_.K[0];
      double y = (v - stereo_calib_.K[5]) / stereo_calib_.K[4];
      double depth = stereo_calib_.K[0] * stereo_calib_.P[3]  /
                      (depth_img.at<float>(v, u));
      p.x = x * depth;
      p.y = y * depth;
      p.z = depth;
      p.b = cv_color.data[v * cv_color.step + u * cv_color.channels() + 2];
      p.g = cv_color.data[v * cv_color.step + u * cv_color.channels() + 1];
      p.r = cv_color.data[v * cv_color.step + u * cv_color.channels() + 0];
      point_cloud->push_back(p);
    }
  }
  return point_cloud;
}

std::vector<cv::DMatch> PointGener::match_point(cv::Mat des1, cv::Mat des2)
{
  cv::BFMatcher bf(cv::NORM_HAMMING);
  std::vector<cv::DMatch> matches;
  bf.match(des1, des2, matches);

  double min_distance = matches[0].distance;
  double max_distance = matches[0].distance;

  for (const auto & match : matches) {
    if (match.distance < min_distance) min_distance = match.distance;
    if (match.distance > max_distance) max_distance = match.distance;
  }

  std::vector<cv::DMatch> good_matches;
  for (const auto & match : matches) {
    if (match.distance <= std::max(2 * min_distance, 30.0)) {
      good_matches.push_back(match);
    }
  }
  return good_matches;
}

std::vector<cv::DMatch> PointGener::filter_stereo_match(std::vector<cv::DMatch> good_matches)
{
  std::vector<cv::DMatch> best_matches;
  for (const auto & match : good_matches) {
  }
  return best_matches;
}
