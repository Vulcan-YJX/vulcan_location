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

PointGener::PointGener(/* args */) {}

PointGener::~PointGener() {}

bool PointGener::mono_point(const Eigen::Vector2d & keypoint, Eigen::Vector3d & output)
{
  output(0) = (keypoint(0) - _cx) * _fx_inv;
  output(1) = (keypoint(1) - _cy) * _fy_inv;
  output(2) = 1.0;
  return true;
}

bool PointGener::stereo_point(const Eigen::Vector3d & keypoint, Eigen::Vector3d & output)
{
  BackProjectMono(keypoint.head(2), output);
  double d = _bf / (keypoint(0) - keypoint(2));
  output = output * d;
  return true;
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
