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

#ifndef CAMERA_INFO_HPP_
#define CAMERA_INFO_HPP_

#include <yaml-cpp/yaml.h>
#include <iostream>
#include "opencv2/opencv.hpp"

class Camera
{
private:
  int _image_height;
  int _image_width;

  double _base_line;

  cv::Mat _mapl1;
  cv::Mat _mapl2;
  cv::Mat _mapr1;
  cv::Mat _mapr2;

public:
  cv::Mat K_l, K_r, T_l, T_r, R_l, R_r, D_l, D_r;
  cv::Mat R1, R2, P1, P2, Q;
  cv::Rect valid_roi1, valid_roi2;
  cv::Size size;

public:
    Camera(/* args */);
    bool init_camera_info(const std::string& camera_file);
    void undistort_stereo(
      cv::Mat& image_left, cv::Mat& image_right, cv::Mat& image_left_rect, cv::Mat& image_right_rect);
    ~Camera();
};

#endif /*CAMERA_INFO_HPP_*/
