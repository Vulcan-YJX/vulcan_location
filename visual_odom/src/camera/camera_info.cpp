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

#include "camera/camera_info.hpp"

CameraInfo::CameraInfo() {}

bool CameraInfo::init_camera_info(const std::string & camera_file)
{
  cv::FileStorage camera_configs(camera_file, cv::FileStorage::READ);
  if (!camera_configs.isOpened()) {
    std::cerr << "ERROR: Wrong path to settings" << std::endl;
    return false;
  }

  _image_height = camera_configs["image_height"];
  _image_width = camera_configs["image_width"];
  _base_line = camera_configs["base_line"];

  camera_configs["LEFT.K"] >> K_l;
  camera_configs["RIGHT.K"] >> K_r;

  camera_configs["LEFT.T"] >> T_l;
  camera_configs["RIGHT.T"] >> T_r;

  camera_configs["LEFT.R"] >> R_l;
  camera_configs["RIGHT.R"] >> R_r;

  camera_configs["LEFT.D"] >> D_l;
  camera_configs["RIGHT.D"] >> D_r;

  if (
    K_l.empty() || K_r.empty() || T_l.empty() || T_r.empty() || R_l.empty() || R_r.empty() ||
    D_l.empty() || D_r.empty() || _image_height == 0 || _image_width == 0) {
    std::cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << std::endl;
    return false;
  }
  size = cv::Size(_image_width, _image_height);
  // _fx = P_l.at<double>(0, 0);
  // _fy = P_l.at<double>(1, 1);
  // _cx = P_l.at<double>(0, 2);
  // _cy = P_l.at<double>(1, 2);
  // _fx_inv = 1.0 / _fx;
  // _fy_inv = 1.0 / _fy;
  cv::stereoRectify(
    K_l, D_l, K_r, D_r, size, R_r, T_r, R1, R2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY, 0, size,
    &valid_roi1, &valid_roi2);
  cv::initUndistortRectifyMap(K_l, D_l, R1, P1, size, CV_32FC1, _mapl1, _mapl2);
  cv::initUndistortRectifyMap(K_r, D_r, R2, P2, size, CV_32FC1, _mapr1, _mapr2);
  return true;
}

void CameraInfo::undistort_stereo(
  cv::Mat & image_left, cv::Mat & image_right, cv::Mat & image_left_rect,
  cv::Mat & image_right_rect)
{
  cv::remap(image_left, image_left_rect, _mapl1, _mapl2, cv::INTER_LINEAR);
  cv::remap(image_right, image_right_rect, _mapr1, _mapr2, cv::INTER_LINEAR);
}
