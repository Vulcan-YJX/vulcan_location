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
  cv::VideoCapture video_cap;
  std::string gst_pipeline;

private:
  std::string gstreamer_pipeline(
    int sensor_id, int capture_width, int capture_height, int display_width, int display_height,
    int framerate, int flip_method);

public:
  Camera(
    int sensor_id, int capture_width, int capture_height, int display_width, int display_height,
    int framerate, int flip_method);
  bool open_device(void);
  bool read_frame(cv::Mat & frame);
  ~Camera();
};

#endif /*CAMERA_INFO_HPP_*/
