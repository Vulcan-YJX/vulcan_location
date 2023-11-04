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

#include "camera/camera.hpp"

Camera::Camera(
  int sensor_id, int capture_width, int capture_height, int display_width, int display_height,
  int framerate, int flip_method)
{
  // Create camera capture pipelines
  std::string gst_pipeline = gstreamer_pipeline(
    sensor_id, capture_width, capture_height, display_width, display_height, framerate,
    flip_method);
  cap(gst_pipeline, cv::CAP_GSTREAMER);
  if (!cap.isOpened()) {
    std::cerr << "Failed to open camera." << std::endl;
  }
}

std::string Camera::gstreamer_pipeline(
  int sensor_id, int capture_width, int capture_height, int display_width, int display_height,
  int framerate, int flip_method)
{
  return "nvarguscamerasrc sensor_id=" + std::to_string(sensor_id) +
         " ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(capture_width) +
         ", height=(int)" + std::to_string(capture_height) +
         ", format=(string)NV12, framerate=(fraction)" + std::to_string(framerate) +
         "/1 ! nvvidconv flip-method=" + std::to_string(flip_method) +
         " ! video/x-raw, width=(int)" + std::to_string(display_width) + ", height=(int)" +
         std::to_string(display_height) +
         ", format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink "
         "max-buffers=2 drop=true";
}

bool Camera::read_frame(cv::Mat & frame)
{
  cap.read(frame);
  if (frame.empty()) {
    return false;
  }
  return true;
}

Camera::~Camera() { cap.release(); }