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

#include <cstring>  // for memset
#include <iostream>
#include <sstream>

#include "camera/camera.hpp"
#include "opencv2/core/version.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "vpi/Image.h"
#include "vpi/OpenCVInterop.hpp"
#include "vpi/Status.h"
#include "vpi/Stream.h"
#include "vpi/algo/ConvertImageFormat.h"
#include "vpi/algo/Rescale.h"
#include "vpi/algo/StereoDisparity.h"

#define CHECK_STATUS(STMT)                              \
  do {                                                  \
    VPIStatus status = (STMT);                          \
    if (status != VPI_SUCCESS) {                        \
      char buffer[VPI_MAX_STATUS_MESSAGE_LENGTH];       \
      vpiGetLastStatusMessage(buffer, sizeof(buffer));  \
      std::ostringstream ss;                            \
      ss << vpiStatusGetName(status) << ": " << buffer; \
      throw std::runtime_error(ss.str());               \
    }                                                   \
  } while (0);

class stereo_depth
{
private:
  Camera stereo_camera_;
  uint64_t backends;
  uint16_t thresholdValue;
  std::string strBackend;

  // VPI objects that will be used
  VPIImage inLeft = NULL;
  VPIImage inRight = NULL;
  VPIImage tmpLeft = NULL;
  VPIImage tmpRight = NULL;
  VPIImage stereoLeft = NULL;
  VPIImage stereoRight = NULL;
  VPIImage disparity = NULL;
  VPIImage confidenceMap = NULL;
  VPIStream stream = NULL;
  VPIPayload stereo = NULL;

  VPIImageFormat stereoFormat;
  VPIImageFormat disparityFormat;
  VPIConvertImageFormatParams convParams;
  VPIStereoDisparityEstimatorCreationParams stereoParams;

public:
  stereo_depth(/* args */) {}
  ~stereo_depth();
  void init_stereo(const std::string vpiBackend, uint16_t set_confidence);
  bool do_estimator(cv::Mat cvImageLeft, cv::Mat cvImageRight, cv::Mat & cvDisparity);
};
