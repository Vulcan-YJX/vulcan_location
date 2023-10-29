
#ifndef ORB_DETECTOR_HPP__
#define ORB_DETECTOR_HPP__

#include <bitset>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <numeric>
#include <sstream>
#include <vector>

#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "vpi/Array.h"
#include "vpi/Image.h"
#include "vpi/OpenCVInterop.hpp"
#include "vpi/Pyramid.h"
#include "vpi/Stream.h"
#include "vpi/algo/ConvertImageFormat.h"
#include "vpi/algo/GaussianPyramid.h"
#include "vpi/algo/ImageFlip.h"
#include "vpi/algo/ORB.h"

class OrbDetector
{
private:
  // VPI objects that will be used
  VPIImage imgInput = NULL;
  VPIImage imgGrayScale = NULL;

  VPIPyramid pyrInput = NULL;
  VPIArray keypoints = NULL;
  VPIArray descriptors = NULL;
  VPIPayload orbPayload = NULL;
  VPIStream stream = NULL;

  // Define the algorithm parameters.
  VPIORBParams orbParams;

  VPIBackend backend;
  VPIBackend backendWithCPU;
  std::string strBackend;
  cv::Mat setup_img;

public:
  OrbDetector(std::string vpiBackend_, cv::Mat setImg);
  void init_orb_detector(
    int32_t intensityThreshold, int32_t maxFeaturesPerLevel, int16_t maxPyramidLevels);
  bool do_estimator(cv::Mat cvImage);
  cv::Mat draw_key_points(
    cv::Mat img, VPIKeypointF32 * kpts, VPIBriefDescriptor * descs, int numKeypoints);
  ~OrbDetector();
  VPIKeypointF32 * outKeypoints;
  VPIBriefDescriptor * outDescriptors;
};

#endif /*ORB_DETECTOR_HPP__*/
