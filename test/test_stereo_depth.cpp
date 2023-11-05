#include <opencv2/core/version.hpp>
#if CV_MAJOR_VERSION >= 3
#include <opencv2/imgcodecs.hpp>
#else
#include <opencv2/contrib/contrib.hpp>  // for colormap
#include <opencv2/highgui/highgui.hpp>
#endif

#include <vpi/Image.h>
#include <vpi/Status.h>
#include <vpi/Stream.h>
#include <vpi/algo/ConvertImageFormat.h>
#include <vpi/algo/Rescale.h>
#include <vpi/algo/StereoDisparity.h>

#include <cstring>  // for memset
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <sstream>
#include <vpi/OpenCVInterop.hpp>

#include "camera/camera.hpp"
#include "feature_extraction/stereo_depth.hpp"

int main(int argc, char * argv[])
{
  // OpenCV image that will be wrapped by a VPIImage.
  // Define it here so that it's destroyed *after* wrapper is destroyed
  cv::Mat cvImageLeft, cvImageRight;
  // Image params
  int32_t W_input = 1280;
  int32_t H_input = 720;
  int32_t W_stereo = 480;
  int32_t H_stereo = 270;
  int32_t FPS = 60;
  int32_t FLIP_METHOD = 0;

  std::shared_ptr<Camera> left_camera =
    std::make_shared<Camera>(0, W_input, H_input, W_stereo, H_stereo, FPS, FLIP_METHOD);
  std::shared_ptr<Camera> right_camera =
    std::make_shared<Camera>(1, W_input, H_input, W_stereo, H_stereo, FPS, FLIP_METHOD);

  std::shared_ptr<StereoDepth> stereo_depth = std::make_shared<StereoDepth>();

  if (!left_camera->open_device()) {
    std::cerr << "wrong left_camera open" << std::endl;
  }
  if (!right_camera->open_device()) {
    std::cerr << "wrong right_camera open" << std::endl;
  }

  if (!left_camera->read_frame(cvImageLeft)) {
    std::cout << "left capture frame fail." << std::endl;
  }
  if (!right_camera->read_frame(cvImageRight)) {
    std::cout << "right capture frame fail." << std::endl;
  }

  // =====================
  // Load the input images
  if (cvImageLeft.empty()) {
    throw std::runtime_error("Can't open '");
  }

  if (cvImageRight.empty()) {
    throw std::runtime_error("Can't open '");
  }
  stereo_depth->init_depth_pipeline("cuda", cvImageLeft, cvImageRight);
  if(!stereo_depth->do_estimate(cvImageLeft, cvImageRight,250)){
    std::cerr << "wrong generate stereo" << std::endl;
  }

  return 0;
}
