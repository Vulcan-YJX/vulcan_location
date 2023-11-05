// #include "camera/camera.hpp"
#include <iostream>
#include <memory>

#include "camera/camera.hpp"
#include "opencv2/opencv.hpp"

int main(int argc, char const * argv[])
{
  // Image params
  int32_t W_input = 1280;
  int32_t H_input = 720;
  int32_t W_stereo = 960;
  int32_t H_stereo = 600;
  int32_t FPS = 60;
  int32_t FLIP_METHOD = 0;

  std::shared_ptr<Camera> left_camera =
    std::make_shared<Camera>(0, W_input, H_input, W_stereo, H_stereo, FPS, FLIP_METHOD);
  std::shared_ptr<Camera> right_camera =
    std::make_shared<Camera>(1, W_input, H_input, W_stereo, H_stereo, FPS, FLIP_METHOD);

  if (!left_camera->open_device()) {
    std::cerr << "wrong left_camera open" << std::endl;
  }
  if (!right_camera->open_device()) {
    std::cerr << "wrong right_camera open" << std::endl;
  }

  uint16_t frame_count = 0;
  cv::Mat left_img, right_img;
  while (frame_count < 10) {
    if (!left_camera->read_frame(left_img)) {
      std::cout << "left capture frame fail." << std::endl;
      continue;
    }
    if (!right_camera->read_frame(right_img)) {
      std::cout << "right capture frame fail." << std::endl;
      continue;
    }
    cv::imwrite("l.png", left_img);
    cv::imwrite("r.png", right_img);
    std::cout << "Image size: " << left_img.size() << std::endl;
    frame_count += 1;
  }
  return 0;
}

