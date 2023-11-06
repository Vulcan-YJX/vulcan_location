#include <memory>
#include <iostream>
#include "camera/camera_info.hpp"
#include "opencv2/opencv.hpp"

int main(int argc, char const *argv[])
{
    std::shared_ptr<CameraInfo> camera_info = std::make_shared<CameraInfo>();
    if(!camera_info->init_camera_info("config/camera_info.yaml")){
        std::cerr << "Read camera init file wrong!" << std::endl;
        return -1;
    }
    cv::Mat left_img = cv::imread("./data/left.jpg");
    cv::Mat right_img = cv::imread("./data/right.jpg");

    if (left_img.empty() || right_img.empty()) {
        std::cerr << "Read image file wrong!" << std::endl;
    }
    cv::Mat undistort_left,undistort_right;
    camera_info->undistort_stereo(left_img,right_img,undistort_left,undistort_right);
    cv::imwrite("undistort_left.png",undistort_left);
    cv::imwrite("undistort_right.png",undistort_right);
    std::cout << "Undistort finish!" << std::endl;
    return 0;
}
