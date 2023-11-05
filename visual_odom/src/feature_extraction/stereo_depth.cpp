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

#include "feature_extraction/stereo_depth.hpp"

#include "vpi_utils.hpp"

StereoDepth::StereoDepth(/* args */) {}

void StereoDepth::init_depth_pipeline(
  std::string nvBackend, cv::Mat cvImageLeft, cv::Mat cvImageRight)
{
  strBackend = nvBackend;
  // =============================
  // Parse command line parameters
  if (strBackend == "cpu") {
    backends = VPI_BACKEND_CPU;
  } else if (strBackend == "cuda") {
    backends = VPI_BACKEND_CUDA;
  } else if (strBackend == "pva") {
    backends = VPI_BACKEND_PVA;
  } else if (strBackend == "ofa") {
    backends = VPI_BACKEND_OFA;
  } else if (strBackend == "ofa-pva-vic") {
    backends = VPI_BACKEND_OFA | VPI_BACKEND_PVA | VPI_BACKEND_VIC;
  } else {
    throw std::runtime_error(
      "Backend '" + strBackend +
      "' not recognized, it must be either cpu, cuda, pva, ofa, ofa-pva-vic.");
  }
  // =================================
  // Allocate all VPI resources needed

  int32_t inputWidth = cvImageLeft.cols;
  int32_t inputHeight = cvImageLeft.rows;

  // Create the stream that will be used for processing.
  CHECK_STATUS(vpiStreamCreate(0, &stream));
  // We now wrap the loaded images into a VPIImage object to be used by VPI.
  // VPI won't make a copy of it, so the original image must be in scope at all times.
  CHECK_STATUS(vpiImageCreateWrapperOpenCVMat(cvImageLeft, 0, &inLeft));
  CHECK_STATUS(vpiImageCreateWrapperOpenCVMat(cvImageRight, 0, &inRight));
  // Format conversion parameters needed for input pre-processing
  CHECK_STATUS(vpiInitConvertImageFormatParams(&convParams));
  // Set algorithm parameters to be used. Only values what differs from defaults will be overwritten.
  CHECK_STATUS(vpiInitStereoDisparityEstimatorCreationParams(&stereoParams));

  int stereoWidth = inputWidth;
  int stereoHeight = inputHeight;
  int outputWidth = inputWidth;
  int outputHeight = inputHeight;

  if (strBackend.find("ofa") != std::string::npos) {
    // Implementations using OFA require BL input
    stereoFormat = VPI_IMAGE_FORMAT_Y16_ER_BL;

    if (strBackend == "ofa") {
      disparityFormat = VPI_IMAGE_FORMAT_S16_BL;
    }

    // Output width including downscaleFactor must be at least max(64, maxDisparity/downscaleFactor) when OFA+PVA+VIC are used
    if (strBackend.find("pva") != std::string::npos) {
      int downscaledWidth =
        (inputWidth + stereoParams.downscaleFactor - 1) / stereoParams.downscaleFactor;
      int minWidth =
        std::max(stereoParams.maxDisparity / stereoParams.downscaleFactor, downscaledWidth);
      outputWidth = std::max(64, minWidth);
      outputHeight = (inputHeight * stereoWidth) / inputWidth;
      stereoWidth = outputWidth * stereoParams.downscaleFactor;
      stereoHeight = outputHeight * stereoParams.downscaleFactor;
    }
    // Maximum disparity can be either 128 or 256
    stereoParams.maxDisparity = 256;
  } else if (strBackend == "pva") {
    // PVA requires that input and output resolution is 480x270
    stereoWidth = outputWidth = 480;
    stereoHeight = outputHeight = 270;

    // maxDisparity must be 64
    stereoParams.maxDisparity = 64;
  }

  // Create the payload for Stereo Disparity algorithm.
  // Payload is created before the image objects so that non-supported backends can be trapped with an error.
  CHECK_STATUS(vpiCreateStereoDisparityEstimator(
    backends, stereoWidth, stereoHeight, stereoFormat, &stereoParams, &stereo));

  // Create the image where the disparity map will be stored.
  CHECK_STATUS(vpiImageCreate(outputWidth, outputHeight, disparityFormat, 0, &disparity));

  // Create the input stereo images
  CHECK_STATUS(vpiImageCreate(stereoWidth, stereoHeight, stereoFormat, 0, &stereoLeft));
  CHECK_STATUS(vpiImageCreate(stereoWidth, stereoHeight, stereoFormat, 0, &stereoRight));

  if (strBackend.find("ofa") != std::string::npos) {
    // OFA also needs a temporary buffer for format conversion
    CHECK_STATUS(vpiImageCreate(inputWidth, inputHeight, VPI_IMAGE_FORMAT_Y16_ER, 0, &tmpLeft));
    CHECK_STATUS(vpiImageCreate(inputWidth, inputHeight, VPI_IMAGE_FORMAT_Y16_ER, 0, &tmpRight));

    if (strBackend.find("pva") != std::string::npos) {
      // confidence map is supported by OFA+PVA
      CHECK_STATUS(
        vpiImageCreate(outputWidth, outputHeight, VPI_IMAGE_FORMAT_U16, 0, &confidenceMap));
    }
  } else if (strBackend == "pva") {
    // PVA also needs a temporary buffer for format conversion and rescaling
    CHECK_STATUS(vpiImageCreate(inputWidth, inputHeight, stereoFormat, 0, &tmpLeft));
    CHECK_STATUS(vpiImageCreate(inputWidth, inputHeight, stereoFormat, 0, &tmpRight));
  } else if (strBackend == "cuda") {
    CHECK_STATUS(vpiImageCreate(inputWidth, inputHeight, VPI_IMAGE_FORMAT_U16, 0, &confidenceMap));
  }
}

bool StereoDepth::do_estimate(cv::Mat cvImageLeft, cv::Mat cvImageRight, int thresholdValue)
{
  try {
    // ================
    // Processing stage
    if (strBackend == "pva" || strBackend == "ofa" || strBackend == "ofa-pva-vic") {
      // Convert opencv input to temporary grayscale format using CUDA
      CHECK_STATUS(
        vpiSubmitConvertImageFormat(stream, VPI_BACKEND_CUDA, inLeft, tmpLeft, &convParams));
      CHECK_STATUS(
        vpiSubmitConvertImageFormat(stream, VPI_BACKEND_CUDA, inRight, tmpRight, &convParams));

      // Do both scale and final image format conversion on VIC.
      CHECK_STATUS(vpiSubmitRescale(
        stream, VPI_BACKEND_VIC, tmpLeft, stereoLeft, VPI_INTERP_LINEAR, VPI_BORDER_CLAMP, 0));
      CHECK_STATUS(vpiSubmitRescale(
        stream, VPI_BACKEND_VIC, tmpRight, stereoRight, VPI_INTERP_LINEAR, VPI_BORDER_CLAMP, 0));
    } else {
      // Convert opencv input to grayscale format using CUDA
      CHECK_STATUS(
        vpiSubmitConvertImageFormat(stream, VPI_BACKEND_CUDA, inLeft, stereoLeft, &convParams));
      CHECK_STATUS(
        vpiSubmitConvertImageFormat(stream, VPI_BACKEND_CUDA, inRight, stereoRight, &convParams));
    }

    // ------------------------------
    // Do stereo disparity estimation
    // Submit it with the input and output images
    CHECK_STATUS(vpiSubmitStereoDisparityEstimator(
      stream, backends, stereo, stereoLeft, stereoRight, disparity, confidenceMap, NULL));

    // Wait until the algorithm finishes processing
    CHECK_STATUS(vpiStreamSync(stream));

    // ========================================
    // Output pre-processing and saving to disk
    // Lock output to retrieve its data on cpu memory
    VPIImageData data;
    CHECK_STATUS(
      vpiImageLockData(disparity, VPI_LOCK_READ, VPI_IMAGE_BUFFER_HOST_PITCH_LINEAR, &data));

    // Make an OpenCV matrix out of this image
    cv::Mat cvDisparity;
    CHECK_STATUS(vpiImageDataExportOpenCVMat(data, &cvDisparity));

    // Q10.5 -> float
    cvDisparity.convertTo(cvDisparity, CV_32F, 1.0 / 32.0, 0);

    // Done handling output, don't forget to unlock it.
    CHECK_STATUS(vpiImageUnlock(disparity));

    // If we have a confidence map,
    if (confidenceMap) {
      VPIImageData data;
      CHECK_STATUS(
        vpiImageLockData(confidenceMap, VPI_LOCK_READ, VPI_IMAGE_BUFFER_HOST_PITCH_LINEAR, &data));
      cv::Mat cvConfidence;
      CHECK_STATUS(vpiImageDataExportOpenCVMat(data, &cvConfidence));
      cvConfidence.convertTo(cvConfidence, CV_8UC1, 255.0 / 65535, 0);
      cv::Mat cvMask;
      cv::threshold(cvConfidence, cvMask, thresholdValue, 255, cv::THRESH_BINARY);

      CHECK_STATUS(vpiImageUnlock(confidenceMap));
      cv::Mat resultImage;
      cvDisparity.convertTo(cvMask, CV_32F, 1.0, 0);
      bitwise_and(cvDisparity, cvMask, cvDisparity);
    }
  } catch (std::exception & e) {
    std::cerr << e.what() << std::endl;
    return false;
  }
  return true;
  // cv::imwrite("disparity_" + strBackend + ".png", cvDisparityColor);
}

StereoDepth::~StereoDepth()
{
  vpiStreamDestroy(stream);
  vpiImageDestroy(inLeft);
  vpiImageDestroy(inRight);
  vpiImageDestroy(tmpLeft);
  vpiImageDestroy(tmpRight);
  vpiImageDestroy(stereoLeft);
  vpiImageDestroy(stereoRight);
  vpiImageDestroy(confidenceMap);
  vpiImageDestroy(disparity);
  vpiPayloadDestroy(stereo);
}
