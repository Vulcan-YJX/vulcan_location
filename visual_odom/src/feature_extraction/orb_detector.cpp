
#include "feature_extraction/orb_detector.hpp"

#include "vpi_utils.hpp"

OrbDetector::OrbDetector(std::string vpiBackend_, cv::Mat setImg)
{
  strBackend = vpiBackend_;
  setup_img = setImg;
}
void OrbDetector::init_orb_detector(
  int32_t intensityThreshold, int32_t maxFeaturesPerLevel, int16_t maxPyramidLevels)
{
  if (strBackend == "cpu") {
    backend = VPI_BACKEND_CPU;
  } else if (strBackend == "cuda") {
    backend = VPI_BACKEND_CUDA;
  } else {
    throw std::runtime_error(
      "Backend '" + strBackend + "' not recognized, it must be either cpu or cuda.");
  }
  // Use the selected backend with CPU to be able to read data back from CUDA to CPU for example.
  backendWithCPU = static_cast<VPIBackend>(backend | VPI_BACKEND_CPU);
  // =================================
  // Allocate all VPI resources needed

  // Create the stream where processing will happen
  CHECK_STATUS(vpiStreamCreate(0, &stream));

  CHECK_STATUS(vpiInitORBParams(&orbParams));

  orbParams.fastParams.intensityThreshold = intensityThreshold;
  orbParams.maxFeaturesPerLevel = maxFeaturesPerLevel;
  orbParams.maxPyramidLevels = maxPyramidLevels;

  // We now wrap the loaded image into a VPIImage object to be used by VPI.
  // VPI won't make a copy of it, so the original image must be in scope at all times.
  CHECK_STATUS(vpiImageCreateWrapperOpenCVMat(setup_img, 0, &imgInput));
  CHECK_STATUS(
    vpiImageCreate(setup_img.cols, setup_img.rows, VPI_IMAGE_FORMAT_U8, 0, &imgGrayScale));

  // For the output arrays capacity we can use the maximum number of features per level multiplied by the
  // maximum number of pyramid levels, this will be the de factor maximum for all levels of the input.
  int outCapacity = orbParams.maxFeaturesPerLevel * orbParams.maxPyramidLevels;

  // Create the output keypoint array.
  CHECK_STATUS(
    vpiArrayCreate(outCapacity, VPI_ARRAY_TYPE_KEYPOINT_F32, backendWithCPU, &keypoints));

  // Create the output descriptors array.  To output corners only use NULL instead.
  CHECK_STATUS(
    vpiArrayCreate(outCapacity, VPI_ARRAY_TYPE_BRIEF_DESCRIPTOR, backendWithCPU, &descriptors));

  // For the internal buffers capacity we can use the maximum number of features per level multiplied by 20.
  // This will make FAST find a large number of corners so then ORB can select the top N corners in
  // accordance to Harris score of each corner, where N = maximum number of features per level.
  int bufCapacity = orbParams.maxFeaturesPerLevel * 20;

  // Create the payload for ORB Feature Detector algorithm
  CHECK_STATUS(vpiCreateORBFeatureDetector(backend, bufCapacity, &orbPayload));
}

bool OrbDetector::do_estimator(cv::Mat cvImage)
{
  try {
    CHECK_STATUS(vpiImageSetWrappedOpenCVMat(imgInput, cvImage));
    // First convert input to grayscale
    CHECK_STATUS(vpiSubmitConvertImageFormat(stream, backend, imgInput, imgGrayScale, NULL));

    // Then, create the Gaussian Pyramid for the image and wait for the execution to finish
    CHECK_STATUS(vpiPyramidCreate(
      cvImage.cols, cvImage.rows, VPI_IMAGE_FORMAT_U8, orbParams.maxPyramidLevels, 0.5, backend,
      &pyrInput));
    CHECK_STATUS(
      vpiSubmitGaussianPyramidGenerator(stream, backend, imgGrayScale, pyrInput, VPI_BORDER_CLAMP));

    // Then get ORB features and wait for the execution to finish
    CHECK_STATUS(vpiSubmitORBFeatureDetector(
      stream, backend, orbPayload, pyrInput, keypoints, descriptors, &orbParams,
      VPI_BORDER_LIMITED));

    CHECK_STATUS(vpiStreamSync(stream));

    // =======================================
    // Output processing and saving it to disk

    // Lock output keypoints and scores to retrieve its data on cpu memory
    VPIArrayData outKeypointsData;
    VPIArrayData outDescriptorsData;
    VPIImageData imgData;
    CHECK_STATUS(
      vpiArrayLockData(keypoints, VPI_LOCK_READ, VPI_ARRAY_BUFFER_HOST_AOS, &outKeypointsData));
    CHECK_STATUS(
      vpiArrayLockData(descriptors, VPI_LOCK_READ, VPI_ARRAY_BUFFER_HOST_AOS, &outDescriptorsData));
    CHECK_STATUS(
      vpiImageLockData(imgGrayScale, VPI_LOCK_READ, VPI_IMAGE_BUFFER_HOST_PITCH_LINEAR, &imgData));

    outKeypoints = (VPIKeypointF32 *)outKeypointsData.buffer.aos.data;
    outDescriptors = (VPIBriefDescriptor *)outDescriptorsData.buffer.aos.data;

    cv::Mat img;
    CHECK_STATUS(vpiImageDataExportOpenCVMat(imgData, &img));

    // Draw the keypoints in the output image
    //  cv::Mat outImage = draw_key_points(img, outKeypoints, outDescriptors, *outKeypointsData.buffer.aos.sizePointer);

    // Save the output image to disk
    //  imwrite("orb_feature_detector_" + strBackend + ".png", outImage);

    // Done handling outputs, don't forget to unlock them.
    CHECK_STATUS(vpiImageUnlock(imgGrayScale));
    CHECK_STATUS(vpiArrayUnlock(keypoints));
    CHECK_STATUS(vpiArrayUnlock(descriptors));
  } catch (std::exception & e) {
    std::cerr << e.what() << std::endl;
    return false;
  }
  return true;
}

OrbDetector::~OrbDetector()
{
  vpiStreamSync(stream);

  vpiImageDestroy(imgInput);
  vpiImageDestroy(imgGrayScale);
  vpiArrayDestroy(keypoints);
  vpiArrayDestroy(descriptors);
  vpiPayloadDestroy(orbPayload);
  vpiStreamDestroy(stream);
}

cv::Mat OrbDetector::draw_key_points(
  cv::Mat img, VPIKeypointF32 * kpts, VPIBriefDescriptor * descs, int numKeypoints)
{
  cv::Mat out;
  img.convertTo(out, CV_8UC1);
  cvtColor(out, out, cv::COLOR_GRAY2BGR);

  if (numKeypoints == 0) {
    return out;
  }

  std::vector<int> distances(numKeypoints, 0);
  float maxDist = 0.f;

  for (int i = 0; i < numKeypoints; i++) {
    for (int j = 0; j < VPI_BRIEF_DESCRIPTOR_ARRAY_LENGTH; j++) {
      distances[i] += std::bitset<8 * sizeof(uint8_t)>(descs[i].data[j] ^ descs[0].data[j]).count();
    }
    if (distances[i] > maxDist) {
      maxDist = distances[i];
    }
  }

  uint8_t ids[256];
  std::iota(&ids[0], &ids[0] + 256, 0);
  cv::Mat idsMat(256, 1, CV_8UC1, ids);

  cv::Mat cmap;
  applyColorMap(idsMat, cmap, cv::COLORMAP_JET);

  for (int i = 0; i < numKeypoints; i++) {
    int cmapIdx = static_cast<int>(std::round((distances[i] / maxDist) * 255));

    circle(out, cv::Point(kpts[i].x, kpts[i].y), 3, cmap.at<cv::Vec3b>(cmapIdx, 0), -1);
  }

  return out;
}
