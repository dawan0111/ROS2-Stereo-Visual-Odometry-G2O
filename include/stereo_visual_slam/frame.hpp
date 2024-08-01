#ifndef __FRAME_H__
#define __FRAME_H__

#include "basic_slam/feature.hpp"
#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>

// Forward declaration of Feature class
class Feature;

namespace StereoSLAM {
class Frame {
public:
  Frame();
  void setKeyFrame();
  const int32_t frameId;
  cv::Mat image;
  cv::Mat grayImage;
  std::vector<std::shared_ptr<Feature>> featurePtrs;
  Sophus::SE3d T_wc; // Cam pose in World coordinate
  Sophus::SE3d T_d;  // PrevFrame -> CurrentFrame
  bool isKeyFrame = false;
  bool needUpdate = false;

private:
  static int32_t nextFrameId;
};
} // namespace StereoSLAM

#endif // __FRAME_H__