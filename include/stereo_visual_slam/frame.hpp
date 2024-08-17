#ifndef __FRAME_H__
#define __FRAME_H__

#include "stereo_visual_slam/feature.hpp"
#include <fbow/fbow.h>
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
  cv::Mat imageL;
  cv::Mat imageR;
  std::vector<std::shared_ptr<Feature>> featurePtrs;
  std::vector<std::shared_ptr<Feature>> rightFeaturePtrs;
  cv::Mat briefDesc;
  fbow::fBow fBowFeature;
  Sophus::SE3d T_wc; // Cam pose in World coordinate
  Sophus::SE3d T_d;  // PrevFrame -> CurrentFrame
  bool isKeyFrame = false;
  bool needUpdate = false;

private:
  static int32_t nextFrameId;
};
} // namespace StereoSLAM

#endif // __FRAME_H__