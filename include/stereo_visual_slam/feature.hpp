#ifndef __FEATURE_H__
#define __FEATURE_H__

#include "stereo_visual_slam/frame.hpp"
#include "stereo_visual_slam/map_point.hpp"
#include <opencv2/opencv.hpp>

namespace StereoSLAM {
// Forward declaration of Frame class
class Frame;
class MapPoint;
class Feature {
public:
  Feature(const std::shared_ptr<Frame> frame, const cv::Point2f &point);
  Feature(const std::shared_ptr<Frame> frame, const cv::Point2f &point, const cv::Point2f &prevPoint);

  const cv::Point2f point;
  const cv::Point2f prevPoint = cv::Point2f(0.0, 0.0);
  const std::weak_ptr<Frame> framePtr;
  std::weak_ptr<MapPoint> mapPointPtr;
  bool isInlier = true;
};
} // namespace StereoSLAM

#endif // __FEATURE_H__