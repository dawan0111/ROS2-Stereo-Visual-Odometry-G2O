#ifndef __MAP_H__
#define __MAP_H__

#include "stereo_visual_slam/frame.hpp"
#include "stereo_visual_slam/map_point.hpp"

namespace StereoSLAM {
class Map {

public:
  using MapPointType = std::unordered_map<u_int32_t, std::shared_ptr<MapPoint>>;
  using KeyFrameType = std::map<u_int32_t, std::shared_ptr<Frame>>;

  Map(int16_t localWindowSize = 10);
  bool addMapPoint(std::shared_ptr<MapPoint> mapPoint);
  bool addKeyframe(std::shared_ptr<Frame> frame);
  bool removeActiveKeyframe(int16_t frameId);
  bool removeActiveMapPoint(u_int32_t mapPointId);
  void cleanMap();

  KeyFrameType &getActiveKeyFrames();
  KeyFrameType &getKeyFrames();
  MapPointType &getActiveMapPoints();
  MapPointType &getMapPoints();

  bool getRequiredViewerUpdated() { return requiredViewerUpdated_; }
  void setRequiredViewerUpdated(bool flag) { requiredViewerUpdated_ = flag; }

private:
  MapPointType mapPointPtrs_;
  MapPointType activeMapPointPtrs_;
  KeyFrameType keyFramePtrs_;
  KeyFrameType activeKeyFramePtrs_;

  int16_t localWindowSize_;

  bool requiredViewerUpdated_ = true;
};
} // namespace StereoSLAM

#endif // __MAP_H__