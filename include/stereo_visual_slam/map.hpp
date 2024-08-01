#ifndef __MAP_H__
#define __MAP_H__

#include "basic_slam/frame.hpp"
#include "basic_slam/map_point.hpp"

namespace StereoSLAM {
class Map {

public:
  using MapPointType = std::unordered_map<u_int32_t, std::shared_ptr<MapPoint>>;
  using KeyFrameType = std::unordered_map<u_int32_t, std::shared_ptr<Frame>>;
  Map();
  bool addMapPoint(std::shared_ptr<MapPoint> mapPoint);
  bool addKeyframe(std::shared_ptr<Frame> frame);
  bool addFrame(std::shared_ptr<Frame> frame);

  KeyFrameType &getActiveKeyFrames();
  KeyFrameType &getFrames();
  MapPointType &getActiveMapPoints();

private:
  MapPointType mapPointPtrs_;
  MapPointType activeMapPointPtrs_;
  KeyFrameType framePtrs_;
  KeyFrameType keyFramePtrs_;
  KeyFrameType activeKeyFramePtrs_;
};
} // namespace StereoSLAM

#endif // __MAP_H__