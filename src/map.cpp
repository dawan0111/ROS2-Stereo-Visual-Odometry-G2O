#include "basic_slam/map.hpp"

namespace StereoSLAM {
Map::Map() {}

bool Map::addMapPoint(std::shared_ptr<MapPoint> mapPoint) {
  if (mapPointPtrs_.find(mapPoint->id) == mapPointPtrs_.end()) {
    mapPointPtrs_[mapPoint->id] = mapPoint;
    activeMapPointPtrs_[mapPoint->id] = mapPoint;

    return true;
  }

  return false;
}

bool Map::addKeyframe(std::shared_ptr<Frame> frame) {
  if (keyFramePtrs_.find(frame->frameId) == keyFramePtrs_.end()) {
    keyFramePtrs_[frame->frameId] = frame;
    activeKeyFramePtrs_[frame->frameId] = frame;

    return true;
  }

  return false;
}

bool Map::addFrame(std::shared_ptr<Frame> frame) {
  if (framePtrs_.find(frame->frameId) == framePtrs_.end()) {
    framePtrs_[frame->frameId] = frame;
    return true;
  }

  return false;
}

Map::KeyFrameType &Map::getActiveKeyFrames() { return activeKeyFramePtrs_; }
Map::KeyFrameType &Map::getFrames() { return framePtrs_; }

Map::MapPointType &Map::getActiveMapPoints() { return activeMapPointPtrs_; }
} // namespace StereoSLAM