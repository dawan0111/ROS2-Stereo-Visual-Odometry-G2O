#include "stereo_visual_slam/map.hpp"

namespace StereoSLAM {
Map::Map(int16_t localWindowSize) : localWindowSize_(localWindowSize) {}

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

    if (localWindowSize_ < activeKeyFramePtrs_.size()) {
      removeActiveKeyframe(activeKeyFramePtrs_.begin()->first);
    }
    return true;
  }

  return false;
}

bool Map::removeActiveKeyframe(int16_t frameId) {
  auto it = activeKeyFramePtrs_.find(frameId);
  if (it != activeKeyFramePtrs_.end()) {
    auto &frame = activeKeyFramePtrs_[frameId];
    for (auto &feature : frame->featurePtrs) {
      if (!feature->mapPointPtr.expired()) {
        auto mapPoint = feature->mapPointPtr.lock();
        mapPoint->removeObserve(feature);
        removeActiveMapPoint(mapPoint->id);
      }
    }
    activeKeyFramePtrs_.erase(it);
    return true;
  }
  return false;
}

bool Map::removeActiveMapPoint(u_int32_t mapPointId) {
  if (activeMapPointPtrs_.find(mapPointId) != activeMapPointPtrs_.end()) {
    auto obsCount = activeMapPointPtrs_[mapPointId]->getObservationCount();
    // std::cout << "ObsCount: " << obsCount << std::endl;
    if (obsCount == 0) {
      activeMapPointPtrs_[mapPointId]->isLocalPoint = false;
      activeMapPointPtrs_.erase(mapPointId);
      return true;
    }
  }
  return false;
}

Map::KeyFrameType &Map::getActiveKeyFrames() { return activeKeyFramePtrs_; }
Map::KeyFrameType &Map::getKeyFrames() { return keyFramePtrs_; }

Map::MapPointType &Map::getActiveMapPoints() { return activeMapPointPtrs_; }
Map::MapPointType &Map::getMapPoints() { return mapPointPtrs_; }
} // namespace StereoSLAM