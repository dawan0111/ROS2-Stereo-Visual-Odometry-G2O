#include "stereo_visual_slam/map_point.hpp"
namespace StereoSLAM {
int32_t MapPoint::nextMapPointId = 0;
MapPoint::MapPoint() : id(++nextMapPointId), observationCount_(0) {}

MapPoint::MapPoint(Eigen::Vector3d &worldPoint) : id(++nextMapPointId), worldPoint_(worldPoint), observationCount_(0) {}

int16_t MapPoint::getObservationCount() const { return observationCount_; }

bool MapPoint::addObserve(std::shared_ptr<Feature> feature) {
  ++observationCount_;
  obsFeatures_.push_back(feature);
  return true;
}

bool MapPoint::removeObserve(std::shared_ptr<Feature> feature) {
  --observationCount_;
  auto new_end = std::remove_if(obsFeatures_.begin(), obsFeatures_.end(), [&feature](const std::weak_ptr<Feature> &wf) {
    auto sp = wf.lock();
    return sp && sp == feature;
  });

  bool found = (new_end != obsFeatures_.end());
  obsFeatures_.erase(new_end, obsFeatures_.end());
  return found;
}
} // namespace StereoSLAM