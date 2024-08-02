#include "stereo_visual_slam/map_point.hpp"
namespace StereoSLAM {
int32_t MapPoint::nextMapPointId = 0;
MapPoint::MapPoint() : id(++nextMapPointId), observationCount_(0) {}

MapPoint::MapPoint(Eigen::Vector3d &worldPoint) : id(++nextMapPointId), worldPoint_(worldPoint), observationCount_(0) {}

const int16_t &MapPoint::getObservationCount() const { return observationCount_; }

bool MapPoint::addObserve(std::shared_ptr<Feature> feature) {
  ++observationCount_;
  obsFeatures_.push_back(feature);
  return true;
}

bool MapPoint::removeObserve(std::shared_ptr<Feature> feature) {
  --observationCount_;
  return true;
}
} // namespace StereoSLAM