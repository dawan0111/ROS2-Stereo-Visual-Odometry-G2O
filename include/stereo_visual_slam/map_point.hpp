#ifndef __MAP_POINT_H__
#define __MAP_POINT_H__

#include "stereo_visual_slam/feature.hpp"
#include <Eigen/Dense>
#include <iostream>
#include <memory>

namespace StereoSLAM {
class Feature;
class MapPoint {
public:
  using Ptr = std::shared_ptr<MapPoint>;
  MapPoint();
  MapPoint(Eigen::Vector3d &worldPoint);
  int16_t getObservationCount() const;
  bool addObserve(std::shared_ptr<Feature> feature);
  bool removeObserve(std::shared_ptr<Feature> feature);

  Eigen::Vector3d &getWorldPoint() { return worldPoint_; }
  void setWorldPoint(Eigen::Vector3d &point) { worldPoint_ = point; }

  std::vector<std::weak_ptr<Feature>> &getObserve() { return obsFeatures_; };

  const int32_t id;
  bool isLocalPoint = false;

private:
  Eigen::Vector3d worldPoint_;
  int16_t observationCount_;
  std::vector<std::weak_ptr<Feature>> obsFeatures_;
  static int32_t nextMapPointId;
};
} // namespace StereoSLAM

#endif // __MAP_POINT_H__