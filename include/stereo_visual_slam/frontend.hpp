#ifndef __FRONTEND_H__
#define __FRONTEND_H__

#include "stereo_visual_slam/frame.hpp"
#include "stereo_visual_slam/map.hpp"
#include "stereo_visual_slam/map_point.hpp"
#include "stereo_visual_slam/pinhole_camera.hpp"
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
namespace StereoSLAM {

enum class Status { INIT, TRACKING, LOSS };
class Frontend {
public:
  Frontend(std::shared_ptr<PinholeCamera> stereoCam, std::shared_ptr<Map> map);
  bool step(std::shared_ptr<Frame> frame);

private:
  void tracking();
  void init();
  int16_t createLeftFeature();
  int16_t matchInRight();
  int16_t trackingFeature();
  void createMapPoint();
  void estimatePose();

private:
  Status status = Status::INIT;
  std::shared_ptr<Frame> currentFrame_;
  std::shared_ptr<PinholeCamera> stereoCam_;
  std::shared_ptr<Map> map_;
};
} // namespace StereoSLAM

#endif // __FRONTEND_H__