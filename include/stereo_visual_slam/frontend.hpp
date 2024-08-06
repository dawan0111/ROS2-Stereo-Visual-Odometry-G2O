#ifndef __FRONTEND_H__
#define __FRONTEND_H__

#include "stereo_visual_slam/frame.hpp"
namespace StereoSLAM {

enum class Status { INIT, TRACKING, LOSS };
class Frontend {
public:
  Frontend();
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
};
} // namespace StereoSLAM

#endif // __FRONTEND_H__