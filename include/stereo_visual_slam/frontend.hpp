#ifndef __FRONTEND_H__
#define __FRONTEND_H__

#include "stereo_visual_slam/backend.hpp"
#include "stereo_visual_slam/frame.hpp"
#include "stereo_visual_slam/map.hpp"
#include "stereo_visual_slam/map_point.hpp"
#include "stereo_visual_slam/pinhole_camera.hpp"
#include <Eigen/Dense>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>
namespace StereoSLAM {

enum class Status { INIT, TRACKING, LOSS };
class Frontend {
public:
  Frontend(std::shared_ptr<PinholeCamera> stereoCam, std::shared_ptr<Map> map, std::shared_ptr<Backend> backend);
  bool step(std::shared_ptr<Frame> frame);

private:
  void tracking();
  void init();
  int16_t createLeftFeature();
  int16_t matchInRight();
  int16_t trackingFeature();
  void createMapPoint();
  void updateObservation();
  // void createFbow();
  int16_t estimatePose();

private:
  Status status_ = Status::INIT;
  std::shared_ptr<Frame> currentFrame_ = nullptr;
  std::shared_ptr<Frame> prevFrame_ = nullptr;
  std::shared_ptr<PinholeCamera> stereoCam_;
  std::shared_ptr<Map> map_;
  std::shared_ptr<Backend> backend_;
  // std::shared_ptr<fbow::Vocabulary> vocabulary_;

  cv::Ptr<cv::GFTTDetector> gftt_;
  cv::Ptr<cv::ORB> orb_;
};
} // namespace StereoSLAM

#endif // __FRONTEND_H__