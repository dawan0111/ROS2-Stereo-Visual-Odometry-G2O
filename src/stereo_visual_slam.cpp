#include "stereo_visual_slam/stereo_visual_slam.hpp"

namespace StereoSLAM {
StereoVisualSLAM::StereoVisualSLAM(const rclcpp::NodeOptions &options) : Node("stereo_visual_slam", options) {
  double focal = 718.8560;
  double baseline = 0.537;
  cv::Point2d pp(607.1928, 185.2157);

  Eigen::Matrix3d K;
  Eigen::Vector3d t;

  /* clang-format off */
  K << focal, 0.0, pp.x,
       0.0, focal, pp.y,
       0.0, 0.0, 1.0;

  t << -386.1448, 0.0, 0.0;
  /* clang-format on */

  t = K.inverse() * t;
  Sophus::SE3d pose = Sophus::SE3d(Eigen::Matrix3d::Identity(), t);

  debugImagePub_ = this->create_publisher<sensor_msgs::msg::Image>("/mono/image", 50);
  pointCloudPub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/mono/pointcloud", 10);
  pathPub_ = this->create_publisher<nav_msgs::msg::Path>("/mono/path", 10);

  map_ = std::make_shared<Map>();
  stereoCam_ = std::make_shared<PinholeCamera>(focal, focal, pp.x, pp.y, baseline, pose);
  backend_ = std::make_shared<Backend>(stereoCam_, map_);
  frontend_ = std::make_shared<Frontend>(stereoCam_, map_, backend_);

  viewer_ = std::make_unique<Viewer>(map_, stereoCam_, this->get_clock(), debugImagePub_, pointCloudPub_, pathPub_);

  leftImageSub_ = std::make_shared<message_filters::Subscriber<Image>>(this, "/stereo/image_left");
  rightImageSub_ = std::make_shared<message_filters::Subscriber<Image>>(this, "/stereo/image_right");
  syncStereo_ = std::make_shared<message_filters::Synchronizer<ImageSyncPolicy>>(ImageSyncPolicy(10), *leftImageSub_,
                                                                                 *rightImageSub_);
  syncStereo_->registerCallback(
      std::bind(&StereoVisualSLAM::ImageCallback, this, std::placeholders::_1, std::placeholders::_2));

  double frequency = 30.0;
  auto interval = std::chrono::duration<double>(1.0 / frequency);
  timer_ = this->create_wall_timer(interval, [this]() -> void { viewer_->update(); });
}

void StereoVisualSLAM::ImageCallback(const Image::ConstSharedPtr &leftImage, const Image::ConstSharedPtr &rightImage) {
  auto frame = std::make_shared<Frame>();

  auto CVImageL = cv_bridge::toCvCopy(leftImage, leftImage->encoding)->image;
  auto CVImageR = cv_bridge::toCvCopy(rightImage, rightImage->encoding)->image;

  cv::cvtColor(CVImageL, frame->imageL, cv::COLOR_BGR2GRAY);
  cv::cvtColor(CVImageR, frame->imageR, cv::COLOR_BGR2GRAY);

  frontend_->step(frame);
  viewer_->debugImageUpdate(frame);
}
} // namespace StereoSLAM