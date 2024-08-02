#include "stereo_visual_slam/stereo_visual_slam.hpp"

namespace StereoSLAM {
StereoVisualSLAM::StereoVisualSLAM(const rclcpp::NodeOptions &options) : Node("stereo_visual_slam", options) {
  frontend_ = std::make_shared<Frontend>();
  leftImageSub_ = std::make_shared<message_filters::Subscriber<Image>>(this, "/stereo/image_left");
  rightImageSub_ = std::make_shared<message_filters::Subscriber<Image>>(this, "/stereo/image_right");
  syncStereo_ = std::make_shared<message_filters::Synchronizer<ImageSyncPolicy>>(ImageSyncPolicy(10), *leftImageSub_,
                                                                                 *rightImageSub_);
  syncStereo_->registerCallback(
      std::bind(&StereoVisualSLAM::ImageCallback, this, std::placeholders::_1, std::placeholders::_2));
}

void StereoVisualSLAM::ImageCallback(const Image::ConstSharedPtr &leftImage, const Image::ConstSharedPtr &rightImage) {
  auto frame = std::make_shared<Frame>();

  auto CVImageL = cv_bridge::toCvCopy(leftImage, leftImage->encoding)->image;
  auto CVImageR = cv_bridge::toCvCopy(rightImage, rightImage->encoding)->image;

  cv::cvtColor(frame->imageL, CVImageL, cv::COLOR_BGR2GRAY);
  cv::cvtColor(frame->imageR, CVImageR, cv::COLOR_BGR2GRAY);

  frontend_->step(frame);
}
} // namespace StereoSLAM