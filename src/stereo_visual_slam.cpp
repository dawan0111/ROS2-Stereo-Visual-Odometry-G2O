#include "stereo_visual_slam/stereo_visual_slam.hpp"

namespace StereoSLAM {
StereoVisualSLAM::StereoVisualSLAM(const rclcpp::NodeOptions &options) : Node("stereo_visual_slam", options) {
  leftImageSub_ = std::make_shared<message_filters::Subscriber<Image>>(this, "/stereo/image_left");
  rightImageSub_ = std::make_shared<message_filters::Subscriber<Image>>(this, "/stereo/image_right");
  syncStereo_ = std::make_shared<message_filters::Synchronizer<ImageSyncPolicy>>(ImageSyncPolicy(10), *leftImageSub_,
                                                                                 *rightImageSub_);
  syncStereo_->registerCallback(
      std::bind(&StereoVisualSLAM::ImageCallback, this, std::placeholders::_1, std::placeholders::_2));
}

void StereoVisualSLAM::ImageCallback(const Image::ConstSharedPtr &leftImage, const Image::ConstSharedPtr &rightImage) {
  std::cout << "Image callback!!" << std::endl;
}
} // namespace StereoSLAM