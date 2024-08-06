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

  debugImagePub_ = this->create_publisher<sensor_msgs::msg::Image>("/mono/image", 50);
}

void StereoVisualSLAM::ImageCallback(const Image::ConstSharedPtr &leftImage, const Image::ConstSharedPtr &rightImage) {
  auto frame = std::make_shared<Frame>();

  auto CVImageL = cv_bridge::toCvCopy(leftImage, leftImage->encoding)->image;
  auto CVImageR = cv_bridge::toCvCopy(rightImage, rightImage->encoding)->image;

  cv::cvtColor(CVImageL, frame->imageL, cv::COLOR_BGR2GRAY);
  cv::cvtColor(CVImageR, frame->imageR, cv::COLOR_BGR2GRAY);

  frontend_->step(frame);
  debugImagePublish(frame);
}

void StereoVisualSLAM::debugImagePublish(const std::shared_ptr<Frame> &frame) {
  cv::Mat debugImage;
  auto leftImageCol = frame->imageL.cols;

  cv::hconcat(frame->imageL, frame->imageR, debugImage);
  cv::cvtColor(debugImage, debugImage, cv::COLOR_GRAY2BGR);

  for (int i = 0; i < frame->featurePtrs.size(); ++i) {
    auto &keyPoint = frame->featurePtrs[i];
    if (!keyPoint->isInlier) {
      cv::circle(debugImage, keyPoint->point, 4, cv::Scalar(0, 0, 255), 1, cv::LINE_4, 0);
    } else {
      cv::circle(debugImage, keyPoint->point, 4, cv::Scalar(0, 255, 0), 1, cv::LINE_4, 0);
    }

    if (frame->rightFeaturePtrs[i] != nullptr) {
      auto rightPoint = frame->rightFeaturePtrs[i]->point;
      auto rightX = rightPoint.x + leftImageCol;
      auto rightY = rightPoint.y;
      cv::circle(debugImage, cv::Point2f(rightX, rightY), 4, cv::Scalar(0, 255, 0), 1, cv::LINE_4, 0);
      cv::line(debugImage, keyPoint->point, cv::Point2f(rightX, rightY), cv::Scalar(0, 255, 0), 1);
    }

    // if (keyPoint->mapPointPtr.lock() != nullptr && keyPoint->framePtr.lock() != nullptr) {
    //   auto mapPoint = keyPoint->mapPointPtr.lock();
    //   auto frame = keyPoint->framePtr.lock();
    //   auto uv = pinholeCamera_->world2pixel(mapPoint->getWorldPoint(), frame->T_wc);
    //   auto uvPoint = cv::Point2f(uv(0), uv(1));

    //   cv::circle(debugImage, uvPoint, 4, cv::Scalar(255, 0, 255), 1, cv::LINE_4, 0);
    //   cv::line(debugImage, uvPoint, keyPoint->point, cv::Scalar(0, 255, 255), 1);
    // }
  }

  auto message = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", debugImage).toImageMsg();
  message->header.stamp = this->get_clock()->now();
  debugImagePub_->publish(*message);
}
} // namespace StereoSLAM