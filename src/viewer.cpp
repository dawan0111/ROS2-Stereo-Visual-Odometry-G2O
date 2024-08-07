#include "stereo_visual_slam/viewer.hpp"

namespace StereoSLAM {
Viewer::Viewer(std::shared_ptr<Map> map, std::shared_ptr<PinholeCamera> stereoCam, rclcpp::Clock::SharedPtr clock,
               rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debugImagePub,
               rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointCloudPub)
    : map_(map), stereoCam_(stereoCam), clock_(clock), debugImagePub_(debugImagePub), pointCloudPub_(pointCloudPub) {
  std::cout << "Viewer Constructor" << std::endl;
}

void Viewer::update() {
  const auto &mapPoints = map_->getActiveMapPoints();
  std::vector<Eigen::Vector3d> vector;

  for (const auto &[id, mapPoint] : mapPoints) {
    vector.push_back(mapPoint->getWorldPoint());
  }

  auto pointCloud = sensor_msgs::msg::PointCloud2();
  pointCloud.height = 1;
  pointCloud.width = mapPoints.size();
  pointCloud.is_dense = false;
  sensor_msgs::PointCloud2Modifier modifier(pointCloud);
  modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");

  sensor_msgs::PointCloud2Iterator<float> iter_x(pointCloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(pointCloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(pointCloud, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_rgb(pointCloud, "rgb");

  for (const auto &[id, mapPoint] : mapPoints) {
    auto vec = mapPoint->getWorldPoint();

    *iter_x = static_cast<float>(vec(0));
    *iter_y = static_cast<float>(vec(1));
    *iter_z = static_cast<float>(vec(2));

    uint32_t rgb = (static_cast<uint32_t>(255) << 16) | (static_cast<uint32_t>(255) << 8) |
                   static_cast<uint32_t>(255); // R=255, G=0, B=0

    if (mapPoint->isLocalPoint) {
      rgb = (static_cast<uint32_t>(255) << 16) | (static_cast<uint32_t>(0) << 8) |
            static_cast<uint32_t>(0); // R=255, G=0, B=0
    }
    std::memcpy(&(*iter_rgb), &rgb, sizeof(uint32_t));

    ++iter_x;
    ++iter_y;
    ++iter_z;
    ++iter_rgb;
  }

  pointCloud.header.frame_id = "camera_optical_link";
  pointCloud.header.stamp = clock_->now();
  pointCloudPub_->publish(pointCloud);
}

void Viewer::debugImageUpdate(const std::shared_ptr<Frame> frame) {
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
  message->header.stamp = clock_->now();
  debugImagePub_->publish(*message);
}
} // namespace StereoSLAM