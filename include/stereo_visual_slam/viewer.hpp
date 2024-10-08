#ifndef __VIEWER_H__
#define __VIEWER_H__

#include "stereo_visual_slam/frame.hpp"
#include "stereo_visual_slam/map.hpp"
#include "stereo_visual_slam/pinhole_camera.hpp"
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace StereoSLAM {
class Viewer {
public:
  Viewer(std::shared_ptr<Map> map, std::shared_ptr<PinholeCamera> stereoCam, rclcpp::Clock::SharedPtr clock,
         rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debugImagePub,
         rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointCloudPub,
         rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pathPub);
  void update();
  void debugImageUpdate(const std::shared_ptr<Frame> frame);

private:
  void mapPointUpdate();
  void pathUpdate();

private:
  std::shared_ptr<Map> map_;
  std::shared_ptr<PinholeCamera> stereoCam_;
  rclcpp::Clock::SharedPtr clock_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debugImagePub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointCloudPub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pathPub_;

  sensor_msgs::msg::PointCloud2 pointCloudMsg_;
};
} // namespace StereoSLAM

#endif // __VIEWER_H__