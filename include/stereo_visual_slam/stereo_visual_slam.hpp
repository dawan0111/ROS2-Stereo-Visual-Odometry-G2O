#ifndef __STEREO_VISUAL_SLAM_H__
#define __STEREO_VISUAL_SLAM_H__

#include "rclcpp/rclcpp.hpp"
#include "stereo_visual_slam/frame.hpp"
#include "stereo_visual_slam/frontend.hpp"
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_broadcaster.h>

namespace StereoSLAM {
class StereoVisualSLAM : public rclcpp::Node {
public:
  using Image = sensor_msgs::msg::Image;
  using ImageSyncPolicy = message_filters::sync_policies::ApproximateTime<Image, Image>;
  explicit StereoVisualSLAM(const rclcpp::NodeOptions &);

private:
  void ImageCallback(const Image::ConstSharedPtr &leftImage, const Image::ConstSharedPtr &rightImage);
  void debugImagePublish(const std::shared_ptr<Frame> &frame);

private:
  std::shared_ptr<message_filters::Subscriber<Image>> leftImageSub_;
  std::shared_ptr<message_filters::Subscriber<Image>> rightImageSub_;
  std::shared_ptr<message_filters::Synchronizer<ImageSyncPolicy>> syncStereo_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debugImagePub_;

  std::shared_ptr<Frontend> frontend_;
};
} // namespace StereoSLAM

#endif // __STEREO_VISUAL_SLAM_H__