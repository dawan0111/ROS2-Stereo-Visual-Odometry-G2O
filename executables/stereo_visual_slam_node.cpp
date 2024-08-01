#include "stereo_visual_slam/stereo_visual_slam.hpp"
#include <iostream>

int32_t main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  const rclcpp::NodeOptions options;
  auto basic_slam_node = std::make_shared<StereoSLAM::StereoVisualSLAM>(options);

  rclcpp::spin(basic_slam_node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}