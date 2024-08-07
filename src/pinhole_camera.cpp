#include "stereo_visual_slam/pinhole_camera.hpp"
namespace StereoSLAM {
PinholeCamera::PinholeCamera(double fx, double fy, double cx, double cy, double baseline)
    : fx_(fx), fy_(fy), cx_(cx), cy_(cy), baseline_(baseline) {}

Eigen::Vector3d PinholeCamera::pixel2camera(const Eigen::Vector2d &p_p, double depth) {
  Eigen::Vector3d pos = Eigen::Vector3d::Zero();
  auto x = (p_p(0, 0) - cx_) * depth / fx_;
  auto y = (p_p(1, 0) - cy_) * depth / fy_;

  pos(0) = x;
  pos(1) = y;
  pos(2) = depth;

  return pos;
}

cv::Point2f PinholeCamera::pixel2camera(const cv::Point2f &p_p) {
  cv::Point2f point = cv::Point2f(0.0, 0.0);
  auto x = (p_p.x - cx_) / fx_;
  auto y = (p_p.y - cy_) / fy_;

  point.x = x;
  point.y = y;

  return point;
}

Eigen::Vector2d PinholeCamera::world2pixel(const Eigen::Vector3d &p_w, const Sophus::SE3d &T_c_w) {
  return camera2pixel(world2camera(p_w, T_c_w));
}

Eigen::Vector3d PinholeCamera::world2camera(const Eigen::Vector3d &p_w, const Sophus::SE3d &T_c_w) {
  return T_c_w * p_w;
}

Eigen::Vector2d PinholeCamera::camera2pixel(const Eigen::Vector3d &p_c) {
  return Eigen::Vector2d(fx_ * p_c(0, 0) / p_c(2, 0) + cx_, fy_ * p_c(1, 0) / p_c(2, 0) + cy_);
}

Eigen::Vector3d PinholeCamera::pixel2World(const cv::Point2f &pointL, const cv::Point2f &pointR) {
  Eigen::Vector3d worldPoint = Eigen::Vector3d::Zero();
  double z = (baseline_ * fx_) / (pointL.x - pointR.x);
  double x = ((pointL.x - cx_) * z) / fx_;
  double y = ((pointL.y - cy_) * z) / fy_;

  worldPoint(0) = x;
  worldPoint(1) = y;
  worldPoint(2) = z;

  return worldPoint;
}
} // namespace StereoSLAM