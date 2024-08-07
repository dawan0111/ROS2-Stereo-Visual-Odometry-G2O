#ifndef __PINHOLE_CAMERA_H__
#define __PINHOLE_CAMERA_H__

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>
namespace StereoSLAM {
class PinholeCamera {
public:
  PinholeCamera(double fx, double fy, double cx, double cy, double baseline);
  Eigen::Vector3d pixel2camera(const Eigen::Vector2d &p_p, double depth = 1.0);
  cv::Point2f pixel2camera(const cv::Point2f &point);
  Eigen::Vector2d world2pixel(const Eigen::Vector3d &p_w,
                              const Sophus::SE3d &T_c_w);

  Eigen::Vector3d world2camera(const Eigen::Vector3d &p_w,
                               const Sophus::SE3d &T_c_w);

  Eigen::Vector2d camera2pixel(const Eigen::Vector3d &p_c);
  Eigen::Vector3d pixel2World(const cv::Point2f &pointL, const cv::Point2f &pointR);

private:
  double fx_;
  double fy_;
  double cx_;
  double cy_;
  double baseline_;
};
} // namespace StereoSLAM

#endif // __PINHOLE_CAMERA_H__