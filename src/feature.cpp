#include "stereo_visual_slam/feature.hpp"

namespace StereoSLAM {

Feature::Feature(const std::shared_ptr<Frame> frame, const cv::Point2f &point) : framePtr(frame), point(point) {}

Feature::Feature(const std::shared_ptr<Frame> frame, const cv::Point2f &point, const cv::Point2f &prevPoint)
    : framePtr(frame), point(point), prevPoint(prevPoint) {}

} // namespace StereoSLAM