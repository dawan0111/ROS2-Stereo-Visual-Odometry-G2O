#ifndef __FRONTEND_H__
#define __FRONTEND_H__

#include "stereo_visual_slam/frame.hpp"
namespace StereoSLAM {
class Frontend {
public:
  Frontend();
  bool step(std::shared_ptr<Frame> frame);
};
} // namespace StereoSLAM

#endif // __FRONTEND_H__