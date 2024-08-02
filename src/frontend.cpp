#include "stereo_visual_slam/frontend.hpp"

namespace StereoSLAM {
Frontend::Frontend() { std::cout << "FrontEnd Constructor" << std::endl; }

bool Frontend::step(std::shared_ptr<Frame> frame) {
  std::cout << "Input Frame" << std::endl;
  return true;
}
} // namespace StereoSLAM