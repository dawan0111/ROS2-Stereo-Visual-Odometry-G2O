#include "stereo_visual_slam/frame.hpp"
namespace StereoSLAM {
int32_t Frame::nextFrameId = 0;

Frame::Frame() : frameId(++Frame::nextFrameId) {}
// Frame::Frame(int32_t frameId_) : frameId(frameId_) {}

void Frame::setKeyFrame() { isKeyFrame = true; }
} // namespace StereoSLAM