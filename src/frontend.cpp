#include "stereo_visual_slam/frontend.hpp"

namespace StereoSLAM {
Frontend::Frontend() { std::cout << "FrontEnd Constructor" << std::endl; }

bool Frontend::step(std::shared_ptr<Frame> frame) {
  std::cout << "Input Frame" << std::endl;
  currentFrame_ = frame;

  if (status == Status::INIT) {
    init();
  } else {
    tracking();
  }
  return true;
}

void Frontend::tracking() {}

void Frontend::init() {
  createLeftFeature();
  matchInRight();
  createMapPoint();

  std::cout << "Frontend: Init" << std::endl;
}

int16_t Frontend::createLeftFeature() {
  std::vector<cv::KeyPoint> keypoints;
  int8_t fast_threshold = 120;
  bool nonMaxSuppression = true;

  cv::FAST(currentFrame_->imageL, keypoints, fast_threshold, nonMaxSuppression);

  for (auto &keyPoint : keypoints) {
    currentFrame_->featurePtrs.push_back(Feature::Ptr(new Feature(currentFrame_, keyPoint.pt)));
  }

  std::cout << "Detected feature count: " << currentFrame_->featurePtrs.size() << std::endl;
  return currentFrame_->featurePtrs.size();
}

int16_t Frontend::matchInRight() {
  std::vector<cv::Point2f> leftKeypoints, rightKeypoints;

  for (auto &feature : currentFrame_->featurePtrs) {
    leftKeypoints.push_back(feature->point);
    rightKeypoints.push_back(feature->point);
  }

  std::vector<uchar> status;
  cv::Mat error;
  cv::calcOpticalFlowPyrLK(
      currentFrame_->imageL, currentFrame_->imageR, leftKeypoints, rightKeypoints, status, error, cv::Size(11, 11), 3,
      cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);

  int num_good_pts = 0;
  for (size_t i = 0; i < status.size(); ++i) {
    if (status[i]) {
      Feature::Ptr feat(new Feature(currentFrame_, rightKeypoints[i]));
      feat->isLeftFeature = false;
      currentFrame_->rightFeaturePtrs.push_back(feat);
      num_good_pts++;
    } else {
      currentFrame_->rightFeaturePtrs.push_back(nullptr);
    }
  }
  std::cout << "Find " << num_good_pts << " in the right image." << std::endl;
  return num_good_pts;
}

int16_t Frontend::trackingFeature() { return 0; }

void Frontend::createMapPoint() {}

void Frontend::estimatePose() {}
} // namespace StereoSLAM