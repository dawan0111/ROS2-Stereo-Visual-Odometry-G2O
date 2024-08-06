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
  // createMapPoint();

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

int16_t Frontend::trackingFeature() {

  std::vector<cv::Point2f> leftPoints, rightPoints;

  for (int i = 0; i < currentFrame_->featurePtrs.size(); ++i) {
    if (currentFrame_->rightFeaturePtrs[i] != nullptr) {
      auto &leftFeature = currentFrame_->featurePtrs[i];
      auto &rightFeature = currentFrame_->rightFeaturePtrs[i];
      leftPoints.push_back(camera_->pixel2camera(leftFeature->point));
      rightPoints.push_back(camera_->pixel2camera(rightFeature->point));
    }
  }

  /* clang-format off */
  cv::Mat T1 = (cv::Mat_<float>(3, 4) << 
    1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0);
  cv::Mat T2 = (cv::Mat_<float>(3, 4) <<
    1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0);
  /* clang-format on */

  cv::Mat worldHomoPoints;

  cv::triangulatePoints(T2, T1, leftPoints, rightPoints, worldHomoPoints);

  for (int i = 0; i < worldHomoPoints.cols; ++i) {
    if (currentFrame_->featurePtrs[i]->mapPointPtr.lock() != nullptr) {
      continue;
    }

    cv::Mat x = worldHomoPoints.col(i);
    Eigen::Vector4d homogenousWorldPoint = Eigen::Vector4d::Identity();
    Eigen::Vector3d worldPoint = Eigen::Vector3d::Zero();

    x /= x.at<float>(3, 0);

    homogenousWorldPoint(0) = x.at<float>(0, 0);
    homogenousWorldPoint(1) = x.at<float>(1, 0);
    homogenousWorldPoint(2) = x.at<float>(2, 0);
    homogenousWorldPoint(3) = 1.0;

    if (homogenousWorldPoint(2) > 0 && homogenousWorldPoint(2) < 100) {
      // homogenousWorldPoint = worldPose * homogenousWorldPoint;

      worldPoint(0) = homogenousWorldPoint(0);
      worldPoint(1) = homogenousWorldPoint(1);
      worldPoint(2) = homogenousWorldPoint(2);

      auto mapPointPtr = std::make_shared<MapPoint>(worldPoint);
      mapPointPtr->addObserve(currentFrame_->featurePtrs[i]);

      currentFrame_->featurePtrs[i]->mapPointPtr = mapPointPtr;
      map_->addMapPoint(mapPointPtr);
    }
  }
}

void Frontend::createMapPoint() {}

void Frontend::estimatePose() {}
} // namespace StereoSLAM