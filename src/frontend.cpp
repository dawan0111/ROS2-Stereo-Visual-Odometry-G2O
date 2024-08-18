#include "stereo_visual_slam/frontend.hpp"

namespace StereoSLAM {
Frontend::Frontend(std::shared_ptr<PinholeCamera> stereoCam, std::shared_ptr<Map> map, std::shared_ptr<Backend> backend,
                   std::shared_ptr<fbow::Vocabulary> vocabulary)
    : stereoCam_(stereoCam), map_(map), backend_(backend), vocabulary_(vocabulary) {
  std::cout << "FrontEnd Constructor" << std::endl;
  gftt_ = cv::GFTTDetector::create(150, 0.01, 20);
  orb_ = cv::ORB::create(2000);
}

bool Frontend::step(std::shared_ptr<Frame> frame) {
  // std::cout << "Input Frame" << std::endl;
  currentFrame_ = frame;

  if (status_ == Status::INIT) {
    init();
  } else {
    tracking();
  }

  prevFrame_ = currentFrame_;
  return true;
}

void Frontend::tracking() {
  auto trackingFeatureCount = trackingFeature();
  auto inlierFeatureCount = estimatePose();
  // std::cout << "Tracking: " << trackingFeatureCount << "/ Inliers: " << inlierFeatureCount << std::endl;

  if (inlierFeatureCount <= 70) {
    std::cout << "Set Keyframe #" << currentFrame_->frameId << std::endl;

    updateObservation();

    currentFrame_->setKeyFrame();
    createFbow();
    map_->addKeyframe(currentFrame_);
    // backend_->updateMap();

    createLeftFeature();
    matchInRight();
    createMapPoint();
  }
}

void Frontend::init() {
  currentFrame_->T_wc = Sophus::SE3d();
  currentFrame_->T_d = Sophus::SE3d();
  createLeftFeature();
  matchInRight();
  createMapPoint();

  currentFrame_->setKeyFrame();
  createFbow();
  map_->addKeyframe(currentFrame_);

  std::cout << "Frontend: Init" << std::endl;
  status_ = Status::TRACKING;
}

int16_t Frontend::createLeftFeature() {
  cv::Mat mask(currentFrame_->imageL.size(), CV_8UC1, 255);
  for (auto &feat : currentFrame_->featurePtrs) {
    cv::rectangle(mask, feat->point.pt - cv::Point2f(10, 10), feat->point.pt + cv::Point2f(10, 10), 0, cv::FILLED);
  }

  std::vector<cv::KeyPoint> keypoints;
  gftt_->detect(currentFrame_->imageL, keypoints, mask);
  for (auto &kp : keypoints) {
    currentFrame_->featurePtrs.push_back(std::make_shared<Feature>(currentFrame_, kp));
  }
  return currentFrame_->featurePtrs.size();
}

int16_t Frontend::matchInRight() {
  std::vector<cv::Point2f> leftKeypoints, rightKeypoints;

  for (auto &feature : currentFrame_->featurePtrs) {
    leftKeypoints.push_back(feature->point.pt);
    rightKeypoints.push_back(feature->point.pt);
  }

  std::vector<uchar> status;
  cv::Mat error;
  double epsilon = 1.0;
  cv::calcOpticalFlowPyrLK(
      currentFrame_->imageL, currentFrame_->imageR, leftKeypoints, rightKeypoints, status, error, cv::Size(11, 11), 3,
      cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);

  int num_good_pts = 0;
  for (size_t i = 0; i < status.size(); ++i) {
    if (status[i]) {
      Feature::Ptr feat(new Feature(currentFrame_, cv::KeyPoint(rightKeypoints[i].x, rightKeypoints[i].y, 1.0)));
      feat->isLeftFeature = false;
      currentFrame_->rightFeaturePtrs.push_back(std::move(feat));
      num_good_pts++;
    } else {
      currentFrame_->rightFeaturePtrs.push_back(nullptr);
    }
  }
  std::cout << "Find " << num_good_pts << " in the right image." << std::endl;
  return num_good_pts;
}

int16_t Frontend::trackingFeature() {
  if (prevFrame_ == nullptr) {
    std::cout << "???" << std::endl;
    status_ = Status::INIT;
    return -1;
  }
  std::vector<cv::Point2f> prevKeypoints, currKeypoints;

  for (auto &feature : prevFrame_->featurePtrs) {
    prevKeypoints.push_back(feature->point.pt);
    currKeypoints.push_back(feature->point.pt);
  }
  std::vector<uchar> status;
  cv::Mat error;

  cv::calcOpticalFlowPyrLK(
      prevFrame_->imageL, currentFrame_->imageL, prevKeypoints, currKeypoints, status, error, cv::Size(11, 11), 3,
      cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);

  for (size_t i = 0; i < status.size(); ++i) {
    const auto &prevKeypoint = prevFrame_->featurePtrs[i];
    if (status[i] && !prevKeypoint->mapPointPtr.expired()) {
      Feature::Ptr feat(
          new Feature(currentFrame_, cv::KeyPoint(currKeypoints[i].x, currKeypoints[i].y, 1.0), prevKeypoints[i]));
      feat->mapPointPtr = prevKeypoint->mapPointPtr;

      currentFrame_->featurePtrs.push_back(std::move(feat));
    }
  }

  return currentFrame_->featurePtrs.size();
}

void Frontend::createMapPoint() {
  auto framePose = currentFrame_->T_wc.inverse();
  auto pose = stereoCam_->getPose().matrix();
  int16_t landmarkCount = 0;
  std::vector<cv::Point2f> leftPoints, rightPoints;
  std::vector<Feature::Ptr> features;

  for (int i = 0; i < currentFrame_->featurePtrs.size(); ++i) {
    if (currentFrame_->rightFeaturePtrs[i] != nullptr && currentFrame_->featurePtrs[i]->mapPointPtr.expired()) {
      auto &feature = currentFrame_->featurePtrs[i];
      auto &rightFeature = currentFrame_->rightFeaturePtrs[i];

      leftPoints.push_back(stereoCam_->pixel2camera(feature->point.pt));
      rightPoints.push_back(stereoCam_->pixel2camera(rightFeature->point.pt));
      features.push_back(feature);
    }
  }

  /* clang-format off */
  cv::Mat T1 = (cv::Mat_<float>(3, 4) << 
    1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0);
  cv::Mat T2 = (cv::Mat_<float>(3, 4) <<
    pose(0, 0), pose(0, 1), pose(0, 2), pose(0, 3),
    pose(1, 0), pose(1, 1), pose(1, 2), pose(1, 3),
    pose(2, 0), pose(2, 1), pose(2, 2), pose(2, 3));
  /* clang-format on */

  cv::Mat worldHomoPoints;

  cv::triangulatePoints(T1, T2, leftPoints, rightPoints, worldHomoPoints);

  for (int i = 0; i < worldHomoPoints.cols; ++i) {
    cv::Mat x = worldHomoPoints.col(i);
    Eigen::Vector4d homogenousWorldPoint = Eigen::Vector4d::Identity();
    Eigen::Vector3d worldPoint = Eigen::Vector3d::Zero();

    x /= x.at<float>(3, 0);

    homogenousWorldPoint(0) = x.at<float>(0, 0);
    homogenousWorldPoint(1) = x.at<float>(1, 0);
    homogenousWorldPoint(2) = x.at<float>(2, 0);
    homogenousWorldPoint(3) = 1.0;

    if (homogenousWorldPoint(2) > 0 && homogenousWorldPoint(2) <= 50) {
      homogenousWorldPoint = framePose * homogenousWorldPoint;

      worldPoint(0) = homogenousWorldPoint(0);
      worldPoint(1) = homogenousWorldPoint(1);
      worldPoint(2) = homogenousWorldPoint(2);

      auto mapPointPtr = std::make_shared<MapPoint>(worldPoint);
      mapPointPtr->isLocalPoint = true;
      mapPointPtr->addObserve(features[i]);

      features[i]->mapPointPtr = mapPointPtr;
      map_->addMapPoint(mapPointPtr);
      ++landmarkCount;
    }
  }

  // std::cout << "Find Landmark: " << landmarkCount << std::endl;
}

int16_t Frontend::estimatePose() {
  cv::Mat rVec;
  cv::Mat tVec;
  // cv::Mat rVec = (cv::Mat_<double>(3, 1) << 0.0, 0.0, 0.0);
  // cv::Mat tVec = (cv::Mat_<double>(3, 1) << prevFrame_->T_wc.translation().x(), prevFrame_->T_wc.translation().y(),
  //                 prevFrame_->T_wc.translation().z());
  std::vector<int> inliers;
  std::vector<Feature::Ptr> features;
  std::vector<cv::Point3f> worldPoints;
  std::vector<cv::Point2f> pixelPoints;

  for (auto &feature : currentFrame_->featurePtrs) {
    if (feature->mapPointPtr.lock() != nullptr) {
      auto mapPoint = feature->mapPointPtr.lock();
      auto &worldPoint = mapPoint->getWorldPoint();

      features.push_back(feature);
      pixelPoints.push_back(feature->point.pt);
      worldPoints.push_back(cv::Point3f(worldPoint(0), worldPoint(1), worldPoint(2)));

      feature->isInlier = false;
    }
  }

  auto K = stereoCam_->getCVIntrinsic();
  auto distCoeffs = stereoCam_->getCVDistCoeff();
  bool success = cv::solvePnPRansac(worldPoints, pixelPoints, K, distCoeffs, rVec, tVec, false, 100, 2.0, 0.99, inliers,
                                    cv::SOLVEPNP_ITERATIVE);
  if (success) {
    cv::Mat R;
    Eigen::Matrix3d eigenR;
    Eigen::Vector3d eigenT;
    cv::Rodrigues(rVec, R);

    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        eigenR(i, j) = R.at<double>(i, j);
      }
    }

    eigenT(0) = tVec.at<double>(0, 0);
    eigenT(1) = tVec.at<double>(1, 0);
    eigenT(2) = tVec.at<double>(2, 0);

    for (auto &inlier : inliers) {
      features[inlier]->isInlier = true;
    }

    for (auto &feature : features) {
      if (!feature->isInlier) {
        feature->mapPointPtr.reset();
        feature->isInlier = true;
      }
    }

    currentFrame_->T_wc = Sophus::SE3d(eigenR, eigenT);
  } else {
    std::cout << "pose estimate failed!!" << std::endl;
  }

  return inliers.size();
}

void Frontend::updateObservation() {
  for (auto &feature : currentFrame_->featurePtrs) {
    if (!feature->mapPointPtr.expired()) {
      auto mapPointPtr = feature->mapPointPtr.lock();
      mapPointPtr->addObserve(feature);
    }
  }
}

void Frontend::createFbow() {
  std::vector<cv::KeyPoint> keyPoints;
  cv::Mat descriptors;

  orb_->detectAndCompute(currentFrame_->imageL, cv::Mat(), keyPoints, descriptors);

  currentFrame_->fBowFeature = vocabulary_->transform(descriptors);
  currentFrame_->briefDesc = descriptors;
  // std::cout << "Create descriptor) cols: " << descriptors.cols << ", rows: " << descriptors.rows << std::endl;
}
} // namespace StereoSLAM