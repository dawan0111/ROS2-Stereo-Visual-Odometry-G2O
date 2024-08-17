#include "stereo_visual_slam/backend.hpp"

namespace StereoSLAM {
Backend::Backend(std::shared_ptr<PinholeCamera> camera, std::shared_ptr<Map> map) : camera_(camera), map_(map) {
  optimizer_ = std::make_unique<g2o::SparseOptimizer>();
  solver_ =
      new g2o::OptimizationAlgorithmLevenberg(std::make_unique<BlockSolverType>(std::make_unique<LinearSolverType>()));
  optimizer_->setAlgorithm(solver_);
  optimizer_->setVerbose(true);

  auto cvCamK = camera_->getCVIntrinsic();

  double focal_length = cvCamK.at<double>(0, 0);
  Eigen::Vector2d principal_point(cvCamK.at<double>(0, 2), cvCamK.at<double>(1, 2));

  g2o::CameraParameters *cam_params = new g2o::CameraParameters(focal_length, principal_point, 0.);
  cam_params->setId(0);

  optimizer_->addParameter(cam_params);
}

void Backend::updateMap() {
  auto &activeKeyFrames = map_->getActiveKeyFrames();
  auto &activeMapPoints = map_->getActiveMapPoints();

  if (activeKeyFrames.size() > 2) {
    optimize(activeKeyFrames, activeMapPoints);
  }
}

void Backend::optimize(Map::KeyFrameType &activeKeyframes, Map::MapPointType &activeMapPoints) {
  std::unordered_map<int32_t, g2o::VertexSE3Expmap *> vertices;
  std::unordered_map<int32_t, g2o::VertexPointXYZ *> mapPointVertices;
  u_int32_t keyFrameSize = 0;
  u_int32_t edgeSize = 0;
  u_int16_t frameCount = 0;
  for (auto &[id, frame] : activeKeyframes) {
    auto pose = frame->T_wc;
    g2o::SE3Quat g2oPose(pose.rotationMatrix(), pose.translation());
    g2o::VertexSE3Expmap *vertex = new g2o::VertexSE3Expmap();
    vertex->setId(id);
    vertex->setEstimate(g2oPose);

    if (frameCount < 2) {
      vertex->setFixed(true);
    }

    optimizer_->addVertex(vertex);
    vertices.insert({id, vertex});
    keyFrameSize = std::max(keyFrameSize, id);
    ++frameCount;
  }

  for (auto &[id, mapPoint] : activeMapPoints) {
    int16_t mapPointId = keyFrameSize + id;
    g2o::VertexPointXYZ *vertex = new g2o::VertexPointXYZ();
    vertex->setId(mapPointId);
    vertex->setMarginalized(true);
    vertex->setEstimate(mapPoint->getWorldPoint());
    optimizer_->addVertex(vertex);
    mapPointVertices.insert({id, vertex});

    if (mapPoint->getObserve().size() >= 2) {
      for (auto &observe : mapPoint->getObserve()) {
        if (observe.lock() == nullptr) {
          continue;
        }
        auto feature = observe.lock();
        if (!feature->isInlier || feature->framePtr.lock() == nullptr) {
          continue;
        }

        auto keyFrame = feature->framePtr.lock();

        auto landmarkId = id;
        auto keyFrameId = keyFrame->frameId;

        if (mapPointVertices.find(landmarkId) != mapPointVertices.end() &&
            vertices.find(keyFrameId) != vertices.end()) {
          g2o::EdgeProjectXYZ2UV *edge = new g2o::EdgeProjectXYZ2UV();
          g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
          Eigen::Vector2d measure = Eigen::Vector2d::Zero();

          measure(0) = feature->point.pt.x;
          measure(1) = feature->point.pt.y;

          edge->setId(id);
          edge->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer_->vertex(mapPointId)));
          edge->setVertex(
              1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer_->vertices().find(keyFrameId)->second));
          edge->setMeasurement(measure);
          edge->setInformation(Eigen::Matrix2d::Identity());
          edge->setRobustKernel(rk);
          edge->setParameterId(0, 0);
          optimizer_->addEdge(edge);

          ++edgeSize;
        }
      }
    }
  }

  optimizer_->initializeOptimization();
  optimizer_->setVerbose(false);
  optimizer_->optimize(30);

  for (const auto &[id, vertex] : vertices) {
    const auto &poseEstimate = vertex->estimate();
    Eigen::Quaterniond q = poseEstimate.rotation();
    Eigen::Vector3d t = poseEstimate.translation();
    activeKeyframes[id]->T_wc = Sophus::SE3d(q, t);
    activeKeyframes[id]->needUpdate = true;
  }

  for (const auto &[id, mapPointVertex] : mapPointVertices) {
    const auto &posEstimate = mapPointVertex->estimate();
    Eigen::Vector3d pos = Eigen::Vector3d::Zero();
    pos(0) = posEstimate(0);
    pos(1) = posEstimate(1);
    pos(2) = posEstimate(2);

    activeMapPoints[id]->setWorldPoint(pos);
  }

  map_->setRequiredViewerUpdated(true);
  optimizer_->clear();
}

} // namespace StereoSLAM