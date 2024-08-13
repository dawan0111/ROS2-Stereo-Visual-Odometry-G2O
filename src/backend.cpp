#include "stereo_visual_slam/backend.hpp"

namespace StereoSLAM {
BackEnd::BackEnd(std::shared_ptr<Map> map, std::shared_ptr<Config> config, std::shared_ptr<PinholeCamera> camera)
    : map_(map), config_(config), camera_(camera) {
  optimizer_ = std::make_unique<g2o::SparseOptimizer>();
  solver_ =
      new g2o::OptimizationAlgorithmLevenberg(std::make_unique<BlockSolverType>(std::make_unique<LinearSolverType>()));
  optimizer_->setAlgorithm(solver_);
  optimizer_->setVerbose(true);

  double focal_length = config->get<double>("focal_x");
  Eigen::Vector2d principal_point(config->get<double>("c_x"), config->get<double>("c_y"));

  std::cout << "focal_length: " << focal_length << std::endl;
  std::cout << "Point: " << principal_point << std::endl;

  g2o::CameraParameters *cam_params = new g2o::CameraParameters(focal_length, principal_point, 0.);
  cam_params->setId(0);

  optimizer_->addParameter(cam_params);
}

void BackEnd::updateMap() {
  auto &activeKeyFrames = map_->getActiveKeyFrames();
  auto &activeMapPoints = map_->getActiveMapPoints();

  optimize(activeKeyFrames, activeMapPoints);
}

void BackEnd::optimize(Map::KeyFrameType &activeKeyframes, Map::MapPointType &activeMapPoints) {
  std::unordered_map<int32_t, g2o::VertexSE3Expmap *> vertices;
  std::unordered_map<int32_t, g2o::VertexPointXYZ *> mapPointVertices;
  u_int32_t keyFrameSize = 0;
  for (auto &[id, frame] : activeKeyframes) {
    auto pose = frame->T_wc;
    g2o::SE3Quat g2oPose(pose.rotationMatrix(), pose.translation());
    g2o::VertexSE3Expmap *vertex = new g2o::VertexSE3Expmap();
    vertex->setId(id);
    vertex->setEstimate(g2oPose);
    optimizer_->addVertex(vertex);
    vertices.insert({id, vertex});
    keyFrameSize = std::max(keyFrameSize, id);
  }

  for (auto &[id, mapPoint] : activeMapPoints) {
    int16_t mapPointId = keyFrameSize + id;
    g2o::VertexPointXYZ *vertex = new g2o::VertexPointXYZ();
    vertex->setId(mapPointId);
    vertex->setMarginalized(true);
    vertex->setEstimate(mapPoint->getWorldPoint());
    optimizer_->addVertex(vertex);
    mapPointVertices.insert({id, vertex});

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

      if (mapPointVertices.find(landmarkId) != mapPointVertices.end() && vertices.find(keyFrameId) != vertices.end()) {
        g2o::EdgeProjectXYZ2UV *edge = new g2o::EdgeProjectXYZ2UV();
        g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
        Eigen::Vector2d measure = Eigen::Vector2d::Zero();

        measure(0) = feature->point.x;
        measure(1) = feature->point.y;

        edge->setId(id);
        edge->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer_->vertex(mapPointId)));
        edge->setVertex(1,
                        dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer_->vertices().find(keyFrameId)->second));
        edge->setMeasurement(measure);
        edge->setInformation(Eigen::Matrix2d::Identity());
        edge->setRobustKernel(rk);
        edge->setParameterId(0, 0);
        optimizer_->addEdge(edge);
      }
    }
  }

  optimizer_->initializeOptimization();
  optimizer_->setVerbose(false);
  optimizer_->optimize(50);

  std::cout << "Pose size: " << vertices.size() << std::endl;
  std::cout << "Landmark size: " << mapPointVertices.size() << std::endl;

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

  optimizer_->clear();
}

} // namespace StereoSLAM