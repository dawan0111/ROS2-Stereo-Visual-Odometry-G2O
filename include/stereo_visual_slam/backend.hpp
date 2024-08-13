#ifndef __BACK_END_H__
#define __BACK_END_H__

#include "stereo_visual_slam/config.hpp"
#include "stereo_visual_slam/map.hpp"
#include "stereo_visual_slam/pinhole_camera.hpp"
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <iostream>
namespace StereoSLAM {
class BackEnd {
  using BlockSolverType = g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>>;
  using LinearSolverType = g2o::LinearSolverCSparse<BlockSolverType::PoseMatrixType>;

public:
  BackEnd(std::shared_ptr<Map> map, std::shared_ptr<Config> config, std::shared_ptr<PinholeCamera> camera);
  void updateMap();
  void optimize(Map::KeyFrameType &activeKeyframes, Map::MapPointType &activeMapPoints);

private:
  std::shared_ptr<Map> map_;
  std::shared_ptr<Config> config_;
  std::shared_ptr<PinholeCamera> camera_;

  std::unique_ptr<g2o::SparseOptimizer> optimizer_;
  g2o::OptimizationAlgorithmLevenberg *solver_;

  std::vector<g2o::VertexSE3Expmap *> poseVertexs_;
  std::vector<g2o::VertexPointXYZ *> pointVertexs_;
};
} // namespace StereoSLAM

#endif // __BACK_END_H__