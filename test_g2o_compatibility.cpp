#include <iostream>
#include "g2o/core/optimizable_graph.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/types/slam3d/types_slam3d.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"

//G2O_USE_TYPE_GROUP(slam3d);
//
//typedef g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1>> SlamBlockSolver;
//typedef g2o::LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

int32_t main (int32_t argc_, char** argv_) {

  //ds check test if dynamic testing is desired
  bool test_dynamic_allocation = false;
  if (argc_ == 2) {
    test_dynamic_allocation = true;
  }

  //ds allocate an optimizable graph
//  std::unique_ptr<SlamLinearSolver> linearSolver = g2o::make_unique<SlamLinearSolver>();
//  linearSolver->setBlockOrdering(true);
//  std::unique_ptr<SlamBlockSolver> blockSolver = g2o::make_unique<SlamBlockSolver>(std::move(linearSolver));
//  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(std::move(blockSolver));
//  SlamLinearSolver* linearSolver = new SlamLinearSolver();
//  linearSolver->setBlockOrdering(true);
//  SlamBlockSolver* blockSolver = new SlamBlockSolver(linearSolver);
//  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(blockSolver);

  if (test_dynamic_allocation) {
    std::cerr << "------------------------------------------------------------------------------------------ DYNAMIC" << std::endl;

    //ds allocate optimizer
    std::cerr << "allocating graph" << std::endl;
    g2o::SparseOptimizer* optimizer = new g2o::SparseOptimizer();
    std::cerr << "allocated" << std::endl;
//    std::cerr << "setting algorithm" << std::endl;
//    optimizer->setAlgorithm(solver);
//    std::cerr << "set" << std::endl;

    //ds clean pose graph
    optimizer->clear();
    optimizer->setVerbose(true);

    //ds set world parameter (required for landmark EdgeSE3PointXYZ measurements)
    std::cerr << "allocating parameter" << std::endl;
    g2o::ParameterSE3Offset* parameter_world_offset = new g2o::ParameterSE3Offset();
    std::cerr << "setting parameter ID" << std::endl;
    parameter_world_offset->setId(0);
//    std::cerr << "adding parameter to graph" << std::endl;
//    optimizer->addParameter(parameter_world_offset);
//    std::cerr << "done" << std::endl;
    std::cerr << "deallocating graph" << std::endl;
    delete optimizer;
    std::cerr << "deallocated" << std::endl;
    std::cerr << "leaving scope" << std::endl;
  } else {
    std::cerr << "------------------------------------------------------------------------------------------ STATIC" << std::endl;

    //ds allocate optimizer
    std::cerr << "allocating graph" << std::endl;
    g2o::SparseOptimizer optimizer;
    std::cerr << "allocated" << std::endl;
//    std::cerr << "setting algorithm" << std::endl;
//    optimizer.setAlgorithm(solver);
//    std::cerr << "set" << std::endl;

    //ds clean pose graph
    optimizer.clear();
    optimizer.setVerbose(true);

    //ds set world parameter (required for landmark EdgeSE3PointXYZ measurements)
    std::cerr << "allocating parameter" << std::endl;
    g2o::ParameterSE3Offset* parameter_world_offset = new g2o::ParameterSE3Offset();
    std::cerr << "setting parameter ID" << std::endl;
    parameter_world_offset->setId(0);
//    std::cerr << "adding parameter to graph" << std::endl;
//    optimizer.addParameter(parameter_world_offset);
//    std::cerr << "done" << std::endl;
    std::cerr << "leaving scope" << std::endl;
  }

  std::cerr << "terminating" << std::endl;
  return 0;
}
