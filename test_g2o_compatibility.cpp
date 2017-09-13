#include <iostream>
#include "g2o/core/optimizable_graph.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/types/slam3d/types_slam3d.h"

//G2O_USE_TYPE_GROUP(slam3d);

int32_t main (int32_t argc_, char** argv_) {
  {
    std::cerr << "------------------------------------------------------------------------------------------ DYNAMIC" << std::endl;

    //ds allocate optimizer
    std::cerr << "allocating graph" << std::endl;
    g2o::SparseOptimizer* optimizer = new g2o::SparseOptimizer();
    std::cerr << "allocated" << std::endl;

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
  }

  {
    std::cerr << "------------------------------------------------------------------------------------------ STATIC" << std::endl;

    //ds allocate optimizer
    std::cerr << "allocating graph" << std::endl;
    g2o::SparseOptimizer optimizer;
    std::cerr << "allocated" << std::endl;

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
