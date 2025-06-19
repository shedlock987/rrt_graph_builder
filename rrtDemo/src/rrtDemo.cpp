
#include "rrtDemo.h"
#include "graph.h"
#include "rrt.h"
#include <iostream>
#include <boost/python.hpp>

using namespace boost::python;

namespace rrt
{
    VisRRT::VisRRT() : rrt_pimpl_(new RRT(
        -5.0, 0.0, 5.0, 5.0,   // _range_a_x, _range_a_y, _range_b_x, _range_b_y
        0.0, 0.0, 5.0, 5.0,    // _origin_x, _origin_y, _dest_x, _dest_y
        0.8, 1.0, 0.5, 2.0,    // _max_angle_rad, _max_dist, _min_dist, _max_interval
        10.0,                  // _max_time
        true,                  // _dim
        10000                  // _node_limit
    ))
    {      
    }

    VisRRT::~VisRRT() = default;

    void VisRRT::buildRRT() {
        rrt_pimpl_->buildRRT();
    }

    void VisRRT::printGraph() {
        rrt_pimpl_->printGraph();
    }
}

BOOST_PYTHON_MODULE(display_RRT) {
    class_<rrt::VisRRT, boost::noncopyable>("RRT")
        .def("buildRRT", &rrt::VisRRT::buildRRT)
        .def("printGraph", &rrt::VisRRT::printGraph);
}

int main() {
    rrt::Node::coordinate_t origin(0.0F,0.0F,0.0F);
    rrt::Node::coordinate_t lowerLeft(-5.0F, 0.0F, 0.0F);
    rrt::Node::coordinate_t upperRight(5.0F, 5.00, 0.0F);
    rrt::Graph testGraph(3.0F,3.0F);

    rrt::RRT testRRT(lowerLeft, upperRight, origin, upperRight, 1.05F, 1.0F, 0.5F, 0.2F, 10.0F, false, 50);
    
    testRRT.buildRRT();

    //rrt::VisRRT displayRRT;
    
    //displayRRT.rrt_pimpl_->buildRRT();
    //testRRT.printGraph();

    return 0;
}
