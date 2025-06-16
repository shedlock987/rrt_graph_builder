
#include "rrtDemo.h"
#include "graph.h"
#include <iostream>
#include <boost/python.hpp>

using namespace boost::python;

namespace rrt
{
    VisRRT::VisRRT() : rrt_pimpl_()
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

    rrt::RRT testRRT(lowerLeft, upperRight, origin, upperRight, 0.8F, 1.0F, 0.5F, 2, true, 10000);
    
    testRRT.buildRRT();

    rrt::VisRRT displayRRT;
    
    //displayRRT.rrt_pimpl_->buildRRT();
    //testRRT.printGraph();

    return 0;
}
