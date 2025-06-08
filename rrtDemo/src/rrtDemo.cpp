#include "graph.h"
#include "rrt.h"
#include <iostream>
#include <boost/python.hpp>

int main() {
    rrt::Node::coordinate_t origin(0.0F,0.0F,0.0F);
    rrt::Node::coordinate_t lowerLeft(-5.0F, 0.0F, 0.0F);
    rrt::Node::coordinate_t upperRight(5.0F, 5.00, 0.0F);
    rrt::Graph testGraph(3.0F,3.0F);

    rrt::RRT testRRT(lowerLeft, upperRight, origin, upperRight, 0.8F, 1.0F, 0.5F, 2, true, 10000);
    
    testRRT.buildRRT();
    //testRRT.printGraph();
}
