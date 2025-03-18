#include "graph.h"
#include "rrt.h"
#include <iostream>

int main() {
    rrt::Node::coordinate_t origin(0.0F,0.0F,0.0F);
    rrt::Node::coordinate_t lowerLeft(-5.0F, 0.0F, 0.0F);
    rrt::Node::coordinate_t upperRight(5.0F, 10.00, 0.0F);
    rrt::Graph testGraph(3.0F,3.0F);

    rrt::RRT testRRT(lowerLeft, upperRight, origin, upperRight, 0.8F, 0.25, 1, true, 50);

    testRRT.buildRRT();
    testRRT.printGraph();




}
