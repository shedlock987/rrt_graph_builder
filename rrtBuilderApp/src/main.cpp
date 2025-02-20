#include "graph.h"
#include <iostream>

int main() {
rrt::coordinate_t origin(2.0F,2.5F);
rrt::coordinate_t pt1(4.1F, 5.0F);
rrt::coordinate_t pt2(8.0F, 10.0F);
rrt::Graph testGraph(3.0F,3.0F);

testGraph.addNode(origin.x_,origin.y_, origin.time_, 1.0F);

testGraph.addNode(4.1, 5, 0, 2.2F);


testGraph.addNode(testGraph._adjacencyList.front(), 77.0F, 88.0F, 0.0F, 10.0F);


testGraph.printGraph();
//testGraph._adjacencyList.at(0)->printNode();


}
