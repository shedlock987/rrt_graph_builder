#include "graph.h"
#include <iostream>

int main() {
rrt::coordinate_t origin(2.0F,2.5F);
rrt::coordinate_t pt1(4.1F, 5.0F);
rrt::coordinate_t pt2(8.0F, 10.0F);
rrt::Graph testGraph(3.0F,3.0F);

rrt::Node* handle = testGraph._adjacencyList.front();

testGraph.addNode(origin.x_,origin.y_, origin.time_, 2.0F);
testGraph.addNode(handle, origin.x_,3.0F, origin.time_, 1.5F);

testGraph.deleteNode(handle);

testGraph.addNode(4.1, 5, 0, 2.2F);
testGraph.addNode(testGraph._adjacencyList.front(), 77.0F, 88.0F, 0.0F, 10.0F);
testGraph.addNode(testGraph._adjacencyList.at(3), 100.0F, 100.0F, 0.0F, 10.0F);
testGraph.addNode(testGraph._adjacencyList.at(3), 200.0F, 200.0F, 0.0F, 20.0F);
testGraph.addNode(testGraph._adjacencyList.at(3), 300.0F, 300.0F, 0.0F, 30.0F);

//testGraph.deleteNode(handle);


//testGraph.printGraph();



}
