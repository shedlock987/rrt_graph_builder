#include "graph.h"
#include <iostream>

int main() {
rrt::Node::coordinate_t origin(2.0F,2.5F,0.0F);
rrt::Node::coordinate_t pt1(4.1F, 5.0F, 0.0F);
rrt::Node::coordinate_t pt2(8.0F, 10.0F, 0.0F);
rrt::Graph testGraph(3.0F,3.0F);

rrt::Node* handle = testGraph.adjacencyList_.front();
/*
testGraph.addNode(2.0F,2.5F, 0.0F, 2.0F);
testGraph.addNode(handle, 2.0F,3.0F, 0.0F, 1.5F);
testGraph.printGraph();
testGraph.deleteNode(handle);
testGraph.printGraph();
testGraph.addNode(4.1, 5, 0, 2.2F);
testGraph.addNode(testGraph.adjacencyList_.front(), 77.0F, 88.0F, 0.0F, 10.0F);
testGraph.addNode(testGraph.adjacencyList_.at(3), 100.0F, 100.0F, 0.0F, 10.0F);
testGraph.addNode(testGraph.adjacencyList_.at(3), 200.0F, 200.0F, 0.0F, 20.0F);
testGraph.addNode(testGraph.adjacencyList_.at(3), 300.0F, 300.0F, 0.0F, 30.0F);
*/
//testGraph.deleteNode(handle);


//testGraph.printGraph();



}
