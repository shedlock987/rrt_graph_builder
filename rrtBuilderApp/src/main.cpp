#include "graph.h"
#include <iostream>

int main() {
rrt::coordinate_t origin(2.0F,2.5F);
rrt::coordinate_t pt1(4.1F, 5.0F);
rrt::coordinate_t pt2(8.0F, 10.0F);
rrt::Graph testGraph;
testGraph.addNode(origin.x_,origin.y_, origin.time_, 1.0F);
std::cout   << "MAIN   size:" << testGraph._adjacencyList.size() << "    "
            << testGraph._adjacencyList.back()->crdnts_.x_ << " "
            << testGraph._adjacencyList.back()->crdnts_.y_ << " "
            << testGraph._adjacencyList.back()->crdnts_.time_ << std::endl; 

testGraph.addNode(4.1, 5, 0, 2.2F);
std::cout   << "MAIN   size:" << testGraph._adjacencyList.size() << "    "
            << testGraph._adjacencyList.at(2)->crdnts_.x_ << " "
            << testGraph._adjacencyList.at(2)->crdnts_.y_ << " "
            << testGraph._adjacencyList.at(2)->crdnts_.time_ << std::endl; 

testGraph.addNode(testGraph._adjacencyList.back(), 77.0F, 88.0F, 0.0F, 10.0F);
std::cout   << "MAIN   size:" << testGraph._adjacencyList.size() << "    "
            << testGraph._adjacencyList.at(3)->crdnts_.x_ << " "
            << testGraph._adjacencyList.at(3)->crdnts_.y_ << " "
            << testGraph._adjacencyList.at(3)->crdnts_.time_ << std::endl; 

testGraph.printGraph();

}
