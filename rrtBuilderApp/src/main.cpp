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
//new rrt::Node connect();
//testGraph.addNode(testGraph._adjacencyList().front(), 66, 88, 0, 100F);

//testGraph.printGraph();



//testGraph.addNode(testGraph._adjacencyList.front(), &pt2, 3.3F);
//std::cout << "size: " << testGraph._adjacencyList.size() << "\n";

//rrt::Node test = &testGraph._adjacencyList[2];
//rrt::coordinate_t display = testGraph.getCoordinate(test);

////std::cout << "Coordnates of tail: x:" << display.x_ << "  y:" << display.y_ << std::endl;

//rrt::Node * test2 = testGraph._adjacencyList[2];
//auto display2 = testGraph.getCoordinate(test2);
//std::cout << "Coordnates of tail: x:" << display2.x_ << "  y:" << display2.y_ << std::endl;

//std::cout << "Coordnates of pt1: x:" << pt1.x_ << "  y:" << pt1.y_ << std::endl;

//std::cout << "\n I'm Alive !!!!! \n";
}
