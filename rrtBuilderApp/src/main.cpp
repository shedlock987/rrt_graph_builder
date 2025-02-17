#include "graph.h"
#include <iostream>

int main() {
rrt::coordinate_t origin(2.0F,2.0F);
rrt::coordinate_t pt1(4.0F, 5.0F);
rrt::coordinate_t pt2(8.0F, 10.0F);
rrt::Graph testGraph;
std::cout << "size: " << testGraph._adjacencyList.size() << "\n";
testGraph.addNode(&origin, 1.0F);
testGraph.printGraph();
std::cout << "size: " << testGraph._adjacencyList.size() << "\n";
testGraph.addNode(&pt1, 2.2F);
testGraph.printGraph();
std::cout << "size: " << testGraph._adjacencyList.size() << "\n";


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
