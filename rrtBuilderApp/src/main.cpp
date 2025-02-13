#include "graph.h"
#include <iostream>

int main() {
rrt::coordinate_t origin(2,2.0F,2.0F, 2.0F);
rrt::Graph testGraph;
std::cout << testGraph._linkedList.size() << "\n";
testGraph.addNodes(origin, 1.0F);
std::cout << testGraph._linkedList.size() << "\n";
testGraph.addNodes(origin, 2.0F);
std::cout << testGraph._linkedList.size() << "\n";
std::cout << "\n I'm Alive !!!!! \n";
}
