#include "graph.h"
#include <iostream>

int main() {
rrt::Graph testGraph(2);
testGraph.addEdge(0,1);
testGraph.addNodes(2);
testGraph.addEdge(1,2);
testGraph.addEdge(1,3);
testGraph.addEdge(3,0);
testGraph.printGraph();
std::cout << "\n I'm Alive !!!!! \n";
}
