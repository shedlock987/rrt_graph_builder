#include "rrtDemo.h"
#include <iostream>

int main(int argc, char** argv)
{
    // Use the default VisRRT constructor and run a simple build
    rrt::VisRRT vis;
    vis.buildRRT();
    std::cout << "RRT built. Nodes: " << vis.getNodeCount() << std::endl;
    return 0;
}