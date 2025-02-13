/**
 * MIT License
 *
 * Copyright (c) 2024 Ryan Shedlock
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * @file graph.cpp
 * @brief Linked list to be used in an RRT Graph
 * @author Ryan Shedlock <rmshedlock@gmail.com>
 * @version 1.0
 */

#include "graph.h"
#define WARN 0

namespace rrt
{
    coordinate_t::coordinate_t() : dim_(3), x_(0.0F), y_(0.0F), z_(0.0F)
    {
    }
    coordinate_t::coordinate_t(double _x, double _y) : dim_(2), x_(_x), y_(_y), z_(0.0F)
    {
    }
    coordinate_t::coordinate_t(double _x, double _y, double _z) : dim_(3), x_(_x), y_(_y), z_(0.0F)
    {
    }

    int coordinate_t::getDimension()
    {
        return this->dim_;
    }
    
    Node::Node(coordinate_t * _crdnts) : crdnts_(_crdnts)
    {
    }

    Graph::Graph()
    {
        _linkedList.resize(1);

        //_linkedList.emplace_back({0,0,0,0},0,nullptr);

    }

    void Graph::addNodes(coordinate_t * _crdnts, double _back_edge_weight)
    {
        // Check if this is a new graph
        if(_linkedList.size() < 1)
        {
            _linkedList.resize(1);
            #ifdef WARN
                std::cout << "Initializing new RRT graph \n";
            #endif
        }
        else
        {
            _linkedList.resize(_linkedList.size() + 1);
        }

    }
/* HERE
    Node Graph::findNearest()
    {
        coordinate_t _point(2,0,0,0);

        return _linkedList[0]->back_node_;
    }
*/    
    Graph::~Graph(){}

    // Function to add an edge to the graph
    // Parameters: src - source vertex
    // dest - destination vertex
    void Graph::addEdge(int _src, int _dest)
    {

        // Add the destination to the adjacency list of the
        // source
       // _adjList[_src].push_back(_dest);


    }

    // Function to print the adjacency list of the graph
    void Graph::printGraph()
    {
/*
        // Iterate through each vertex
        for (int i = 0; i < _adjList.size(); ++i) {
            // Print the vertex
            std::cout << i << ": ";
            // Iterate through the adjacency list of the
            // vertex
            for (int j = 0; j < _adjList[i].size(); ++j) {
                // Print each adjacent vertex
                std::cout << _adjList[i][j] << " -> ";
            }
            // Indicate the end of the adjacency list
            std::cout << "NULL" << std::endl;
        }
            */
    }

}