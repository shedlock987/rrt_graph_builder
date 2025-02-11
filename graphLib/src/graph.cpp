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
#define LINKED_LIST 0

namespace rrt
{
    Node::Node(std::vector<double> _crdnts, double _back_edge_weight, Node * _back_node)
    {
        //Check coordinate size
        if(_crdnts.size() > 4)
        {
            this->dimension_ = 4;
            #ifdef WARN
                std::cout << "Dimension too large, saturating at size=4 \n";
            #endif
        }

        //Set coordinates
        for(auto i=0; i<this->dimension_; i++)
        {
            crdnts_.push_back(_crdnts[i]); 
        }

        //Assign Edge Weight
        if(_back_edge_weight < 0)
        {
            #ifdef WARN
                std::cout << "Edge Weight cannot be negative, defaulting to zero \n";
            #endif
            this->back_edge_weight_ = 0;
        }
        else{
            this->back_edge_weight_ = _back_edge_weight;
        }

        //Back-Link the linked list
        if( _back_node == nullptr)
        {
            #ifdef WARN
                std::cout << "Invalid Entry, Linked List backwards pointer is NULL \n";
            #endif
        }
        this->back_node_ = _back_node;

        
    }

    Graph::Graph()
    {

        #ifndef LINKED_LIST
            _adjList.resize(1);
        #endif

        #ifdef LINKED_LIST
            _linkedList.resize(1);

            //_linkedList.emplace_back({0,0,0,0},0,nullptr);
        #endif

    }

    void Graph::addNodes(int _cnt)
    {
        #ifndef LINKED_LIST
            if (_cnt > 0)
            {
                size_ = size_ + _cnt;
            }
            _adjList.resize(size_);
        #endif

        #ifdef LINKED_LIST

        #endif
    }
    
    Graph::~Graph(){}

    // Function to add an edge to the graph
    // Parameters: src - source vertex
    // dest - destination vertex
    void Graph::addEdge(int _src, int _dest)
    {
        #ifndef LINKED_LIST
            // Add the destination to the adjacency list of the
            // source
            _adjList[_src].push_back(_dest);
        #endif

        #ifdef LINKED_LIST

        #endif
    }

    // Function to print the adjacency list of the graph
    void Graph::printGraph()
    {
        #ifndef LINKED_LIST
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
        #endif

        #ifdef LINKED_LIST

        #endif
    }
}