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
 * @file graph.h
 * @brief Linked list to be used in an RRT Graph
 * @author Ryan Shedlock <rmshedlock@gmail.com>
 * @version 1.0
 */
#ifndef Graph_H_
#define Graph_H_

#include <stddef.h>
#include <string.h>
#include <cstdint>
#include <iostream>
#include <vector>

namespace rrt
{
    /**
     * @brief      Structure defining a node in the linked list
     *
     */
    class Node
        {
            private:
                double back_edge_weight_;
                int dimension_;
                //x,y,z,t
                std::vector<double> crdnts_; 
            public:                
                std::vector<Node *> fwd_node_;
                Node * back_node_;

                Node(std::vector<double> _crdnts, double _back_edge_weight, Node * _back_node);
        };

class Graph {
    private:
        // Adjacency list to represent the graph
        std::vector<std::vector<int> > _adjList;
        int size_;

        // Linked List to represent the graph
        std::vector<Node *> _linkedList;
    
    public:
        // Constructor to initialize the graph
        // Parameters: vertices - number of vertices in the
        // graph
        //  directed - flag to indicate if the graph is directed
        //  (default is false)
        Graph();

        /**
         * @brief      Destroys the object.
         */
        ~Graph();

        void addNodes(int _cnt);
    
        // Function to add an edge to the graph
        // Parameters: src - source vertex
        // dest - destination vertex
        void addEdge(int _src, int _dest);
    
        // Function to print the adjacency list of the graph
        void printGraph();
    };
}

#endif /* Graph_H_ */