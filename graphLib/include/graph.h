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
#include <algorithm>

namespace rrt
{

    class coordinate_t
    {
        public:
            double x_;
            double y_;
            double time_;
            coordinate_t();
            coordinate_t(const coordinate_t &_copy);
            coordinate_t(double _x, double _y);
            coordinate_t(double _x, double _y, double _time);
            int getDimension();
        private:
            int dim_;


    };

    class Node
        {
            private:
                int dimension_;
            public: 
                double back_edge_weight_;
                coordinate_t crdnts_;       
                std::vector<Node *> fwd_node_;
                Node* back_node_;
                Node();
                Node(coordinate_t * _crdnts);
                Node(coordinate_t * _crdnts, double _back_edge_weight);
                Node(const Node &_copy);
                Node(double _x, double _y, double _time, double _back_edge_weight);
                Node(double _x, double _y, double _back_edge_weight);
                coordinate_t getCoordinate();
                void addFwdNode(Node * _cnnctn);
                void debugPrintNode();

                friend class Graph;
        };

class Graph {
    private:
        bool _initCmplt = false;
    
    public:
        std::vector<Node *> _adjacencyList; //move this to private after debug
        Graph();
        Graph(double _xHead, double _yHead);
        ~Graph();

        void addNode(double _x, double _y, double _time, double _back_edge_weight);
        void addNode(Node* _link, double _x, double _y, double _time, double _back_edge_weight);
        void deleteNode(Node* _handle);
        int getIndex(Node* _handle);
        coordinate_t getCoordinate(Node* _handle);
        void addEdge(Node* _src, Node* _dest);
    
        // Function to print the adjacency list of the graph
        void printGraph();
    };
}

#endif /* Graph_H_ */