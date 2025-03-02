/**
 * MIT License
 *
 * Copyright (c) 2025 Ryan Shedlock
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
 * @brief Graph in the form of an Adjacency List
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
#include <tuple>

namespace rrt
{
    class Node
        {
            private:
                int dimension_;
            public: 
                double back_edge_weight_;
                typedef std::tuple<double, double, double> coordinate_t;
                coordinate_t crdnts_;       
                std::vector<Node *> fwd_node_;
                Node* back_node_;
                Node();
                Node(coordinate_t _crdnts);
                Node(coordinate_t _crdnts, double _back_edge_weight);
                Node(const Node &_copy);
                Node(double _x, double _y, double _time, double _back_edge_weight);
                Node(double _x, double _y, double _back_edge_weight);
                void addFwdNode(Node * _cnnctn);
                void debugPrintNode();

                friend class Graph;
        };

class Graph {
    private:
        bool initCmplt_ = false;
    public:
        Graph();
        Graph(double _xHead, double _yHead);
        ~Graph();
        Node::coordinate_t getCoordinate(Node* _handle);
        void printGraph();

        //protected:
        std::vector<Node *> adjacencyList_; 
        void addNode(double _x, double _y, double _time, double _back_edge_weight);
        void addNode(Node* _link, double _x, double _y, double _time, double _back_edge_weight);
        void deleteNode(Node* _handle);
        int getIndex(Node* _handle);
        void addEdge(Node* _src, Node* _dest);
    

    };
}

#endif /* Graph_H_ */