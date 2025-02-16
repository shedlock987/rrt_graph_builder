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

    class coordinate_t
    {
        public:
            double x_;
            double y_;
            double time_;
            coordinate_t();
            coordinate_t(double _x, double _y);
            coordinate_t(double _x, double _y, double _time);
            int getDimension();
        private:
            int dim_;


    };

    class Node
        {
            private:
                double back_edge_weight_;
                int dimension_;
            public: 
   
                coordinate_t crdnts_;  //x,y,z,t       
                std::vector<Node *> fwd_node_;
                Node * back_node_;
                Node();
                Node(coordinate_t * _crdnts);
                Node(coordinate_t * _crdnts, double _back_edge_weight);
                void addFwdNode(Node * _cnnctn);
        };

class Graph {
    private:

    
    public:
        std::vector<Node *> _linkedList; //move this to private after debug

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

        void addNode(coordinate_t * _crdnts, double _back_edge_weight);
        void addNode(Node * _link, coordinate_t * _crdnts, double _back_edge_weight);
        coordinate_t getCoordinate(Node * _handle);

        //Node findNearest();
    
        // Function to add an edge to the graph
        // Parameters: src - source vertex
        // dest - destination vertex
        void addEdge(int _src, int _dest);
    
        // Function to print the adjacency list of the graph
        void printGraph();
    };
}

#endif /* Graph_H_ */