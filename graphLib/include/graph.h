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
    typedef std::tuple<double, double, double> coordinate_t;
    class Node
        {
            private:
            Node* back_node_;
            
            public: 
            double back_edge_weight_;
            typedef std::tuple<double, double, double> coordinate_t;
            coordinate_t crdnts_;       
            std::vector<Node *> fwd_node_;
            
                /**
                 * @brief   Constructs a new Node/Vertex
                 */   
                Node();

                /**
                 * @brief   Constructs a new Node/Vertex
                 *
                 * @param[in]  _crdnts  Cordinates of the Node/Vertex
                 */                
                Node(coordinate_t _crdnts);

                /**
                 * @brief   Constructs a new Node/Vertex
                 *
                 * @param[in]  _crdnts Coordinates of the Node/Vertex
                 * * @param[in]  _back_edge_weight  The weight of the back-connected edge
                 */
                Node(coordinate_t _crdnts, double _back_edge_weight);

                /**
                 * @brief   Node/Vertex Copy Constructor
                 *
                 * @param[in]  _copy    Node/Vertex to be copied
                 * @param[in]  _back_edge_weight    The weight of the back-connected edge
                 */
                Node(const Node &_copy);

                /**
                 * @brief   Constructs a new 3D Node/Vertex
                 *
                 * @param[in]  _x   The x-axis cordinate of the Node/Vertex
                 * @param[in]  _y   The y-axis cordinate of the Node/Vertex
                 * @param[in]  _time    The time coordinate of the Node/Vertex (assumes 3D)
                 * @param[in]  _back_edge_weight    The weight of the back-connected edge
                 */
                Node(double _x, double _y, double _time, double _back_edge_weight);

                /**
                 * @brief   Constructs a new 2D Node/Vertex
                 *
                 * @param[in]  _x   The x-axis cordinate of the Node/Vertex
                 * @param[in]  _y   The y-axis cordinate of the Node/Vertex
                 * @param[in]  _back_edge_weight    The weight of the back-connected edge
                 */
                Node(double _x, double _y, double _back_edge_weight);

                /**
                 * @brief   Dynamically adds a connection to another Node/vertex in the forward direction
                 *
                 * @param[in]  _cnnctn  Pointer to the forward node we will connect to
                 */
                void addFwdNode(Node * _cnnctn);

                /**
                 * @brief Console prints Node/Vertex Parameters, used for debugging
                 *
                */
                void debugPrintNode();

                /**
                 * @brief   Returns the x-axis coordinate of the node/vertex 
                 *
                 * @return  x-axis coordinate
                 */
                double getX();

                /**
                 * @brief   Returns the x-axis coordinate of the node/vertex 
                 *
                 * @return  y-axis coordinate
                 */
                double getY();

                /**
                 * @brief   Returns the time/temporal component of the node/vertex 
                 *
                 * @return  timestamp
                 */
                double getTm();

                /**
                 * @brief   Sets/updates the coordinates for the Node/Vertex
                 */
                void setCord(double _x, double _y, double _tm);

                /**
                 * @brief   Gets the Pointer to the Backward-connected Node/Vertex
                 *
                 * @return  Pointer to Backward-connected Node/Vertex
                 */
                Node* getBackCnnctn();

                /**
                 * @brief  Sets/Updates the Pointer to the Backward-connected Node/Vertex
                 *
                 * @param[in]  _cnnctn  Pointer to the backward node we will connect to
                 */
                void setBackCnnctn(Node* _cnnctn);

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
        void addNode(Node::coordinate_t _point, double _back_edge_weight);
        void addNode(Node* _link, Node::coordinate_t _point, double _back_edge_weight);
        void deleteNode(Node* _handle);
        int getIndex(Node* _handle);
        void addEdge(Node* _src, Node* _dest);
    

    };
}

#endif /* Graph_H_ */