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
            public: 
            typedef std::tuple<double, double, double> coordinate_t; /**< custom type used for 2D eculidan coordinate with a time component */
              
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
                 * @param[in]  _time    The time/temporal component of the Node/Vertex (assumes 3D)
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
                 * @brief   Destroys the Node/Vertex
                 */
                ~Node();

                /**
                 * @brief   Dynamically adds a connection to another Node/vertex in the forward direction
                 *
                 * @param[in]  _cnnctn  Pointer to the forward node we will connect to
                 */
                void addFwdNode(Node * _cnnctn);

                /**
                 * @brief Console prints Node/Vertex Parameters, used for debugging
                */
                void debugPrintNode();

                /**
                 * @brief   Returns the x-axis coordinate of the node/vertex 
                 *
                 * @return  x-axis coordinate
                 */
                double xCrdnt();

                /**
                 * @brief   Returns the x-axis coordinate of the node/vertex 
                 *
                 * @return  y-axis coordinate
                 */
                double yCrdnt();

                /**
                 * @brief   Returns the time/temporal component of the node/vertex 
                 *
                 * @return  timestamp
                 */
                double time();

                /**
                 * @brief   Sets/updates the coordinates for the Node/Vertex
                 * 
                 * @param[in]  _x   The x-axis cordinate of the Node/Vertex
                 * @param[in]  _y   The y-axis cordinate of the Node/Vertex
                 * @param[in]  _tm  The time/temporal component of the Node/Vertex (assumes 3D)
                 */
                void setCrdnts(double _x, double _y, double _tm);

                /**
                 * @brief   Sets/updates the coordinates for the Node/Vertex
                 * 
                 * @param[in]  _crdnts   The desired Values for the coordinates
                 */
                void setCrdnts(coordinate_t _crdnts);

                /**
                 * @brief   Gets the coordinates for the Node/Vertex
                 * 
                 * @return  The Coordinates of type coordinate_t
                 */
                coordinate_t Crdnts();

                /**
                 * @brief   Gets the Pointer to the Backward-connected Node/Vertex aka Back-Edge
                 *
                 * @return  Pointer to Backward-connected Node/Vertex aka Back-Edge
                 */
                Node* BackCnnctn();

                /**
                 * @brief  Sets/Updates the Pointer to the Backward-connected Node/Vertex aka Back-Edge
                 *
                 * @param[in]  _cnnctn  Pointer to the backward node we will connect to
                 */
                void setBackCnnctn(Node* _cnnctn);

                /**
                 * @brief  Gets the weight of the Backward-connected Node/Vertex aka Back-Edge
                 *
                 * @return  Weight of the Node/Vertex's Back Connection
                 */
                double backEdgeWeight();

                /**
                 * @brief  Sets/Updates the weight of the Backward-connected Node/Vertex aka Back-Edge
                 *
                 * @param[in]  _back_edge_weight  The desired/new weight of the back-connected edge
                 */
                void setBackEdgeWeight(double _back_edge_weight);


                private:
                Node* back_node_; /**< Pointer to the backward connected Node/Vertex aka back-edge*/
                coordinate_t crdnts_; /**< Coordinates of the Node/Vertex */
                double back_edge_weight_; /**< Weight of the backward-connected Edge */
                
                friend class Graph;
        };

class Graph {
    private:
        bool initCmplt_ = false; /**< Flag to indicate the Graph is initialized */
    public:

        /**
        * @brief    Constructs a new Graph in the form of an Adjacency List, Head at 0,0
        */   
        Graph();

        /**
        * @brief    Constructs a new Graph in the form of an Adjacency List, user specificed head location
        
        * @param[in]    _xHead  xCordinate of the head of the graph
        * @param[in]    _yHead  yCordinate of the head of the graph
        */   
        Graph(double _xHead, double _yHead);

        /**
        * @brief    Destroys the Graph 
        */
        ~Graph();

        /**
        * @brief    Returns the coordinates of a given Node/Vertex
        * 
        * @return   The Coordinates of type coordinate_t
        */
        Node::coordinate_t getCoordinate(Node* _handle);

        /**
        * @brief    Console prints the Graph, used for debugging
        */
        void printGraph();

        /**
        * @brief    Inserts a new Node/Vertex into the Graph connected to the last node
        *
        * @param[in]    _point Coordinates of the new Node/Vertex
        * @param[in]    _back_edge_weight The weight of the back-connected edge
        */           
        void addNode(Node::coordinate_t _point, double _back_edge_weight);

         /**
        * @brief    Inserts a new Node/Vertex into the Graph connected to selected node
        *
        * @param[in]    _link Pointer to the Graph-Node we want to connect our new node to
        * @param[in]    _point Coordinates of the new Node/Vertex
        * @param[in]    _back_edge_weight The weight of the back-connected edge
        */         
        void addNode(Node* _link, Node::coordinate_t _point, double _back_edge_weight);

         /**
        * @brief    Removes a specified Node/Vertex from the Graph, Deleted Node's forward links are migrated to the back-connected Node
        *
        * @param[in]    _handle Pointer of the Node/Vertex we want removed
        */        
        void deleteNode(Node* _handle);

         /**
        * @brief    Removes a specified Node/Vertex from the Graph, Deleted Node's forward links are migrated to the back-connected Node
        *
        * @param[in]    _handle Pointer of the Node/Vertex we want removed
        * @return   The Pointer to a node which corresponds to an index of the adjacency list
        */          
        int getIndex(Node* _handle);

         /**
        * @brief    Creates a new connection/edge between two existing nodes/vertices in the adajacency list
        *
        * @param[in]    _src Pointer of the Node/Vertex we want the directed edge to originate from
        * @param[in]    _dest Pointer to the destination Node/Vertex we want the directed edge to point to
        */   
        void addEdge(Node* _src, Node* _dest);
    
        std::vector<Node *> adjacencyList_; /**< The Adjacency List which represents our graph */

    };
}

#endif /* Graph_H_ */