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
#ifndef GRAPH_H_
#define GRAPH_H_

#include <vector>
#include <algorithm>
#include <tuple>
#include <iostream>

namespace rrt
{
    /// @brief x,y,time,heading
    typedef std::tuple<double, double, double, double> pose_t;
    class Node
    {
        public:
                /**
                 * @brief   Constructs a new Node/Vertex
                 */   
                Node();

                /**
                 * @brief   Constructs a new Node/Vertex
                 *
                 * @param[in]  _pose  Cordinates of the Node/Vertex
                 */                
                Node(pose_t _pose);

                /**
                 * @brief   Constructs a new Node/Vertex
                 *
                 * @param[in]  _pose Coordinates of the Node/Vertex
                 * * @param[in]  _back_edge_weight  The weight of the back-connected edge
                 */
                Node(pose_t _pose, double _back_edge_weight);

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
                 * @brief   Dynamically removes a connection to another Node/vertex in the forward direction
                 *
                 * @param[in]  _cnnctn  Pointer to the forward node we will disconnect from
                 */
                void removeFwdNode(Node * _cnnctn);

                /**
                 * @brief Console prints Node/Vertex Parameters, used for debugging
                */
                void debugPrintNode();

                /**
                 * @brief   Returns the x-axis coordinate of the node/vertex 
                 *
                 * @return  x-axis coordinate
                 */
                double xCrdnt() const;

                /**
                 * @brief   Returns the x-axis coordinate of the node/vertex 
                 *
                 * @return  y-axis coordinate
                 */
                double yCrdnt() const;

                /**
                 * @brief   Returns the time/temporal component of the node/vertex 
                 *
                 * @return  timestamp
                 */
                double time() const;

                /**
                 * @brief   Returns the time/temporal component of the node/vertex 
                 *
                 * @return  timestamp
                 */
                double heading() const;

                /**
                 * @brief   Sets/updates the coordinates for the Node/Vertex
                 * 
                 * @param[in]  _x   The x-axis cordinate of the Node/Vertex
                 * @param[in]  _y   The y-axis cordinate of the Node/Vertex
                 * @param[in]  _tm  The time/temporal component of the Node/Vertex (assumes 3D)
                 * @param[in]  _hdng  The heading component of the Node/Vertex
                 */
                void setPose(double _x, double _y, double _tm, double _hdng);

                /**
                 * @brief   Sets/updates the coordinates for the Node/Vertex
                 * 
                 * @param[in]  _pose   The desired Values for the coordinates
                 */
                void setPose(pose_t _pose);

                /**
                 * @brief   Gets the coordinates for the Node/Vertex
                 * 
                 * @return  The Coordinates of type pose_t
                 */
                pose_t Pose() const;

                /**
                 * @brief   Gets the Pointer to the Backward-connected Node/Vertex aka Back-Edge
                 *
                 * @return  Pointer to Backward-connected Node/Vertex aka Back-Edge
                 */
                Node* BackCnnctn() const;

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
                double backEdgeWeight() const;

                /**
                 * @brief  Sets/Updates the weight of the Backward-connected Node/Vertex aka Back-Edge
                 *
                 * @param[in]  _back_edge_weight  The desired/new weight of the back-connected edge
                 */
                void setBackEdgeWeight(double _back_edge_weight);

                /**
                 * @brief   Returns a const reference to forward connections
                 * @note    Prefer this for tests/inspection. Use addFwdNode/remove via public API when mutating.
                 */
                const std::vector<Node*>& getFwdNodes() const;

                 /**
                 * @brief   Returns the number of forward connections
                 * @note    Prefer this for tests/inspection. 
                 */
                size_t fwdNodeCount() const;

                /**
                 * @brief   Checks if a given node is a forward connection
                 * 
                 * @param[in]    n  Pointer to the node to check
                 * @return   True if the node is a forward connection, false otherwise
                 */
                bool hasFwdNode(Node* n) const;

                /**
                 * @brief   Returns the forward connection at a given index
                 * 
                 * @param[in]    i  Index of the forward connection to retrieve
                 * @return   Pointer to the forward connection at the given index
                 */
                Node* fwdNodeAt(size_t i);

                /**
                 * @brief   Returns the forward connection at a given index (const version)
                 * 
                 * @param[in]    i  Index of the forward connection to retrieve
                 * @return   Pointer to the forward connection at the given index
                 */
                const Node* fwdNodeAt(size_t i) const;

            private:
                std::vector<Node *> fwd_node_;
                Node* back_node_;
                pose_t pose_;
                double back_edge_weight_;
    };

    class Graph {
    private:
        bool initCmplt_ = false;
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
        * @return   The Coordinates of type pose_t
        */
        pose_t getCoordinate(Node* _handle) const;

        /**
        * @brief    Console prints the Graph, used for debugging
        */
        void printGraph() const;

        /**
        * @brief    Inserts a new Node/Vertex into the Graph connected to the last node
        *
        * @param[in]    _point Coordinates of the new Node/Vertex
        * @param[in]    _back_edge_weight The weight of the back-connected edge
        */           
        void addNode(pose_t _point, double _back_edge_weight);

         /**
        * @brief    Inserts a new Node/Vertex into the Graph connected to selected node
        *
        * @param[in]    _link Pointer to the Graph-Node we want to connect our new node to
        * @param[in]    _point Coordinates of the new Node/Vertex
        * @param[in]    _back_edge_weight The weight of the back-connected edge
        */         
        void addNode(Node* _link, pose_t _point, double _back_edge_weight);

        /**
        * @brief    Takes a local-dynamic node, copies it, and appends it to the graph connected to the same node as the original
        *
        * @param[in]    _copy Pointer to the Graph-Node we want to copied and appended to the graph
        */         
        void pushNode(Node* _copy);

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
        int getIndex(Node* _handle) const;

         /**
        * @brief    Creates a new connection/edge between two existing nodes/vertices
        *
        * @param[in]    _src Pointer of the Node/Vertex we want the directed edge to originate from
        * @param[in]    _dest Pointer to the destination Node/Vertex we want the directed edge to point to
        */   
        void addEdge(Node* _src, Node* _dest);

        /**
        * @brief    Deletes an connection/edge between two existing nodes/vertices 
        *
        * @param[in]    _src Pointer of the Node/Vertex 
        * @param[in]    _dest Pointer to the destination Node/Vertex 
        */ 
        void deleteEdge(Node* _src, Node* _dest);

        /**
        * @brief    Reassigns the back connection/edge and does housekeeping to remove the fwd link in the old back node
        *
        * @param[in]    _src Pointer of the Node/Vertex of the NEW source
        * @param[in]    _fwd Pointer to the destination Node/Vertex we want the directed edge to point to
        */   
        void updateEdge(Node* _src, Node* _fwd);
    
        std::vector<Node *> adjacencyList_;
    };

} // namespace rrt

#endif /* GRAPH_H_ */