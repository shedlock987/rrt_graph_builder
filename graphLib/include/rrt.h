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
 * @brief RRT builder using adjacency list graph
 * @author Ryan Shedlock <rmshedlock@gmail.com>
 * @version 1.0
 */
#ifndef RRT_H_
#define RRT_H_

#include "graph.h"
#include <cmath>
#include <cfloat>
#include <tuple>
#include <random>
#include <iomanip>

namespace rrt
{
    class RRT : public Graph
    {
        private:
        double max_angle_rad_; /**< Constraint: maximum permitted angle between 2 Nodes */
        double max_dist_; /**< Constraint: maximum permitted distance between 2 Nodes */
        double min_dist_; /**< Constraint: minimum permitted distance between 2 Nodes */
        double max_interval; /**< Constraint: maximum time interval between 2 Nodes */
        double max_time_;   /**< Constraint: maximum time interval between 2 Nodes */
        bool dim_3D_ = false; /**< Specifies if the RRT is 2D or 3D, initialized to 2D, no temporal */
        bool cmplt = false; /**< Flag to indicate the RRT is complete */
        int node_limit_; /**< Maximum number of nodes permitted in the RRT */
        int dest_cnnctn_limit = 10; 
        Node* endNode;

        Node::coordinate_t range_a_; /**< Lower Left Corner of the Operating Region */
        Node::coordinate_t range_b_; /**< Upper Right Corner of the Operating Region */
        Node::coordinate_t origin_; /**< Origin of the RRT */
        Node::coordinate_t dest_; /**< Destination of the RRT */

        /**
        * @brief   Searches the adjacency list for the nearest node to the given node
        * 
        * @param[in]    _handle  The given node we want to reference the search against 
        * @return   Pointer to the Graph-Node which is nearest to the handle
        */
        Node* findNearest(Node *_handle);

        /**
        * @brief   Calculates the 2D eculidian distance between two nodes 
        * 
        * @param[in]    _ref  The first node
        * @param[in]    _handle  The second node
        * @return   Eculidian distance between the two nodes
        */
        double calcDist(Node *_handle, Node *_ref);

        /**
        * @brief   Calculates the angle (in radians) between two nodes 
        * 
        * @param[in]    _ref  The first node
        * @param[in]    _handle  The second node
        * @return   Angle between the two nodes
        */       
        double calcAngle(Node *_handle, Node *_ref);

        /**
        * @brief   Calculates an edge weight based upon kinematic cost constraints
        * 
        * @param[in]    _ref  The first node
        * @param[in]    _handle  The second node
        * @return   Edge weight based upon kinematic cost constraints
        */            
        double calcKinematicEdge(Node *_handle, Node *_ref);

        /**
        * @brief   Checks if the constraints (distance, angle, interval) are satisfied between a given node and its nearest neighbor
        * 
        * @note   This is only intended to be used by the Build function since it only checks against the nearest neighbor
        * @param[in]    _handle  The given node we want to check constraints for
        * @return   Returns true if the constraints are satisfied, false otherwise
        */        
        bool checkConstraints(Node *_handle);

        /**
        * @brief   Applies/Updates node parameters to ensure the constraints are satisfied
        * 
        * @note    Also ensures the node is within the operating region
        * @param[in]    _handle  The given node we want to force constraints upon
        */
        void applyConstraints(Node *_handle);

        /**
        * @brief   Checks if the RRT gaph has a Node near the desired destination and said node is within the constraints relative to the destination
        * 
        * @note    Sets this->cmplt = true if the destination is reached within constraints
        * */
        void checkDone();



        public:
        /**
        * @brief    Constructs/Initializes a new RRT Graph in the form of an Adjacency List
        */   
        RRT();

        /**
        * @brief    Constructs/Initializes a new RRT Graph in the form of an Adjacency List
        * 
        * @param[in]    _range_a  Lower Left Corner of the Operating Region
        * @param[in]    _range_b  Upper Right Corner of the Operating Region
        * @param[in]    _origin   Origin of the RRT
        * @param[in]    _dest     Destination of the RRT
        * @param[in]    _max_angle_rad  Constraint: maximum permitted angle between 2 Nodes
        * @param[in]    _max_dist Constraint: maximum permitted distance between 2 Nodes
        * @param[in]    _min_dist Constraint: minimum permitted distance between 2 Nodes
        * @param[in]    _max_time Constraint: maximum time interval between 2 Nodes
        * @param[in]    _dim     Specifies if the RRT is 2D or 3D
        * @param[in]    _node_limit  Maximum number of nodes permittedin the RRT
        */           
        RRT(Node::coordinate_t _range_a, Node::coordinate_t _range_b,
            Node::coordinate_t _origin, Node::coordinate_t _dest,
            double _max_angle_rad, double _max_dist, double _min_dist,
            double _max_time, 
            bool _dim, int _node_limit);

        /**
        * @brief    Constructs/Initializes a new RRT Graph in the form of an Adjacency List, No temporal component
        * 
        * @param[in]    _range_a  Lower Left Corner of the Operating Region
        * @param[in]    _range_b  Upper Right Corner of the Operating Region
        * @param[in]    _origin   Origin of the RRT
        * @param[in]    _dest     Destination of the RRT
        * @param[in]    _max_angle_rad  Constraint: maximum permitted angle between 2 Nodes
        * @param[in]    _max_dist Constraint: maximum permitted distance between 2 Nodes
        * @param[in]    _min_dist Constraint: minimum permitted distance between 2 Nodes
        * @param[in]    _max_time Constraint: maximum time interval between 2 Nodes
        * @param[in]    _dim     Specifies if the RRT is 2D or 3D
        * @param[in]    _node_limit  Maximum number of nodes permittedin the RRT
        */        
        RRT(Node::coordinate_t _range_a, Node::coordinate_t _range_b,
            Node::coordinate_t _origin, Node::coordinate_t _dest,
            double _max_angle_rad, double _max_dist, double _min_dist, 
            int _node_limit);

        /**
        * @brief    Destroys the RRT Graph 
        */
        ~RRT();

        /**
        * @brief    Generates the RRT Graph
        */        
        void buildRRT();
    };
}

#endif /* RRT_H_ */