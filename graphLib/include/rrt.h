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
#include <optional>

namespace rrt
{
    class RRT : public Graph
    {
        private:
        double max_angle_rad_; /**< Constraint: maximum permitted angle between 2 Nodes */
        double max_kappa_rad_; /**< Constraint: maximum permitted curvature between 3 Nodes */
        double max_long_accel_; /**< Constraint: maximum permitted longitudinal acceleration between 2 Nodes */
        double max_long_jerk_; /**< Constraint: maximum permitted longitudinal jerk between 3 Nodes */
        double max_dist_; /**< Constraint: maximum permitted distance between 2 Nodes */
        double min_dist_; /**< Constraint: minimum permitted distance between 2 Nodes */
        double max_interval_; /**< Constraint: maximum time interval between 2 Nodes */
        double max_time_;   /**< Constraint: maximum time */
        bool dim_3D_ = false; /**< Specifies if the RRT is 2D or 3D, initialized to 2D, no temporal */
        bool cmplt = false; /**< Flag to indicate the RRT is complete */
        bool admissible_ = false; /**< An Admissible solution exists to the destination node */
        int iteration_limit_; /**< Maximum number of of iterations, protect against infinite loops when no admissability */
        static constexpr int sequential_check_limit_ = 15; /**< After this many sequential nodes in occupied space, we stop and assume a non-admissible trajectory*/
        double initial_heading_ = 0.0F; /**< Initial heading of the RRT at the origin node */
        int max_admissible_; /**< Maximum number of admissible trajectories */
        std::vector<Node *> destNodes; /**< list of admissible end/destination nodes */

        pose_t range_a_; /**< Lower Left Corner of the Operating Region */
        pose_t range_b_; /**< Upper Right Corner of the Operating Region */
        pose_t origin_; /**< Origin of the RRT */
        pose_t dest_; /**< Destination of the RRT */

        /**
        * @brief   Searches the adjacency list for the nearest node to the given node
        * 
        * @param[in]    _handle  The given node we want to reference the search against 
        * @param[in]    _temporal if true will return 3D nearest (z-plane beiing time). otherwise only cares about x,y
        * @note         Excludes/omits destination node from search
        * @return       Pointer to the Graph-Node which is nearest to the handle
        */
        Node* findNearest(Node *_handle, bool _temporal);

        /**
        * @brief   Calculates the 2D eculidian distance between two nodes 
        * 
        * @param[in]    _ref  The first node
        * @param[in]    _handle  The second node
        * @param[in]    _temporal if true will return 3D distance. Otherwise only cares about x,y
        * @return   Eculidian distance between the two nodes
        */
        double calcDist(Node *_handle, Node *_ref, bool _temporal);

        /**
        * @brief   Calculates the angle (in radians) between two nodes 
        * 
        * @param[in]    _ref  The first node
        * @param[in]    _handle  The second node
        * @return   Angle between the two nodes
        * @note     The angle is calculated as the delta between the pose of the reference node and the eculidean position of the handle
        * @note     It is assumed that setting of the pose of the handle will be done after/elsewhere
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

        /**
        * @brief   Calculates the 2D (x,y) curvature between 3 Nodes using the Menger Curvature formula
        * 
        * @param[in]    _ref0  The first node
        * @param[in]    _ref1  The second node
        * @param[in]    _ref2  The third node
        * */
        double calcMengerCurvature(Node* _ref0, Node* _ref1, Node* _ref2);

        /**
         * @brief   Adjusts spataial parameters of the current node to ensure curvature constraints are satisfied
         * 
         * @param[in]    _handle  The current node we want to constrain
         */
        void constrainCurvature(Node* _handle);

        /**
         * @brief   Calculates the maximum permitted spatial constraints based on absolute limit params and applies it to the current node
         * 
         * @param[in]    _handle  The current node we want to constrain
         * @param[in]    _nearest  The nearest node to the current node
         */
        void applyAbsoluteConstraints(Node* _handle, Node* _nearest);

        /**
         * @brief   Aligns the heading of the current node to be tangent to the menger curvature from the previous 2 nodes
         * 
         * @param[in]    _handle  The current node we want to align
         * @param[in]    _center_x  The x coordinate of the center of curvature
         * @param[in]    _center_y  The y coordinate of the center of curvature
         */
        void alignHeadingToTangent(Node* _handle, double _center_x, double _center_y);

        /**
         * @brief   Calculates the longitudinal acceleration between two nodes
         * 
         * @param[in]    _handle  The current node
         * @param[in]    _ref  The reference node
         * @return   The longitudinal acceleration between the two nodes (units per time^2)
         */
        double calcLongAccel(Node* _handle, Node* _ref);

        /**
         * @brief   Calculates the longitudinal jerk between two nodes
         * 
         * @param[in]    _handle  The current node
         * @param[in]    _ref0  The first reference node
         * @return   The longitudinal jerk between the two nodes (units per time per time^3)
         */
        double calcLongJerk(Node* _handle, Node* _ref);

        /**
         * @brief   Adjusts spataial and temporal parameters of the current node to ensure longitudinal dynamic constraints are satisfied
         * 
         * @param[in]    _handle  The current node
         */
        void constrainLongitudinal(Node* _handle);

        public:
        /**
        * @brief   Compares a node (coordinates) against the occupancy map to determine if it is occupied
        * 
        * @param[in]    _handle  The node (coordinates) we want
        * @return   Returns true if the node is occupied, false otherwise
        */
        bool isOccupied(Node *_handle);

        /**
         * @brief An custom type representing a single voxel in an occupancy map.
         *
         * @details This is represented as a pair containing:
         *          - pose_t: The coordinates of the voxel (x, y, time). Where Time is a physical dimension
         *          - double: A double representing the size of the voxel in meters.
         * @note    The coordinate is at the 2D (x,y) geometric center of the grid cell, and extends vertical by 1 unit time
         */
        typedef std::pair<pose_t, double> occupancy_t; 

        /**
         * @brief An Occupancy Map (Grid) represented as a simple list of occupied voxels
         *
         * @details This is represented as a vector of occupancy_t, where each element (pose_t)
         *          present in the list implies occupied space/time. Space is assumed to be free if 
         *          not present in the list. 
         */
        std::optional<std::vector<occupancy_t>> occupancy_map_; 

        /**
        * @brief    Constructs/Initializes a new RRT Graph in the form of an Adjacency List
        * @note     ADefault constructor, Freespaceboundaries are set to reserved values
        * 
        * @param[in]    _occupancy_map  Optional occupancy map, if provided, will be used to check for freespace
        */   
        RRT(const std::optional<std::vector<occupancy_t>> _occupancy_map);

        /**
        * @brief    Constructs/Initializes a new RRT Graph in the form of an Adjacency List
        * @note     DConstuctor where boundaries are explicitly defined by x and y coordinates, includes occupancy map
        * 
        * @param[in]    _occupancy_map  Optional occupancy map, if provided, will be used to check for freespace
        * @param[in]    _range_a_x  X Coordinate Lower Left Corner of the Operating Region
        * @param[in]    _range_a_y  Y Coordinate Lower Left Corner of the Operating Region
        * @param[in]    _range_b_x  X Coordinate Upper Right Corner of the Operating Region
        * @param[in]    _range_b_y  Y Coordinate Upper Right Corner of the Operating Region
        * @param[in]    _origin_x  The x coordinate of the origin  
        * @param[in]    _origin_y  The y coordinate of the origin 
        * @param[in]    _dest_x  The x coordinate of the destination
        * @param[in]    _dest_y  The y coordinate of the destination
        * @param[in]    _max_angle_rad  Constraint: maximum permitted angle between 2 Nodes
        * @param[in]    _max_dist Constraint: maximum permitted distance between 2 Nodes
        * @param[in]    _min_dist Constraint: minimum permitted distance between 2 Nodes
        * @param[in]    _max_interval Constraint: maximum time interval between 2 Nodes
        * @param[in]    _max_time Absolute temporal boundary/limit for the RRT
        * @param[in]    _dim     Specifies if the RRT is 2D or 3D
        * @param[in]    _iteration_limit  Maximum number of nodes permittedin the RRT
        * @param[in]    _max_admissible  Maximum number of admissible trajectories permitted in the graph
        */   
        RRT(std::vector<occupancy_t> _occupancy_map,
            double _range_a_x, double _range_a_y, double _range_b_x, double _range_b_y,
            double _origin_x, double _origin_y, double _dest_x, double _dest_y,
            double _max_angle_rad, double _max_dist, double _min_dist,
            double _max_interval, double _max_time, bool _dim, int _iteration_limit, int _max_admissible);

       /**
        * @brief    Constructs/Initializes a new RRT Graph in the form of an Adjacency List
        * @note     Constuctor where boundaries are of the type pose_t, incluides occupancy map
        * 
        * @param[in]    _occupancy_map  Optional occupancy map, if provided, will be used to check for freespace
        * @param[in]    _range_a  Lower Left Corner of the Operating Region
        * @param[in]    _range_b  Upper Right Corner of the Operating Region
        * @param[in]    _origin   Origin of the RRT
        * @param[in]    _dest     Destination of the RRT
        * @param[in]    _max_angle_rad  Constraint: maximum permitted angle between 2 Nodes
        * @param[in]    _max_dist Constraint: maximum permitted distance between 2 Nodes
        * @param[in]    _min_dist Constraint: minimum permitted distance between 2 Nodes
        * @param[in]    _max_interval Constraint: maximum time interval between 2 Nodes
        * @param[in]    _max_time Absolute temporal boundary/limit for the RRT
        * @param[in]    _dim     Specifies if the RRT is 2D or 3D
        * @param[in]    _iteration_limit  Maximum number of nodes permittedin the RRT
        * @param[in]    _max_admissible  Maximum number of admissible trajectories permitted in the graph
        */     
        RRT(std::vector<occupancy_t> _occupancy_map,
        pose_t _range_a, pose_t _range_b,
        pose_t _origin, pose_t _dest,
        double _max_angle_rad, double _max_dist, double _min_dist, 
        double _max_interval, double _max_time, bool _dim, int _iteration_limit,
        int _max_admissible);

        /**
        * @brief    Constructs/Initializes a new RRT Graph in the form of an Adjacency List
        * @note     Constuctor where boundaries are of the type pose_t
        * 
        * @param[in]    _range_a  Lower Left Corner of the Operating Region
        * @param[in]    _range_b  Upper Right Corner of the Operating Region
        * @param[in]    _origin   Origin of the RRT
        * @param[in]    _dest     Destination of the RRT
        * @param[in]    _max_angle_rad  Constraint: maximum permitted angle between 2 Nodes
        * @param[in]    _max_dist Constraint: maximum permitted distance between 2 Nodes
        * @param[in]    _min_dist Constraint: minimum permitted distance between 2 Nodes
        * @param[in]    _max_interval Constraint: maximum time interval between 2 Nodes
        * @param[in]    _max_time Absolute temporal boundary/limit for the RRT
        * @param[in]    _dim     Specifies if the RRT is 2D or 3D
        * @param[in]    _iteration_limit  Maximum number of nodes permittedin the RRT
        * @param[in]    _max_admissible  Maximum number of admissible trajectories permitted in the graph
        */           
        RRT(pose_t _range_a, pose_t _range_b,
            pose_t _origin, pose_t _dest,
            double _max_angle_rad, double _max_dist, double _min_dist,
            double _max_interval, double _max_time, 
            bool _dim, int _iteration_limit,
            int _max_admissible);

        /**
        * @brief    Constructs/Initializes a new RRT Graph in the form of an Adjacency List
        * @note     Constuctor where boundaries are explicitly defined by x and y coordinates
        * 
        * @param[in]    _range_a_x  X Coordinate Lower Left Corner of the Operating Region
        * @param[in]    _range_a_y  Y Coordinate Lower Left Corner of the Operating Region
        * @param[in]    _range_b_x  X Coordinate Upper Right Corner of the Operating Region
        * @param[in]    _range_b_y  Y Coordinate Upper Right Corner of the Operating Region
        * @param[in]    _origin_x  The x coordinate of the origin  
        * @param[in]    _origin_y  The y coordinate of the origin 
        * @param[in]    _dest_x  The x coordinate of the destination
        * @param[in]    _dest_y  The y coordinate of the destination
        * @param[in]    _max_angle_rad  Constraint: maximum permitted angle between 2 Nodes
        * @param[in]    _max_dist Constraint: maximum permitted distance between 2 Nodes
        * @param[in]    _min_dist Constraint: minimum permitted distance between 2 Nodes
        * @param[in]    _max_interval Constraint: maximum time interval between 2 Nodes
        * @param[in]    _max_time Absolute temporal boundary/limit for the RRT
        * @param[in]    _dim     Specifies if the RRT is 2D or 3D
        * @param[in]    _iteration_limit  Maximum number of nodes permittedin the RRT
        * @param[in]    _max_admissible  Maximum number of admissible trajectories permitted in the graph
        */        
        RRT(double _range_a_x, double _range_a_y, double _range_b_x, double _range_b_y,
            double _origin_x, double _origin_y, double _dest_x, double _dest_y,
            double _max_angle_rad, double _max_dist, double _min_dist, double _max_interval, 
            double _max_time, bool _dim, int _iteration_limit,
            int _max_admissible);

        /**
        * @brief    Destroys the RRT Graph 
        */
        ~RRT();

        /**
        * @brief    Builds the RRT Graph through completion
        */        
        void buildRRT();

        /**
        * @brief    Builds the RRT Graph One node at a time
        * @details
        *           It is intended to use this function in a loop to build the RRT incrementally
        *           This will allow ther user to dynamically scale/change the constraints or update
        *           the destination
        * @return   Returns true if the RRT is complete, false otherwise
        */        
        bool stepRRT();

                /**
        * @brief   Sets the boundaries of where the RRT will build
        * 
        * @param[in]    _range_a  Lower Left Corner of the Operating Region
        * @param[in]    _range_b  Upper Right Corner of the Operating Region
        */
        void setBoundaries(pose_t _range_a, pose_t _range_b);

        /**
        * @brief   Sets the boundaries of where the RRT will build
        * 
        * @param[in]    _range_a_x  X Coordinate Lower Left Corner of the Operating Region
        * @param[in]    _range_a_y  Y Coordinate Lower Left Corner of the Operating Region
        * @param[in]    _range_b_x  X Coordinate Upper Right Corner of the Operating Region
        * @param[in]    _range_b_y  Y Coordinate Upper Right Corner of the Operating Region
        * @param[in]    _time_horizon  Maximum time horizon for the RRT
        */
        void setBoundaries(double _range_a_x, double _range_a_y, 
                           double _range_b_x, double _range_b_y, double _time_horizon);

        /**
        * @brief   Updates/sets the geometric location of the origin node of the RRT
        * 
        * @param[in]    _origin  The coordinate of the origin  
        */       
        void setOrigin(pose_t _origin);

        /**
        * @brief   Updates/sets the geometric location of the origin node of the RRT
        * 
        * @param[in]    _origin_x  The x coordinate of the origin  
        * @param[in]    _origin_y  The y coordinate of the origin 
        */     
        void setOrigin(double _origin_x, double _origin_y);

        /**
        * @brief   Updates/sets the geometric location of the origin node of the RRT
        * 
        * @param[in]    _origin_x  The x coordinate of the origin  
        * @param[in]    _origin_y  The y coordinate of the origin 
        * @param[in]    _origin_time  The time coordinate of the origin
        */     
        void setOrigin(double _origin_x, double _origin_y, double _origin_time);

        /**
        * @brief   Updates/sets the geometric location of the origin node of the RRT
        * 
        * @param[in]    _origin_x  The x coordinate of the origin  
        * @param[in]    _origin_y  The y coordinate of the origin 
        * @param[in]    _origin_time  The time coordinate of the origin
        * @param[in]    _initial_heading  The initial heading of the RRT at the origin node
        */     
        void setOrigin(double _origin_x, double _origin_y, double _origin_time, double _initial_heading);

        /**
        * @brief   Updates/sets the geometric location of the destination node of the RRT
        * 
        * @param[in]    _dest  The coordinate of the destination  
        */            
        void updateDestination(pose_t _dest);

        /**
        * @brief   Updates/sets the geometric location of the destination node of the RRT
        * 
        * @param[in]    _dest_x  The x coordinate of the destination  
        * @param[in]    _dest_y  The y coordinate of the destination
        */ 
        void updateDestination(double _dest_x, double _dest_y);

            /**
        * @brief   Updates/sets the geometric location of the destination node of the RRT
        * 
        * @param[in]    _dest_x  The x coordinate of the destination  
        * @param[in]    _dest_y  The y coordinate of the destination
        */ 
        void updateDestination(double _dest_x, double _dest_y, double _dest_time);

        /**
        * @brief   Updates/sets the geometric constraints for next-node placement
        * 
        * @param[in]    _max_angle_rad  maximum angle allowed between two nodes in radians
        * @param[in]    _max_dist  maximum distance allowed between two nodes
        * @param[in]    _min_dist  minimum distance allowed between two nodes
        * @param[in]    _max_time  maximum <positive> time interval allowed between two nodes 
        */ 
        void updateConstraints(double _max_angle_rad, double _max_dist, double _min_dist, double _max_interval_);

        /**
        * @brief   Sets the RRT to be 2D or 3D
        * 
        * @param[in]    _dim_3D  If true, the RRT will be 3D, otherwise it will be 2D  
        */   
        void setDim3D(bool _dim_3D);

        /**
        * @brief    Inj the event of an non-admissible solution this limit protects against infinite loops
        * @param[in]    _iteration_limit  The maximum number of iterations permitted in the RRT
        */
        void setIterationLimit(int _iteration_limit);

        /**
        * @brief   Sets the occupancy map for the RRT
        * @note    if using the stepRRT function, the occupancy map must be set before the first call
        * 
        * @param[in]    _occupancy_map  Optional occupancy map, if prov
        */ 
        void setOccupancyMap(std::vector<occupancy_t> &_occupancy_map);

        /**
         * @brief   Returns the completion state of the RRT
         * @return   Returns true if the RRT is complete, false otherwise
         */
        bool isComplete();

        /**
         * @brief   Returns the admissability state of the RRT
         * @return   Returns true if the RRT has an admissible solution to the destination node with the set constraints
         */
        bool isAdmissible();

        /**
         * @brief   Overrides the initial heading of the RRT
         */
        void updateInitialHeading(double _initial_heading);
    };
}

#endif /* RRT_H_ */