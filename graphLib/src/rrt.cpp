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
 

 #include "rrt.h"
 

 namespace rrt 
 {
    RRT::RRT(const std::optional<std::vector<occupancy_t>> _occupancy_map) : 
        occupancy_map_(_occupancy_map)
    {
        setBoundaries(-5.0F, -5.0F, 5.0F, 5.0F, 10.0F);
        setOrigin(0.0F, -5.0F);
        updateDestination(5.0F, 5.0F);
        updateConstraints(1.05F, 1.0F, 0.5F, 0.0F);
        setDim3D(false);
        iteration_limit_ = 50;
        initial_heading_ = 0.0F;
        max_admissible_ = 1;
    }

    RRT::RRT(std::vector<occupancy_t> _occupancy_map,
            double _range_a_x, double _range_a_y, double _range_b_x, double _range_b_y,
            double _origin_x, double _origin_y, double _dest_x, double _dest_y,
            double _max_angle_rad, double _max_dist, double _min_dist,
            double _max_interval, double _max_time, bool _dim, int _iteration_limit,
            int _max_admissible) : 
        occupancy_map_(_occupancy_map), 
        range_a_(std::make_tuple(_range_a_x, _range_a_y, 0.0F, 0.0F)),
        range_b_(std::make_tuple(_range_b_x, _range_b_y, _max_time, 0.0F)),
        origin_(std::make_tuple(_origin_x, _origin_y, 0.0F, 0.0F)),
        dest_(std::make_tuple(_dest_x, _dest_y, 0.0F, 0.0F)),
        max_angle_rad_(_max_angle_rad), max_dist_(_max_dist), min_dist_(_min_dist),
        max_interval_(_max_interval), max_time_(_max_time), dim_3D_(_dim), 
        iteration_limit_(_iteration_limit), max_admissible_(_max_admissible)
    {
        setOrigin(_origin_x, _origin_y);
        setBoundaries(_range_a_x, _range_a_y, _range_b_x, _range_b_y, _max_time);
        Node *end = new Node(_dest_x, _dest_y, 0.0F, 0.0F);
        initial_heading_ = calcAngle(adjacencyList_.front(), end);
        adjacencyList_.front()->setPose(_range_a_x, _range_a_y, 0.0F, initial_heading_);
    }

    RRT::RRT(std::vector<occupancy_t> _occupancy_map,
        pose_t _range_a, pose_t _range_b,
        pose_t _origin, pose_t _dest,
        double _max_angle_rad, double _max_dist, double _min_dist,
        double _max_interval, double _max_time, bool _dim, int _iteration_limit,
        int _max_admissible) :
        occupancy_map_(_occupancy_map),
        range_a_(_range_a), range_b_(_range_b),
        origin_(_origin), dest_(_dest),
        max_angle_rad_(_max_angle_rad), max_dist_(_max_dist), min_dist_(_min_dist),
        max_interval_(_max_interval), max_time_(_max_time), dim_3D_(_dim),
        iteration_limit_(_iteration_limit), max_admissible_(_max_admissible)
    {
        setOrigin(_origin);
        setBoundaries(_range_a, _range_b);
    }

    RRT::RRT(double _range_a_x, double _range_a_y, double _range_b_x, double _range_b_y,
        double _origin_x, double _origin_y, double _dest_x, double _dest_y,
        double _max_angle_rad, double _max_dist, double _min_dist, 
        double _max_interval, double _max_time, bool _dim, int _iteration_limit,
        int _max_admissible) :
        range_a_(std::make_tuple(_range_a_x, _range_a_y, 0.0F, 0.0F)),
        range_b_(std::make_tuple(_range_b_x, _range_b_y, _max_time, 0.0F)),
        origin_(std::make_tuple(_origin_x, _origin_y, 0.0F, 0.0F)),
        dest_(std::make_tuple(_dest_x, _dest_y, 0.0F, 0.0F)),
        max_angle_rad_(_max_angle_rad), max_dist_(_max_dist), min_dist_(_min_dist),
        max_interval_(_max_interval), max_time_(_max_time), dim_3D_(_dim), 
        iteration_limit_(_iteration_limit), max_admissible_(_max_admissible)
    {
        setOrigin(_origin_x, _origin_y);
        setBoundaries(_range_a_x, _range_a_y, _range_b_x, _range_b_y, _max_time);
        Node *end = new Node(_dest_x, _dest_y, 0.0F, 0.0F);
        initial_heading_ = calcAngle(adjacencyList_.front(), end);
        adjacencyList_.front()->setPose(_range_a_x, _range_a_y, 0.0F, initial_heading_);
    }

    RRT::RRT(pose_t _range_a, pose_t _range_b,
        pose_t _origin, pose_t _dest,
        double _max_angle_rad, double _max_dist, double _min_dist, 
        double _max_interval, double _max_time, bool _dim, int _iteration_limit,
        int _max_admissible) :
        range_a_(_range_a), range_b_(_range_b), 
        origin_(_origin), dest_(_dest),
        max_angle_rad_(_max_angle_rad), max_dist_(_max_dist), min_dist_(_min_dist), 
        max_interval_(_max_interval), max_time_(_max_time), dim_3D_(_dim),
        iteration_limit_(_iteration_limit), max_admissible_(_max_admissible)
    {
        setOrigin(_origin);
        setBoundaries(_range_a, _range_b);
    }


    RRT::~RRT() = default;

    void RRT::setBoundaries(pose_t _range_a, pose_t _range_b)
    {
        range_a_ = _range_a;
        range_b_ = _range_b;
        max_time_ = std::get<2>(_range_b); // Assuming the time component is in the z-axis
        auto start_time = std::get<2>(_range_a);
        if(start_time >= 0.0F && max_time_ > start_time)
        {
            setDim3D(true); // If the z-axis is not zero, we are in 3D
        }
        else
        {
            setDim3D(false); // Otherwise, we are in 2D
        }
    }

    void RRT::setBoundaries(double _range_a_x, double _range_a_y, 
                           double _range_b_x, double _range_b_y, double _time_horizon)
    {
        max_time_ = _time_horizon;
        range_a_ = std::make_tuple(_range_a_x, _range_a_y, 0.0F, 0.0F);
        range_b_ = std::make_tuple(_range_b_x, _range_b_y, _time_horizon, 0.0F);
        if(_time_horizon > 0.0F)
        {
            setDim3D(true); // If the z-axis is not zero, we are in 3D
        }
        else
        {
            setDim3D(false); // Otherwise, we are in 2D
        }
    }

     
    void RRT::setOrigin(pose_t _origin)
    {
        if (!adjacencyList_.empty()) {
            adjacencyList_.front()->setPose(_origin);
        }
        else
        {
            addNode(_origin, 0.0F);
        }
    }
 
    void RRT::setOrigin(double _origin_x, double _origin_y)
    {
        if (!adjacencyList_.empty()) {
            adjacencyList_.front()->setPose(_origin_x, _origin_y, 0.0F, 0.0F);
        }
        else
        {
            addNode(std::make_tuple(_origin_x, _origin_y, 0.0F, 0.0F), 0.0F);
        }
    }

    void RRT::setOrigin(double _origin_x, double _origin_y, double _origin_time)
    {
        if (!adjacencyList_.empty()) {
            adjacencyList_.front()->setPose(_origin_x, _origin_y, _origin_time, 0.0F);
        }
        else
        {
            addNode(std::make_tuple(_origin_x, _origin_y, _origin_time, 0.0F), 0.0F);
        }
    }

    void RRT::setOrigin(double _origin_x, double _origin_y, double _origin_time, double _origin_heading)
    {
        if (!adjacencyList_.empty()) {
            adjacencyList_.front()->setPose(_origin_x, _origin_y, _origin_time, _origin_heading);
        }
        else
        {
            addNode(std::make_tuple(_origin_x, _origin_y, _origin_time, 0.0F), 0.0F);
        }
    }
          
    void RRT::updateDestination(pose_t _dest)
    {
        dest_ = _dest;
    }

    void RRT::updateDestination(double _dest_x, double _dest_y)
    {
        dest_ = std::make_tuple(_dest_x, _dest_y, max_time_, 0.0F);
    }

    void RRT::updateDestination(double _dest_x, double _dest_y, double _dest_time)
    {
        if(_dest_time > max_time_)
        {
            dest_ = std::make_tuple(_dest_x, _dest_y, max_time_, 0.0F);
        }
        else
        {
            dest_ = std::make_tuple(_dest_x, _dest_y, _dest_time, 0.0F);
        }
    }

    void RRT::updateConstraints(double _max_angle_rad, double _max_dist, double _min_dist, double _max_interval)
    {
        max_angle_rad_ = _max_angle_rad;
        max_dist_ = _max_dist;
        min_dist_ = _min_dist;
        max_interval_ = _max_interval;
    }
 
    void RRT::setDim3D(bool _dim_3D)
    {
        dim_3D_ = _dim_3D;
    }

    void RRT::setIterationLimit(int _iteration_limit)
    {
        iteration_limit_ = _iteration_limit;
    }

    void RRT::setOccupancyMap(std::vector<occupancy_t> &_occupancy_map)
    {
        occupancy_map_ = _occupancy_map;
    }

    Node* RRT::findNearest(Node *_handle, bool _temporal)
    {
        int idx = -1;
        int i = 0;
        double temp;
        double min = DBL_MAX;

        /// Need to optimize 
        for(const auto &iter : adjacencyList_)
        {
            /// Ignore/do not consider any destination nodes or nodes with forward connections to destination nodes
            if (std::find(destNodes.begin(), destNodes.end(), iter) != destNodes.end())
            {
                i++;
                continue;
            }

            /// Check if this node has any forward connections to destination nodes
            bool has_fwd_to_dest = false;
            for (const auto& fwd : iter->fwd_node_)
            {
                if (std::find(destNodes.begin(), destNodes.end(), fwd) != destNodes.end())
                {
                    has_fwd_to_dest = true;
                    break;
                }
            }
            if (has_fwd_to_dest)
            {
                i++;
                continue;
            }

            temp = calcDist(_handle, iter, _temporal);

            /// Make sure we're not comparing the handle to itself 
            if(temp < min && _handle != iter)
            {
                min = temp;
                idx = i;
            }
            i++;
        }

        // If no valid nearest node found, return nullptr
        if (idx == -1)
            return nullptr;

        return adjacencyList_.at(idx);
    }

    double RRT::calcDist(Node *_handle, Node *_ref, bool _temporal)
    {
        if(_temporal && dim_3D_)
        {
            return std::sqrt(std::pow(_handle->xCrdnt() - _ref->xCrdnt(), 2) + 
                             std::pow(_handle->yCrdnt() - _ref->yCrdnt(), 2) +
                             std::pow(_handle->time() - _ref->time(), 2));
        }
        else
        {
            return std::sqrt(std::pow(_handle->xCrdnt() - _ref->xCrdnt(), 2) + 
                             std::pow(_handle->yCrdnt() - _ref->yCrdnt(), 2));
        }
    }

    double RRT::calcAngle(Node *_handle, Node *_ref)
    {
        double delta_x = _handle->xCrdnt() - _ref->xCrdnt();
        double delta_y = _handle->yCrdnt() - _ref->yCrdnt();
        double angle = std::atan2(delta_y, delta_x);

        if(_ref->heading() > angle)
        {
            angle += _ref->heading();
        }
        else if (_ref->heading() < angle)
        {
            angle -= _ref->heading();
        }
        
        return angle;
    }

    double RRT::calcKinematicEdge(Node *_handle, Node *_ref)
    {
        /// Calculate a composite back-edge weight based on kinematic cost 
        return 0;
    }

    bool RRT::checkConstraints(Node *_handle)
    {
        Node *nearest = findNearest(_handle, true);
        double dist = calcDist(_handle, nearest, true);
        double angle = calcAngle(_handle, nearest);
        double tm = _handle->time();
        auto eplison = 0.0001F;

        if((std::abs(angle) < max_angle_rad_ || std::abs(std::abs(angle) - max_angle_rad_) <= eplison) &&
           (std::abs(dist) < max_dist_ || std::abs(std::abs(dist) - max_dist_) <= eplison) &&
           (std::abs(dist) > min_dist_ || std::abs(std::abs(dist) - min_dist_) <= eplison) &&
           (tm >= nearest->time() || (std::fabs(tm - nearest->time() <= eplison))) &&
           (tm <= (nearest->time() + max_interval_) || std::abs(nearest->time() + max_interval_ - max_interval_) <= eplison)
           )
        {
            return true;
        }
        else 
        {
            return false;
        }
    }

    void RRT::applyConstraints(Node *_handle)
    {
        Node *nearest = findNearest(_handle, true);

        /// connects our new node to the nearest while also destroying the fwd 
        /// connection in the old back link to prevent loop-back in RRT
        updateEdge(nearest, _handle);

        ///Impose Spatial Absolute constraints (Angle and Distance)
        applyAbsoluteConstraints(_handle, nearest);

        ///Impose Dynamic constraints (Curvature)
        double kappa = calcMengerCurvature(_handle, nearest, nearest->BackCnnctn());

        if(std::abs(kappa) > max_kappa_rad_)
        {
            constrainCurvature(_handle);
        }

        ///Impose Dynamic Temporal Constraints (Longitudinal Acceleration/Jerk) - Not implemented yet

    }

    void RRT::checkDone()
    {
        /// Make sure this graph isn't already complete `
        if(!cmplt)
        {
            /// Create Dummy end-node with time of zero
            Node *end = new Node();
            end->setPose(std::get<0>(dest_), std::get<1>(dest_), 0.0F, std::get<3>(dest_));

            /// Find the nearest Node to end 
            Node *nearest = findNearest(end, false);

            /// Update the destination time to be equal to the nearest node's time
            end->setPose(std::get<0>(dest_), std::get<1>(dest_), nearest->time(), std::get<3>(dest_));

            /// Grab the preceeding node to nearest
            Node *second_nearest = nearest->BackCnnctn();

            /// Stash these in case the heuristic fails and we need to revert
            pose_t original_pose_ = nearest->Pose();
            double original_back_edge_weight_ = nearest->backEdgeWeight();

            /// Calculate parameters between end and end/second_nearest
            double dist = calcDist(end, second_nearest, false);
            double angle = calcAngle(end, second_nearest);
            auto epsilon  = 0.0001F;

            /// Check if the node meets the spatial constraints (intentionally ignoring angle, just seeing if its close enough)
            if(std::abs(calcDist(end, nearest, false)) < max_dist_)
            {
                /// Heuristic: lets smooth out the end and make a nice "landing path" to the destination

                /// Check constraints against second nearest (doubling the constraints values (Assuming time is fine, if it werrent we wouldnt be in this method))
                if((std::abs(angle) < 2 * max_angle_rad_ || std::abs(std::abs(angle) - 2 * max_angle_rad_) <= epsilon) &&
                (std::abs(dist) < 2 * max_dist_ || std::abs(std::abs(dist) - 2 *max_dist_) <= epsilon) &&
                (std::abs(dist) > 2 * min_dist_ || std::abs(std::abs(dist) - 2 *min_dist_) <= epsilon))
                {   

                    auto nearest_x = ((end->xCrdnt() - second_nearest->xCrdnt()) / 2.0F) + second_nearest->xCrdnt();
                    auto nearest_y = ((end->yCrdnt() - second_nearest->yCrdnt()) / 2.0F) + second_nearest->yCrdnt();
                    auto nearest_heading = (angle - second_nearest->heading()) / 2.0F + second_nearest->heading();

                    /// Reposition nearest to be at the midpoint between the preceeding and the end node
                    nearest->setPose(nearest_x, nearest_y, nearest->time(), nearest_heading);
                    end->setPose(std::get<0>(dest_), std::get<1>(dest_), nearest->time(), angle);
                    

                    /// Double check that the second nearest node is still within constraints
                    if(!isOccupied(nearest))
                    {
                        /// Formally add the dummy destination node to the graph
                        pushNode(end);
                        addEdge(nearest, adjacencyList_.back());

                        /// list this node as an admissible destination node
                        destNodes.push_back(adjacencyList_.back());
                        admissible_ = true;
                        if(destNodes.size() >= max_admissible_)
                        {
                            cmplt = true;
                        }
                    }
                    else
                    {
                        /// Put humpty dumpty back together again
                        nearest->setPose(original_pose_);
                        nearest->setBackEdgeWeight(original_back_edge_weight_);
                    }
                }
                else{
                    /// Put humpty dumpty back together again
                    nearest->setPose(original_pose_);
                    nearest->setBackEdgeWeight(original_back_edge_weight_);
                }
            }
        }
        
    }

    double RRT::calcMengerCurvature(Node* _ref0, Node* _ref1, Node* _ref2)
    {
        /// Calculate the Menger Curature given three nodes
        double a = calcDist(_ref0, _ref1, false);
        double b = calcDist(_ref1, _ref2, false);
        double c = calcDist(_ref2, _ref0, false);

        double area = 0.25F * std::sqrt((a + (b + c)) * (c - (a - b)) * (c + (a - b)) * (a + (b - c)));

        if(area < 1e-10)
        {
            return 0.0F; // Points are collinear or too close together
        }

        double curvature = (4 * area) / (a * b * c);
        return curvature;
    }

    void RRT::constrainCurvature(Node* _handle)
    {
        /// Check that the handle has two sequential back connections via checking nullptr
        /// If we do not have 2 back connections, we're by the origin and only the max angle constraint applies
        if(_handle->BackCnnctn() || _handle->BackCnnctn()->BackCnnctn())
        {
            if (max_kappa_rad_ <= 0) {
                std::cerr << "Curvature must be positive." << std::endl;
                max_kappa_rad_ = std::abs(max_kappa_rad_);
            }

            /// Determine the sign of the curvature
            int curvature_sign = calcAngle(_handle, _handle->BackCnnctn()) >= 0 ? 1 : -1;

            /// Get coordinates
            double a_x = _handle->BackCnnctn()->BackCnnctn()->xCrdnt();
            double a_y = _handle->BackCnnctn()->BackCnnctn()->yCrdnt();
            double b_x = _handle->BackCnnctn()->xCrdnt();
            double b_y = _handle->BackCnnctn()->yCrdnt();

            // 1. Calculate the radius (R) from the Menger curvature (k).
            auto radius = 1.0 / max_kappa_rad_;

            // Calc the distance between points a and b
            double d = calcDist(_handle->BackCnnctn()->BackCnnctn(), _handle->BackCnnctn(), false);

            // 2. Find the center(s) of the circle.
            // The a-b midpoint 
            pose_t mid = std::make_tuple((a_x + b_x) / 2.0, (a_y + b_y) / 2.0, 0.0F, 0.0F);
            
            // The distance from the a-b midpoint to the circle center.
            double h = std::sqrt(std::pow(radius, 2) - std::pow(d / 2.0, 2));

            // The slope and perpendicular slope of a-b.
            double slope_ab = (b_y - a_y) / (b_x - a_x);
            double perp_slope_ab = -1.0 / slope_ab;

            // Handle vertical line a-b (undefined slope).
            double cx_offset, cy_offset;
            if (std::isinf(perp_slope_ab)) { // a-b is horizontal
                cx_offset = 0;
                cy_offset = h;
            } else if (std::isinf(slope_ab)) { // a-b is vertical
                cx_offset = h;
                cy_offset = 0;
            } else {
                double angle = std::atan(perp_slope_ab);
                cx_offset = h * std::cos(angle);
                cy_offset = h * std::sin(angle);
            }
            
            // Calculate the two possible circle centers.
            pose_t center1 = std::make_tuple(std::get<0>(mid) + cx_offset, std::get<1>(mid) + cy_offset, 0.0F, 0.0F);
            pose_t center2 = std::make_tuple(std::get<0>(mid) - cx_offset, std::get<1>(mid) - cy_offset, 0.0F, 0.0F);

            // 3. Solve for a third point (c).
            // The third point is any point on the circle. We can choose one, for example,
            // by rotating point 'a' around the center by some angle.
            // This example finds a point on the circle with the same y-coordinate as the center.
            
            // Solution from center1.
            double c1_x = std::get<0>(center1) + std::sqrt(std::pow(radius, 2) - std::pow(std::get<1>(mid) - std::get<1>(center1), 2));
            double c1_y = std::get<1>(center1) + (std::get<1>(mid)-std::get<1>(center1));

            // Solution from center2.
            double c2_x = std::get<0>(center2) + std::sqrt(std::pow(radius, 2) - std::pow(std::get<1>(mid) - std::get<1>(center2), 2));
            double c2_y = std::get<1>(center2) + (std::get<1>(mid)-std::get<1>(center2));

            if(curvature_sign >= 0)
            {
                alignHeadingToTangent(_handle, c1_x, c1_y);
                _handle->setPose(c1_x, c1_y, _handle->time(), _handle->heading());
            }
            else
            {
                alignHeadingToTangent(_handle, c2_x, c2_y);
                _handle->setPose(c2_x, c2_y, _handle->time(), _handle->heading());
            }
        }
    }

    void RRT::applyAbsoluteConstraints(Node* _handle, Node* _nearest)
    {
        double dist = calcDist(_handle, _nearest, true);
        double angle = calcAngle(_handle, _nearest);
        double tm = _handle->time();

        /// Apply Absolute Scaling Constraints 
        /// Ensure angle change is within limits relative to the previous node's heading/pose
        if((std::abs(angle) > max_angle_rad_))
        {
            angle = ((angle / std::abs(angle)) * max_angle_rad_) + _nearest->heading();
        }

        if(std::abs(dist) > max_dist_)
        {
            dist = (dist / std::abs(dist)) * max_dist_;
        }
        else if (std::abs(dist) < min_dist_)
        {
            dist = (dist / std::abs(dist)) * min_dist_;
        }

        /// Need to ensure time never runs backwards 
        if(tm < _nearest->time()) 
        {
            tm = _nearest->time() + 0.0001F;
        }
    
        else if(std::abs(tm - _nearest->time()) > max_interval_)
        {
            tm = _nearest->time() + max_interval_;
        }

        double x = (dist * std::cos(angle)) + _nearest->xCrdnt();
        double y = (dist * std::sin(angle)) + _nearest->yCrdnt();

        /// Update the Node with the new coordinates which meet absolute constraints
        _handle->setPose(x,y,tm, angle); 
        _handle->setBackEdgeWeight(dist); //Update this with Lat Acceleration
    }

    void RRT::alignHeadingToTangent(Node* _handle, double _center_x, double _center_y)
    {
        /// Calculate the tangent heading at the node given the center of curvature
        double delta_x = _handle->xCrdnt() - _center_x;
        double delta_y = _handle->yCrdnt() - _center_y;

        /// Tangent is perpendicular to radius vector
        double tangent_angle = std::atan2(delta_y, delta_x) + (M_PI / 2.0F);

        _handle->setPose(_handle->xCrdnt(), _handle->yCrdnt(), _handle->time(), tangent_angle);
    }

    double RRT::calcLongAccel(Node* _handle, Node* _ref)
    {
        /// Calculate the longitudinal acceleration based on back connection nodes
        double dist1 = calcDist(_handle, _ref, false);
        double dist2 = calcDist(_ref, _ref->BackCnnctn(), false);
        double delta_time1 = _handle->time() - _ref->time();
        double delta_time2 = _handle->time() - _ref->BackCnnctn()->time();

        if (delta_time1 > 0 && delta_time2 > 0)
        {
            double velocity1 = dist1 / delta_time1;
            double velocity2 = dist2 / delta_time2;
            return (velocity1 - velocity2) / delta_time1;
        }
        else if (delta_time1 > 0 && delta_time2 <= 0)
        {
            double velocity1 = dist1 / delta_time1;
            return velocity1 / delta_time1; // Assuming initial velocity is zero
        }
        else 
        {
            return 0;
        }
    }

    double RRT::calcLongJerk(Node* _handle, Node* _ref)
    {
        /// Calculate the longitudinal jerk based on back connection nodes
        double accel1 = calcLongAccel(_handle, _ref);
        double accel2 = calcLongAccel(_ref, _ref->BackCnnctn());
        double delta_time = _handle->time() - _ref->time();

        if (delta_time > 0)
        {
            return (accel1 - accel2) / delta_time;
        }
        else
        {
            return 0;
        }
    }

    bool RRT::isOccupied(Node *_handle)
    {
        static int sequential_check = 0;
        bool occupied = false;

        if (!occupancy_map_.has_value())
        {
            return false; /// No occupancy map provided, assume not occupied
        }

        /// Check if the back connection exists; if not, assume not occupied
        if (!_handle->BackCnnctn())
        {
            std::cerr << "Warning: Node has no back connection; cannot check edge occupancy. Terminating RRT" << std::endl;
            cmplt = true; // Terminate RRT build
            return true;
        }

        /// Iterate through the occupied cubes (voxels)
        for (const auto &occupancy : occupancy_map_.value())
        {
            /// Occupied bounding corner locations
            auto half_width = occupancy.second / 2.0F;
            auto x_min = std::get<0>(occupancy.first) - half_width;
            auto x_max = std::get<0>(occupancy.first) + half_width;
            auto y_min = std::get<1>(occupancy.first) - half_width;
            auto y_max = std::get<1>(occupancy.first) + half_width;
            auto time_min = std::get<2>(occupancy.first);
            auto time_max = std::get<2>(occupancy.first) + max_interval_;

            /// Check that the node itself is not in occupied space
            if (_handle->xCrdnt() >= x_min && _handle->xCrdnt() <= x_max &&
                _handle->yCrdnt() >= y_min && _handle->yCrdnt() <= y_max &&
                _handle->time() >= time_min && _handle->time() <= time_max)
            {
                occupied = true;
                std::cout << "Node in occupied space" << std::endl;
                break;
            }


            /// Check that the line segment (edge) between the node and the back link
            /// is not passing through occupied space
            auto line_x1 = _handle->BackCnnctn()->xCrdnt();  // Now safe
            auto line_y1 = _handle->BackCnnctn()->yCrdnt();
            auto line_z1 = _handle->BackCnnctn()->time();
            auto line_x2 = _handle->xCrdnt();
            auto line_y2 = _handle->yCrdnt();
            auto line_z2 = _handle->time();

            /// Line segment intersection check
            double t_min = 0.0;
            double t_max = 1.0;
            double dir_x = line_x2 - line_x1;
            double dir_y = line_y2 - line_y1;
            double dir_z = line_z2 - line_z1;

            /// Check each axis (x, y, z/time)
            for (int i = 0; i < 3; i++) {
                double min_val = (i == 0) ? x_min : (i == 1) ? y_min : time_min;
                double max_val = (i == 0) ? x_max : (i == 1) ? y_max : time_max;
                double p = (i == 0) ? line_x1 : (i == 1) ? line_y1 : line_z1;
                double d = (i == 0) ? dir_x : (i == 1) ? dir_y : dir_z;

                if (std::abs(d) < 1e-10) { /// Line parallel to axis
                    if (p < min_val || p > max_val) {
                        continue; /// Line is outside cube bounds on this axis
                    }
                } else {
                    double t1 = (min_val - p) / d;
                    double t2 = (max_val - p) / d;
                    if (t1 > t2) {
                        std::swap(t1, t2);
                    }
                    t_min = std::max(t_min, t1);
                    t_max = std::min(t_max, t2);
                    if (t_min > t_max) {
                        continue; /// No intersection on this axis
                    }
                }
            }
            /// Check if the valid t range overlaps with [0, 1]
            if (t_min <= t_max && t_min <= 1.0 && t_max >= 0.0) {
                occupied = true;
                break;
            }
        }
        if (occupied)
        {
            sequential_check++;
        }
        else
        {
            sequential_check = 0;
        }

        /// Return true if the node or its connecting edge is in occupied space
        if (sequential_check > sequential_check_limit_)
        {
            std::cout << "Greater than " << sequential_check_limit_ << " Sequential Nodes in Occupied Space: possible non-admissible trajectory." << std::endl;

            /// Exit RRT build if too many sequential occupied nodes
            cmplt = true;
        }
        return occupied;
    }

    bool RRT::isComplete() 
    {
        return cmplt;
    }

    bool RRT::isAdmissible() 
    {
        return admissible_;
    }

    void RRT::updateInitialHeading(double _initial_heading)
    {
        initial_heading_ = _initial_heading;
        adjacencyList_.front()->setPose(std::get<0>(origin_), std::get<1>(origin_), std::get<2>(origin_), initial_heading_);
    }

    void RRT::buildRRT()
    {
        pose_t output;

        while(!cmplt)
        {
            stepRRT();
        }
    }

    bool RRT::stepRRT()
    {
        pose_t output;
        static auto i = 0;

        if(cmplt)
        {
            std::cout << "RRT is already complete, no need to step." << std::endl;
            return cmplt;
        }

        /// Ensure Random Node is within permissible range / operating region 
        static double x_min = std::min(std::get<0>(range_a_), std::get<0>(range_b_));
        static double x_max = std::max(std::get<0>(range_a_), std::get<0>(range_b_));
        static double y_min = std::min(std::get<1>(range_a_), std::get<1>(range_b_));
        static double y_max = std::max(std::get<1>(range_a_), std::get<1>(range_b_));
        
        /// Generate random points within the bounded space 
        static std::random_device rand;
        static std::mt19937 gen(rand());
        static std::uniform_real_distribution<double> range_x(x_min, x_max);
        static std::uniform_real_distribution<double> range_y(y_min, y_max);
        static std::uniform_real_distribution<double> range_tm(0.0F, max_time_);

        auto tm = range_tm(gen);
        if (!dim_3D_)
        {
            tm = 0;
        }

        /// Looping to ensure step call results in a new node in non-occupied space
        auto occupied = true;
        while(occupied && !cmplt)
        {
            /// increment iteration count
            i++;

            /// Generate Random Node within permissible range 
            output = std::make_tuple(range_x(gen), range_y(gen), tm, 0.0F);

            /// Add the Node to the Graph 
            addNode(output, 0.0F);

            /// Apply Constraints, This effectively implements the RRT
            /// and also implements kinematic constraints 
            applyConstraints(adjacencyList_.back());

            /// Check if the new node is in occupied space
            if(isOccupied(adjacencyList_.back()))
            {
                /// If the node is occupied, delete it and try again
                deleteNode(adjacencyList_.back());
                i--; ///This iteration doesn't count because we didnt emplace a Node
            }
            else
            {
                occupied = false; /// Node is not occupied, placement is good
            }
        }

        /// Check to see if the new node is within range of the destination 
        checkDone();

        if(i > iteration_limit_)
        {
            std::cout << "Iteration Limit of " << i-1 << " Reached, Stopping RRT. " << std::endl;
            std::cout << "ITS POSSIBLE THERE IS NOT AN ADMISSIBLE SOLUTION" << std::endl;
            cmplt = true;
        }

        if(cmplt && i <= iteration_limit_)
        {
            std::cout << "RRT Completed with: " << adjacencyList_.size() << " Nodes and " << i << " Iterations." << std::endl;
        }

        return cmplt;
    }
}