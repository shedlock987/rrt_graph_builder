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
        
        /// Defaults
        setBoundaries(-5.0F, -5.0F, 5.0F, 5.0F, 10.0F);
        setOrigin(0.0F, -5.0F);
        updateDestination(5.0F, 5.0F);
        updateConstraints(1.05F, 1.0F, 0.5F, 0.0F);
        setDim3D(false);
        node_limit_ = 50;
    }

    RRT::RRT(std::vector<occupancy_t> _occupancy_map,
            double _range_a_x, double _range_a_y, double _range_b_x, double _range_b_y,
            double _origin_x, double _origin_y, double _dest_x, double _dest_y,
            double _max_angle_rad, double _max_dist, double _min_dist,
            double _max_interval, double _max_time, bool _dim, int _node_limit) : 
                occupancy_map_(_occupancy_map), 
                range_a_(std::make_tuple(_range_a_x, _range_a_y, 0.0F)),
                range_b_(std::make_tuple(_range_b_x, _range_b_y, _max_time)),
                origin_(std::make_tuple(_origin_x, _origin_y, 0.0F)),
                dest_(std::make_tuple(_dest_x, _dest_y, 0.0F)),
                max_angle_rad_(_max_angle_rad), max_dist_(_max_dist), min_dist_(_min_dist),
                max_interval_(_max_interval), max_time_(_max_time), dim_3D_(_dim), node_limit_(_node_limit)
    {
    }

    RRT::RRT(std::vector<occupancy_t> _occupancy_map,
        Node::coordinate_t _range_a, Node::coordinate_t _range_b,
        Node::coordinate_t _origin, Node::coordinate_t _dest,
        double _max_angle_rad, double _max_dist, double _min_dist, 
        double _max_interval, double _max_time, bool _dim, int _node_limit) :
                occupancy_map_(_occupancy_map), 
                range_a_(_range_a), range_b_(_range_b), 
                origin_(_origin), dest_(_dest),
                max_angle_rad_(_max_angle_rad), max_dist_(_max_dist), min_dist_(_min_dist), 
                max_interval_(_max_interval), max_time_(_max_time), dim_3D_(_dim), node_limit_(_node_limit)
    {
    }

    RRT::RRT(Node::coordinate_t _range_a, Node::coordinate_t _range_b,
        Node::coordinate_t _origin, Node::coordinate_t _dest,
        double _max_angle_rad, double _max_dist, double _min_dist, 
        double _max_interval, double _max_time, bool _dim, int _node_limit) :
                range_a_(_range_a), range_b_(_range_b), 
                origin_(_origin), dest_(_dest),
                max_angle_rad_(_max_angle_rad), max_dist_(_max_dist), min_dist_(_min_dist), 
                max_interval_(_max_interval), max_time_(_max_time), dim_3D_(_dim), node_limit_(_node_limit)
    {
    }

    RRT::RRT(double _range_a_x, double _range_a_y, double _range_b_x, double _range_b_y,
        double _origin_x, double _origin_y, double _dest_x, double _dest_y,
        double _max_angle_rad, double _max_dist, double _min_dist, 
        double _max_interval, double _max_time, bool _dim, int _node_limit) :
                range_a_(std::make_tuple(_range_a_x, _range_a_y, 0.0F)),
                range_b_(std::make_tuple(_range_b_x, _range_b_y, _max_time)),
                origin_(std::make_tuple(_origin_x, _origin_y, 0.0F)),
                dest_(std::make_tuple(_dest_x, _dest_y, 0.0F)),
                max_angle_rad_(_max_angle_rad), max_dist_(_max_dist), min_dist_(_min_dist),
                max_interval_(_max_interval), max_time_(_max_time), dim_3D_(_dim), 
                node_limit_(_node_limit)   
    {
    }


    RRT::~RRT() = default;

    void RRT::setBoundaries(Node::coordinate_t _range_a, Node::coordinate_t _range_b)
    {
        range_a_ = _range_a;
        range_b_ = _range_b;
        max_time_ = std::get<2>(_range_b); // Assuming the time component is in the z-axis
        if(std::get<2>(_range_a) <= 0.0F)
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
        range_a_ = std::make_tuple(_range_a_x, _range_a_y, 0.0F);
        range_b_ = std::make_tuple(_range_b_x, _range_b_y, _time_horizon);
        if(_time_horizon <= 0.0F)
        {
            setDim3D(true); // If the z-axis is not zero, we are in 3D
        }
        else
        {
            setDim3D(false); // Otherwise, we are in 2D
        }
    }

     
    void RRT::setOrigin(Node::coordinate_t _origin)
    {
        adjacencyList_.front()->setCrdnts(_origin);
    }
 
    void RRT::setOrigin(double _origin_x, double _origin_y)
    {
        adjacencyList_.front()->setCrdnts(_origin_x, _origin_y, 0.0F);
    }
          
    void RRT::updateDestination(Node::coordinate_t _dest)
    {
        dest_ = _dest;
    }

    void RRT::updateDestination(double _dest_x, double _dest_y)
    {
        dest_ = std::make_tuple(_dest_x, _dest_y, max_time_);
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

    void RRT::setOccupancyMap(std::vector<occupancy_t> &_occupancy_map)
    {
        occupancy_map_ = _occupancy_map;
    }

    Node* RRT::findNearest(Node *_handle)
    {
        int idx;
        int i = 0;
        double temp;
        double min = DBL_MAX;

        /// Need to optimize 
        for(const auto &iter : adjacencyList_)
        {
            temp = calcDist(_handle, iter);

            /// Make sure we're not comparing the handle to itself 
            if(temp < min && _handle != iter)
            {
                min = temp;
                idx = i;
                i++;
            }
            else
            {
                i++;
            }
        }
        return adjacencyList_.at(idx);
    }

    double RRT::calcDist(Node *_handle, Node *_ref)
    {
        if(dim_3D_)
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
        return std::atan2(delta_y, delta_x);
    }

    double RRT::calcKinematicEdge(Node *_handle, Node *_ref)
    {
        /// Calculate a composite back-edge weight based on kinematic cost 
        return 0;
    }

    bool RRT::checkConstraints(Node *_handle)
    {
        Node *nearest = findNearest(_handle);
        double dist = calcDist(_handle, nearest);
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
        Node *nearest = findNearest(_handle);
        double dist = calcDist(_handle, nearest);
        double angle = calcAngle(_handle, nearest);
        double tm = _handle->time();
        bool cnstrnts_met = checkConstraints(_handle);

        /// Apply Scaling Constraints 
        if(std::abs(angle) > max_angle_rad_)
        {
            angle = (angle / std::abs(angle)) * max_angle_rad_;
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
        if(tm <= nearest->time()) 
        {
            tm = nearest->time();
        }
        else if((tm - nearest->time()) > max_interval_)
        {
            tm = nearest->time() + max_interval_;
        }

        double x = (dist * std::cos(angle)) + nearest->xCrdnt();
        double y = (dist * std::sin(angle)) + nearest->yCrdnt();

        /// Update the Node with the new coordinates
        _handle->setCrdnts(x,y,tm);
        _handle->setBackEdgeWeight(dist); //Update this with Lat Acceleration

        cnstrnts_met = checkConstraints(_handle);
    }

    void RRT::checkDone()
    {
        /// Make sure this graph isn't already complete 
        if(!cmplt)
        {
            /// Insert Dummy end-node in graph
            addNode(dest_, 0.0F);
            Node *end = adjacencyList_.back();

            /// Find the nearest Node to end 
            Node *nearest = findNearest(end);

            /// Ensure our dummy end node is on the same time as the nearest node
            end->setCrdnts(end->xCrdnt(), end->yCrdnt(), nearest->time());

            /// Check if we're done 
            if(std::abs(calcDist(end, nearest)) < max_dist_ &&
            std::abs(calcAngle(end, nearest)) < max_angle_rad_ &&
            (end->time() - nearest->time()) <= max_interval_)
            {
                /// We're at the end, connect the end node to the nearest 
                cmplt = true;
                addEdge(nearest, end);
                endNode = end;
            }
            else
            {
                /// Destroy dummy end node
                deleteNode(end);
            }
        }  
    }

    bool RRT::isOccupied(Node *_handle)
    {
        if(!occupancy_map_.has_value())
        {
            return false; /// No occupancy map provided, assume not occupied
        }

        for(const auto &occupancy : occupancy_map_.value())
        {

            auto half_width = occupancy.second / 2.0F;
            auto x_min = std::get<0>(occupancy.first) - half_width;
            auto x_max = std::get<0>(occupancy.first) + half_width;
            auto y_min = std::get<1>(occupancy.first) - half_width;
            auto y_max = std::get<1>(occupancy.first) + half_width;
            auto time_min = std::get<2>(occupancy.first);
            auto time_max = std::get<2>(occupancy.first) + max_interval_;

            if(_handle->xCrdnt() >= x_min && _handle->xCrdnt() <= x_max &&
               _handle->yCrdnt() >= y_min && _handle->yCrdnt() <= y_max &&
               _handle->time() >= time_min && _handle->time() <= time_max)
            {
                return true; /// Node is in occupied space
            }
        }
        return false; /// Node is not in occupied space
    }

    void RRT::buildRRT()
    {
        Node::coordinate_t output;

        while(!cmplt)
        {
            stepRRT();
        }
        std::cout << "RRT Completed with: " << adjacencyList_.size() << " Nodes" << std::endl;
    }

    bool RRT::stepRRT()
    {
        Node::coordinate_t output;
        static auto i = 0;
        i++;

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
        while(occupied)
        {
            /// Generate Random Node within permissible range 
            output = std::make_tuple(range_x(gen), range_y(gen), tm);

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
            }
            else
            {
                occupied = false; /// Node is not occupied, placement is good
            }
        }

        /// Check to see if the new node is within range of the destination 
        checkDone();

        if(i > node_limit_)
        {
            std::cout << "Node Limit Reached, Stopping RRT" << std::endl;
            cmplt = true;
        }

        return cmplt;
    }

 }