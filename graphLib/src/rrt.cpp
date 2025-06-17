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
    RRT::RRT(const std::optional<std::vector<std::vector<std::vector<double>>>> &_occupancy_map) : 
                occupancy_map_(_occupancy_map)
    {
        if(_occupancy_map.has_value())
        {
            /// Find Min and Max of the occupancy map
            auto min_x = std::numeric_limits<double>::max();
            auto min_y = std::numeric_limits<double>::max();
            auto min_t = std::numeric_limits<double>::max();
            auto max_x = std::numeric_limits<double>::min();
            auto max_y = std::numeric_limits<double>::min();
            auto max_t = std::numeric_limits<double>::min();
            /// Iterate through the outer vector (representing the x-dimension)
            for (const auto& vector_y : _occupancy_map.value()) {
                /// Iterate through the middle vector (representing the y-dimension)
                for (const auto& vector_t : vector_y) {
                    /// Iterate through the inner vector (representing the time-dimension)
                    for (const auto& value : vector_t) {
                        /// Update max/min values
                        max_x = std::max(max_x, value);
                        max_y = std::max(max_y, value);
                        max_t = std::max(max_t, value);
                        min_x = std::min(min_x, value);
                        min_y = std::min(min_y, value); 
                        min_t = std::min(min_t, value);
                    }
                }
            }

            /// Time cannot be negative
            max_t = max_t - min_t;

            this->setBoundaries(min_x, min_y, max_x, max_y, max_t);


        }
        else
        {
            this->setBoundaries(-5.0F, -5.0F, 5.0F, 5.0F, 10.0F);
            this->setOrigin(0.0F, -5.0F);
            this->updateDestination(5.0F, 5.0F);
            this->updateConstraints(1.05F, 1.0F, 0.5F, 0.0F);
            this->setDim3D(false);
            this->node_limit_ = 50;
        }
    }

    RRT::RRT(Node::coordinate_t _range_a, Node::coordinate_t _range_b,
        Node::coordinate_t _origin, Node::coordinate_t _dest,
        double _max_angle_rad, double _max_dist, double _min_dist, double _max_time, bool _dim, int _node_limit) :
                range_a_(_range_a), range_b_(_range_b), 
                origin_(_origin), dest_(_dest),
                max_angle_rad_(_max_angle_rad), max_dist_(_max_dist), min_dist_(_min_dist), 
                max_time_(_max_time) , dim_3D_(_dim), node_limit_(_node_limit)
    {
    }

    RRT::RRT(Node::coordinate_t _range_a, Node::coordinate_t _range_b,
        Node::coordinate_t _origin, Node::coordinate_t _dest,
        double _max_angle_rad, double _max_dist, double _min_dist, int _node_limit) :
                range_a_(_range_a), range_b_(_range_b), 
                origin_(_origin), dest_(_dest),
                max_angle_rad_(_max_angle_rad), max_dist_(_max_dist), min_dist_(_min_dist), 
                max_time_(0.0F) , dim_3D_(false), node_limit_(_node_limit)
    {
    }


    RRT::~RRT()
    {

    }

    void RRT::setBoundaries(Node::coordinate_t _range_a, Node::coordinate_t _range_b)
    {
        this->range_a_ = _range_a;
        this->range_b_ = _range_b;
        this->max_time_ = std::get<2>(_range_b); // Assuming the time component is in the z-axis
        if(std::get<2>(_range_a) <= 0.0F)
        {
            this->setDim3D(true); // If the z-axis is not zero, we are in 3D
        }
        else
        {
            this->setDim3D(false); // Otherwise, we are in 2D
        }
    }

    void RRT::setBoundaries(double _range_a_x, double _range_a_y, 
                           double _range_b_x, double _range_b_y, double _time_horizon)
    {
        this->max_time_ = _time_horizon;
        this->range_a_ = std::make_tuple(_range_a_x, _range_a_y, 0.0F);
        this->range_b_ = std::make_tuple(_range_b_x, _range_b_y, _time_horizon);
        if(_time_horizon <= 0.0F)
        {
            this->setDim3D(true); // If the z-axis is not zero, we are in 3D
        }
        else
        {
            this->setDim3D(false); // Otherwise, we are in 2D
        }
    }

     
    void setOrigin(Node::coordinate_t _origin)
    {
        //this->adjacencyList_.front()->setCrdnts(_origin);
    }
 
    void RRT::setOrigin(double _origin_x, double _origin_y)
    {
        //this->adjacencyList_.frontc()->setCrdnts(_origin_x, _origin_y, 0.0F);
    }
          
    void RRT::updateDestination(Node::coordinate_t _dest)
    {
        this->dest_ = _dest;
    }

    void RRT::updateDestination(double _dest_x, double _dest_y)
    {
        this->dest_ = std::make_tuple(_dest_x, _dest_y, this->max_time_);
    }

    void RRT::updateConstraints(double _max_angle_rad, double _max_dist, double _min_dist, double _max_interval)
    {
        this->max_angle_rad_ = _max_angle_rad;
        this->max_dist_ = _max_dist;
        this->min_dist_ = _min_dist;
        this->max_interval_ = _max_interval;
    }
 
    void RRT::setDim3D(bool _dim_3D)
    {
        this->dim_3D_ = _dim_3D;
    }

    Node* RRT::findNearest(Node *_handle)
    {
        int idx;
        int i = 0;
        double temp;
        double min = DBL_MAX;

        /// Need to optimize 
        for(const auto &iter : this->adjacencyList_)
        {
            temp = this->calcDist(_handle, iter);

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
        return this->adjacencyList_.at(idx);
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
        Node *nearest = this->findNearest(_handle);
        double dist = this->calcDist(_handle, nearest);
        double angle = this->calcAngle(_handle, nearest);
        double tm = _handle->time();
        auto eplison = 0.0001F;

        if((std::abs(angle) < this->max_angle_rad_ || std::abs(std::abs(angle) - this->max_angle_rad_) <= eplison) &&
           (std::abs(dist) < this->max_dist_ || std::abs(std::abs(dist) - this->max_dist_) <= eplison) &&
           (std::abs(dist) > this->min_dist_ || std::abs(std::abs(dist) - this->min_dist_) <= eplison) &&
           (tm >= nearest->time() || (std::fabs(tm - nearest->time() <= eplison))) &&
           (tm <= (nearest->time() + this->max_interval_) || std::abs(nearest->time() + this->max_interval_ - this->max_interval_) <= eplison)
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
        Node *nearest = this->findNearest(_handle);
        double dist = this->calcDist(_handle, nearest);
        double angle = this->calcAngle(_handle, nearest);
        double tm = _handle->time();
        bool cnstrnts_met = this->checkConstraints(_handle);

        /// Apply Scaling Constraints 
        if(std::abs(angle) > this->max_angle_rad_)
        {
            angle = (angle / std::abs(angle)) * this->max_angle_rad_;
        }

        if(std::abs(dist) > this->max_dist_)
        {
            dist = (dist / std::abs(dist)) * this->max_dist_;
        }
        else if (std::abs(dist) < this->min_dist_)
        {
            dist = (dist / std::abs(dist)) * this->min_dist_;
        }

        /// Need to ensure time never runs backwards 
        if(tm <= nearest->time()) 
        {
            tm = nearest->time();
        }
        else if((tm - nearest->time()) > this->max_interval_)
        {
            tm = nearest->time() + this->max_interval_;
        }

        double x = (dist * std::cos(angle)) + nearest->xCrdnt();
        double y = (dist * std::sin(angle)) + nearest->yCrdnt();

        /// Update the Node with the new coordinates
        _handle->setCrdnts(x,y,tm);
        _handle->setBackEdgeWeight(dist); //Update this with Lat Acceleration

        cnstrnts_met = this->checkConstraints(_handle);
    }

    void RRT::checkDone()
    {
        /// Make sure this graph isn't already complete 
        if(!this->cmplt)
        {
            /// Insert Dummy end-node in graph
            this->addNode(this->dest_, 0.0F);
            Node *end = this->adjacencyList_.back();

            /// Find the nearest Node to end 
            Node *nearest = this->findNearest(end);

            /// Ensure our dummy end node is on the same time as the nearest node
            end->setCrdnts(end->xCrdnt(), end->yCrdnt(), nearest->time());

            /// Check if we're done 
            if(std::abs(this->calcDist(end, nearest)) < this->max_dist_ &&
            std::abs(this->calcAngle(end, nearest)) < this->max_angle_rad_ &&
            (end->time() - nearest->time()) <= this->max_interval_)
            {
                /// We're at the end, connect the end node to the nearest 
                this->cmplt = true;
                this->addEdge(nearest, end);
                this->endNode = end;
            }
            else
            {
                /// Destroy dummy end node
                this->deleteNode(end);
            }
        }  
    }

    void RRT::buildRRT()
    {
        Node::coordinate_t output;

        while(!this->cmplt)
        {
            this->stepRRT();
        }
        std::cout << "RRT Completed with: " << this->adjacencyList_.size() << " Nodes" << std::endl;
    }

    bool RRT::stepRRT()
    {
        Node::coordinate_t output;
        static auto i = 0;
        i++;

        /// Ensure Random Node is within permissible range / operating region 
        static double x_min = std::min(std::get<0>(this->range_a_), std::get<0>(this->range_b_));
        static double x_max = std::max(std::get<0>(this->range_a_), std::get<0>(this->range_b_));
        static double y_min = std::min(std::get<1>(this->range_a_), std::get<1>(this->range_b_));
        static double y_max = std::max(std::get<1>(this->range_a_), std::get<1>(this->range_b_));
        
        /// Generate random points within the bounded space 
        static std::random_device rand;
        static std::mt19937 gen(rand());
        static std::uniform_real_distribution<double> range_x(x_min, x_max);
        static std::uniform_real_distribution<double> range_y(y_min, y_max);
        static std::uniform_real_distribution<double> range_tm(0.0F, this->max_time_);

        auto tm = range_tm(gen);
        if (!this->dim_3D_)
        {
            tm = 0;
        }

        /// Generate Random Node within permissible range 
        output = std::make_tuple(range_x(gen), range_y(gen), tm);

        /// Add the Node to the Graph 
        this->addNode(output, 0.0F);

        /// Apply Constraints, This effectively implements the RRT
        /// and also implements kinematic constraints 
        this->applyConstraints(this->adjacencyList_.back());

        /// Check to see if the new node is within range of the destination 
        this->checkDone();

        if(i > this->node_limit_)
        {
            std::cout << "Node Limit Reached, Stopping RRT" << std::endl;
            this->cmplt = true;
        }

        return this->cmplt;
    }

 }