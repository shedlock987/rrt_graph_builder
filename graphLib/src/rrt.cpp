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
    RRT::RRT() : range_a_(std::make_tuple(-5.0F, -5.0F, 0.0F)),
                 range_b_(std::make_tuple(5.0F, 5.0F, 0.0F)), 
                 origin_(std::make_tuple(0.0F, -5.0F, 0.0F)),
                 dest_(std::make_tuple(5.0F, 5.0F, 0.0F)),
                 max_angle_rad_(1.05F) , max_dist_(1.0F), min_dist_(0.5F),
                 max_time_(0.0F) , dim_3D_(false), node_limit_(50)
    {
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
            return std::sqrt(std::pow(_handle->getX() - _ref->getX(), 2) + 
                             std::pow(_handle->getY() - _ref->getY(), 2) +
                             std::pow(_handle->getTm() - _ref->getTm(), 2));
        }
        else
        {
            return std::sqrt(std::pow(_handle->getX() - _ref->getX(), 2) + 
                             std::pow(_handle->getY() - _ref->getY(), 2));
        }
    }

    double RRT::calcAngle(Node *_handle, Node *_ref)
    {
        double delta_x = _handle->getX() - _ref->getX();
        double delta_y = _handle->getY() - _ref->getY();
        return std::atan2(delta_y, delta_x);
    }

    double RRT::calcKinematicEdge(Node *_handle, Node *_ref)
    {
        /// Calculate a composite back-edge weight based on kinematic cost 
        return 0;
    }

    bool RRT::checkConstraints(Node *_handle)
    {
        /// Find Nearest existing Node 
        //Node *nearest = this->findNearest(_handle);
        Node *nearest = this->findNearest(_handle);
        double dist = this->calcDist(_handle, nearest);
        double angle = this->calcAngle(_handle, nearest);
        double tm = _handle->getTm();
        auto eplison = 0.0001F;

        if((std::abs(angle) < this->max_angle_rad_ || std::abs(std::abs(angle) - this->max_angle_rad_) <= eplison) &&
           (std::abs(dist) < this->max_dist_ || std::abs(std::abs(dist) - this->max_dist_) <= eplison) &&
           (std::abs(dist) > this->min_dist_ || std::abs(std::abs(dist) - this->min_dist_) <= eplison) &&
           (tm >= nearest->getTm() || (std::fabs(tm - nearest->getTm() <= eplison))) &&
           (tm <= (nearest->getTm() + this->max_interval) || std::abs(nearest->getTm() + this->max_interval - this->max_interval) <= eplison)
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
        double tm = _handle->getTm();
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
        if(tm <= nearest->getTm()) 
        {
            tm = nearest->getTm();
        }
        else if((tm - nearest->getTm()) > this->max_interval)
        {
            tm = nearest->getTm() + this->max_interval;
        }

        double x = (dist * std::cos(angle)) + nearest->getX();
        double y = (dist * std::sin(angle)) + nearest->getY();

        /// Update the Node with the new coordinates
        _handle->setCord(x,y,tm);
        _handle->back_edge_weight_ = dist; //Update this with Lat Acceleration

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

            //find nearest doesnt have check on itelf

            /// Find the nearest Node to end 
            Node *nearest = this->findNearest(end);

            /// Check if we're done 
            if(std::abs(this->calcDist(end, nearest)) < this->max_dist_ &&
            std::abs(this->calcAngle(end, nearest)) < this->max_angle_rad_ &&
            (end->getTm() - nearest->getTm()) <= this->max_interval)
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
        bool valid = false;

        /// Ensure Random Node is within permissible range / operating region 
        double x_min = std::min(std::get<0>(this->range_a_), std::get<0>(this->range_b_));
        double x_max = std::max(std::get<0>(this->range_a_), std::get<0>(this->range_b_));
        double y_min = std::min(std::get<1>(this->range_a_), std::get<1>(this->range_b_));
        double y_max = std::max(std::get<1>(this->range_a_), std::get<1>(this->range_b_));
        
        /// Generate random points within the bounded space 
        std::random_device rand;
        std::mt19937 gen(rand());
        std::uniform_real_distribution<double> range_x(x_min, x_max);
        std::uniform_real_distribution<double> range_y(y_min, y_max);
        std::uniform_real_distribution<double> range_tm(0.0F, this->max_time_);

        int i = 0;
        std::cout << "Building RRT!\n";
        while(!this->cmplt)
        {
            auto tm = range_tm(gen);
            if (this->dim_3D_)
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
            i++;

            //Temporary
            if(i > node_limit_)
                this->cmplt = true;
        }
        std::cout << "RRT Complete with " << i << " Nodes\n";
    }

 }