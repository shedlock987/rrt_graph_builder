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
                 max_angle_rad_(1.05F) , max_dist_(1.0F), 
                 max_time_(0.0F) , dim_3D_(false)
    {
    }

    RRT::RRT(Node::coordinate_t _range_a, Node::coordinate_t _range_b,
        Node::coordinate_t _origin, Node::coordinate_t _dest,
        double _max_angle_rad, double _max_dist, double _max_time, bool _dim) :
                range_a_(_range_a), range_b_(_range_b), 
                origin_(_origin), dest_(_dest),
                max_angle_rad_(_max_angle_rad), max_dist_(_max_dist), 
                max_time_(_max_time) , dim_3D_(_dim)
    {
    }

    RRT::RRT(Node::coordinate_t _range_a, Node::coordinate_t _range_b,
        Node::coordinate_t _origin, Node::coordinate_t _dest,
        double _max_angle_rad, double _max_dist) :
                range_a_(_range_a), range_b_(_range_b), 
                origin_(_origin), dest_(_dest),
                max_angle_rad_(_max_angle_rad), max_dist_(_max_dist), 
                max_time_(0.0F) , dim_3D_(false)
    {
    }


    RRT::~RRT()
    {

    }

    Node* RRT::findNearest(Node::coordinate_t _handle)
    {
        int idx;
        int i = 0;
        double temp;
        double min = DBL_MAX;

        /* Need to optimize */
        for(const auto &iter : this->adjacencyList_)
        {
            temp = this->calcDist(_handle, iter->crdnts_);
            if(temp < min)
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

    double RRT::calcDist(Node::coordinate_t _handle, Node::coordinate_t _ref)
    {
        if(dim_3D_)
        {
            return std::sqrt(std::pow(std::get<0>(_handle) - std::get<0>(_ref), 2) + 
                             std::pow(std::get<1>(_handle) - std::get<1>(_ref), 2) +
                             std::pow(std::get<2>(_handle) - std::get<2>(_ref), 2));
        }
        else
        {
            return std::sqrt(std::pow(std::get<0>(_handle) - std::get<0>(_ref), 2) + 
                             std::pow(std::get<1>(_handle) - std::get<1>(_ref), 2));
        }
    }

    void RRT::applyConstraints(Node *_handle)
    {
        Node::coordinate_t temp = std::make_tuple(0.0F, 0.0F , 0.0F);
    }

    Node::coordinate_t RRT::genRandomCrdnt()
    {
        Node::coordinate_t output;
        bool valid = false;

        double x_min = std::min(std::get<0>(this->range_a_), std::get<0>(this->range_b_));
        double x_max = std::min(std::get<0>(this->range_a_), std::get<0>(this->range_b_));
        double y_min = std::min(std::get<1>(this->range_a_), std::get<1>(this->range_b_));
        double y_max = std::min(std::get<1>(this->range_a_), std::get<1>(this->range_b_));
        double tm_min = std::min(std::get<2>(this->range_a_), std::get<2>(this->range_b_));
        double tm_max = std::min(std::get<2>(this->range_a_), std::get<2>(this->range_b_));
        
        std::default_random_engine rand;
        std::uniform_real_distribution<double> range_x(x_min, x_max);
        std::uniform_real_distribution<double> range_y(y_min, y_max);
        std::uniform_real_distribution<double> range_tm(tm_min, tm_max);

        while(!valid)
        {
            // output = std::make_tuple(rand(range_x), rand(range_y), rand(range_tm));
            // FInd Nearest
            // Compare distance
            // If too close then regenerate
            // If not too close, apply constraints 
            // Probably need to recursively check until we can confirm not too close
            return output;
        }
    }

    void RRT::buildRRT()
    {

        
        while(!cmplt)
        {
            cmplt = true;
        }
    }

 }