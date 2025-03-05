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
    RRT::RRT(Node::coordinate_t _odd_bound_a, Node::coordinate_t _odd_bound_b)
    {
        this->odd_bound_a_ = _odd_bound_a;
        this->odd_bound_b_ = _odd_bound_b;
    }

    RRT::~RRT()
    {

    }

    void RRT::buildRRT()
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
                             std::pow(std::get<0>(_handle) - std::get<1>(_ref), 2) +
                             std::pow(std::get<0>(_handle) - std::get<2>(_ref), 2));
        }
        else
        {
            return std::sqrt(std::pow(std::get<0>(_handle) - std::get<0>(_ref), 2) + 
                             std::pow(std::get<0>(_handle) - std::get<1>(_ref), 2));
        }
    }

    Node::coordinate_t RRT::applyConstraints()
    {
        Node::coordinate_t temp = std::make_tuple(0.0F, 0.0F , 0.0F);
        return temp;
    }

 }