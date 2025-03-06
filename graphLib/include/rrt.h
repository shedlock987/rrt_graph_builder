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

namespace rrt
{
    class RRT : public Graph
    {
        private:
        double max_angle_rad_;
        double max_dist_;
        double max_time_;
        bool dim_3D_ = false;
        bool cmplt = false;

        Node::coordinate_t range_a_; 
        Node::coordinate_t range_b_;
        Node::coordinate_t origin_;
        Node::coordinate_t dest_;

        Node* findNearest(Node::coordinate_t _handle);
        double calcAngle(Node::coordinate_t _handle, Node::coordinate_t _ref);
        double calcDist(Node::coordinate_t _handle, Node::coordinate_t _ref);
        void applyConstraints(Node *_handle);
        Node::coordinate_t genRandomCrdnt();

        public:
        RRT();
        RRT(Node::coordinate_t _range_a, Node::coordinate_t _range_b,
            Node::coordinate_t _origin, Node::coordinate_t _dest,
            double _max_angle_rad, double _max_dist, double _max_time, bool _dim);
        RRT(Node::coordinate_t _range_a, Node::coordinate_t _range_b,
            Node::coordinate_t _origin, Node::coordinate_t _dest,
            double _max_angle_rad, double _max_dist);
        ~RRT();
        void buildRRT();
    };
}

#endif /* RRT_H_ */