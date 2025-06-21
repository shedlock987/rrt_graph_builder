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
 * @file rrtDemo.h
 * @brief Visualization of Rapidly-exploring Random Tree (RRT) algorithm
 * @author Ryan Shedlock <rmshedlock@gmail.com>
 * @version 1.0
 */
#ifndef rrtDemo_H_
#define rrtDemo_H_

#include <memory>
#include "rrt.h"

namespace rrt
{
    class VisRRT
    {
        public:
            /**
             * @brief   Constructs a new VisRRT object
             * @note    Default constructor, initializes the RRT with default parameters
             *          - range_a_x: -5.0, range_a_y: 0.0
             *          - range_b_x: 5.0, range_b_y: 5.0            
             */
            VisRRT();

            /**
             * @brief   Constructs a new VisRRT object
             * @note    Primary constructor used in the Python interface       
             */
            VisRRT(double _range_a_x, double _range_a_y, 
                double _range_b_x, double _range_b_y,
                double _origin_x, double _origin_y, 
                double _dest_x, double _dest_y,
                double _max_angle_rad, double _max_dist, 
                double _min_dist, double _max_interval,
                double _max_time, bool _dim_3D, int _node_limit,
                std::vector<std::vector<double>> _occp_coords, 
                std::vector<double> _occp_widths,  
                std::vector<double> _occp_interval);

            /**
             * @brief   Destroys the VisRRT object
             */
            ~VisRRT();

            void buildRRT();
            bool stepRRT();
            void initializeRRT(
                double _range_a_x, double _range_a_y, 
                double _range_b_x, double _range_b_y,
                double _origin_x, double _origin_y, 
                double _dest_x, double _dest_y,
                double _max_angle_rad, double _max_dist, 
                double _min_dist, double _max_interval,
                double _max_time, bool _dim_3D, int _node_limit);
            void setOccupancyMap(std::vector<std::vector<double>> _occp_coords, 
                                 std::vector<double> _occp_widths, 
                                 std::vector<double> _occp_interval);
            void setOccupancyMap(std::vector<RRT::occupancy_t> &_occupancy_map);
            int getNodeCount();
        private:
        std::unique_ptr<RRT> rrt_; /**< Pointer to the RRT object */
            

    };
}
#endif /* rrtDemo_H_ */