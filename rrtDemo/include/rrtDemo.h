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
#include <vector>
#include "rrt.h"

namespace rrt
{
class VisRRT
{
public:
    /**
     * @brief Constructs a new VisRRT object
     * @note Default constructor, initializes the RRT with default parameters
     * - range_a_x: -5.0, range_a_y: 0.0
     * - range_b_x: 5.0, range_b_y: 5.0
     */
    VisRRT();

    /**
     * @brief Constructs a new VisRRT object
     * @note Constructor using coordinate_t for boundaries/origin/destination (optimized for Python binding)
     * @param _range_a Lower left corner of the operating region (x, y, time)
     * @param _range_b Upper right corner of the operating region (x, y, time)
     * @param _origin Origin coordinates (x, y, time)
     * @param _dest Destination coordinates (x, y, time)
     * @param _max_angle_rad Constraint: maximum permitted angle between 2 Nodes
     * @param _max_dist Constraint: maximum permitted distance between 2 Nodes
     * @param _min_dist Constraint: minimum permitted distance between 2 Nodes
     * @param _max_interval Constraint: maximum time interval between 2 Nodes
     * @param _max_time Absolute temporal boundary/limit for the RRT
     * @param _dim_3D Specifies if the RRT is 2D or 3D
     * @param _node_limit Maximum number of nodes permitted in the RRT
     */
    VisRRT(Node::coordinate_t _range_a, Node::coordinate_t _range_b,
           Node::coordinate_t _origin, Node::coordinate_t _dest,
           double _max_angle_rad, double _max_dist,
           double _min_dist, double _max_interval,
           double _max_time, bool _dim_3D, int _node_limit);

    /**
     * @brief Destroys the VisRRT object
     */
    ~VisRRT();

    void buildRRT();

    bool stepRRT();

    /**
     * @brief Initializes the RRT with coordinate_t for boundaries/origin/destination
     * @param _range_a Lower left corner of the operating region (x, y, time)
     * @param _range_b Upper right corner of the operating region (x, y, time)
     * @param _origin Origin coordinates (x, y, time)
     * @param _dest Destination coordinates (x, y, time)
     * @param _max_angle_rad Constraint: maximum permitted angle between 2 Nodes
     * @param _max_dist Constraint: maximum permitted distance between 2 Nodes
     * @param _min_dist Constraint: minimum permitted distance between 2 Nodes
     * @param _max_interval Constraint: maximum time interval between 2 Nodes
     * @param _max_time Absolute temporal boundary/limit for the RRT
     * @param _dim_3D Specifies if the RRT is 2D or 3D
     * @param _node_limit Maximum number of nodes permitted in the RRT
     */
    void initializeRRT(
        Node::coordinate_t _range_a, Node::coordinate_t _range_b,
        Node::coordinate_t _origin, Node::coordinate_t _dest,
        double _max_angle_rad, double _max_dist,
        double _min_dist, double _max_interval,
        double _max_time, bool _dim_3D, int _node_limit);

    /**
     * @brief Sets the boundaries of where the RRT will build (coordinate_t overload)
     * @param _range_a Lower left corner of the operating region (x, y, time)
     * @param _range_b Upper right corner of the operating region (x, y, time)
     */
    void setBoundaries(Node::coordinate_t _range_a, Node::coordinate_t _range_b);

    /**
     * @brief Sets the boundaries of where the RRT will build (double overload)
     * @param _range_a_x X Coordinate Lower Left Corner of the Operating Region
     * @param _range_a_y Y Coordinate Lower Left Corner of the Operating Region
     * @param _range_b_x X Coordinate Upper Right Corner of the Operating Region
     * @param _range_b_y Y Coordinate Upper Right Corner of the Operating Region
     * @param _time_horizon Maximum time horizon for the RRT
     */
    void setBoundaries(double _range_a_x, double _range_a_y,
                       double _range_b_x, double _range_b_y, double _time_horizon);

    /**
     * @brief Updates/sets the geometric location of the origin node of the RRT (coordinate_t overload)
     * @param _origin The coordinate of the origin
     */
    void setOrigin(Node::coordinate_t _origin);

    /**
     * @brief Updates/sets the geometric location of the origin node of the RRT (double overload)
     * @param _origin_x The x coordinate of the origin
     * @param _origin_y The y coordinate of the origin
     * @param _origin_time The time coordinate of the origin
     */
    void setOrigin(double _origin_x, double _origin_y, double _origin_time);

    /**
     * @brief Updates/sets the geometric location of the destination node of the RRT (coordinate_t overload)
     * @param _dest The coordinate of the destination
     */
    void updateDestination(Node::coordinate_t _dest);

    /**
     * @brief Updates/sets the geometric location of the destination node of the RRT (double overload)
     * @param _dest_x The x coordinate of the destination
     * @param _dest_y The y coordinate of the destination
     * @param _dest_time The time coordinate of the destination
     */
    void updateDestination(double _dest_x, double _dest_y, double _dest_time);

    /**
     * @brief Updates/sets the geometric constraints for next-node placement
     * @param _max_angle_rad maximum angle allowed between two nodes in radians
     * @param _max_dist maximum distance allowed between two nodes
     * @param _min_dist minimum distance allowed between two nodes
     * @param _max_interval maximum time interval allowed between two nodes
     */
    void updateConstraints(double _max_angle_rad, double _max_dist, double _min_dist, double _max_interval);

    /**
     * @brief Sets the RRT to be 2D or 3D
     * @param _dim_3D If true, the RRT will be 3D, otherwise it will be 2D
     */
    void setDim3D(bool _dim_3D);

    /**
     * @brief Updates the Node Limit for the RRT
     * @param _node_limit The maximum number of nodes permitted in the RRT
     */
    void setNodeLimit(int _node_limit);

    /**
     * @brief Sets the occupancy map for the RRT (Python-friendly overload)
     * @param _occp_coords List of [x, y] coordinates for occupied voxels
     * @param _occp_widths List of widths for each voxel
     * @param _occp_interval List of time intervals for each voxel
     */
    void setOccupancyMap(std::vector<std::vector<double>> _occp_coords,
                         std::vector<double> _occp_widths,
                         std::vector<double> _occp_interval);

    int getNodeCount();

    bool isComplete();

    Node* getNodeAt(int idx);

private:
    std::unique_ptr<RRT> rrt_; /**< Pointer to the RRT object */
};
}
#endif /* rrtDemo_H_ */