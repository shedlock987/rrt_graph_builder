
#include "rrtDemo.h"
#include "graph.h"
#include "rrt.h"
#include <iostream>
#include <boost/python.hpp>
#include <vector>

using namespace boost::python;

namespace rrt
{
    VisRRT::VisRRT() : rrt_(new RRT(
        -5.0, 0.0, 5.0, 5.0,   // _range_a_x, _range_a_y, _range_b_x, _range_b_y
        0.0, 0.0, 5.0, 5.0,    // _origin_x, _origin_y, _dest_x, _dest_y
        0.8, 1.0, 0.5, 2.0,    // _max_angle_rad, _max_dist, _min_dist, _max_interval
        10.0,                  // _max_time
        true,                  // _dim
        10000                  // _node_limit
    ))
    {      
    }
        VisRRT::VisRRT(double _range_a_x, double _range_a_y, 
        double _range_b_x, double _range_b_y,
        double _origin_x, double _origin_y, 
        double _dest_x, double _dest_y,
        double _max_angle_rad, double _max_dist, 
        double _min_dist, double _max_interval,
        double _max_time, bool _dim_3D, int _node_limit,
        std::vector<std::vector<double>> _occp_coords, 
        std::vector<double> _occp_widths,  
        std::vector<double> _occp_interval) : rrt_(new RRT(
        _range_a_x, _range_a_y, _range_b_x, _range_b_y,
        _origin_x, _origin_y, _dest_x, _dest_y,
        _max_angle_rad, _max_dist, _min_dist, _max_interval,
        _max_time, _dim_3D, _node_limit))
    {      
        setOccupancyMap(_occp_coords, _occp_widths, _occp_interval);
    }

    VisRRT::~VisRRT() = default;

    void VisRRT::buildRRT() {
        rrt_->buildRRT();
    }

    bool VisRRT::stepRRT() {
        return rrt_->stepRRT();
    }

    void VisRRT::initializeRRT(
        double _range_a_x, double _range_a_y, 
        double _range_b_x, double _range_b_y,
        double _origin_x, double _origin_y, 
        double _dest_x, double _dest_y,
        double _max_angle_rad, double _max_dist, 
        double _min_dist, double _max_interval,
        double _max_time, bool _dim_3D, int _node_limit)
    {
        rrt_->setBoundaries(_range_a_x, _range_a_y, _range_b_x, _range_b_y, _max_time);
        rrt_->setOrigin(_origin_x, _origin_y);
        rrt_->updateDestination(_dest_x, _dest_y);
        rrt_->updateConstraints(_max_angle_rad, _max_dist, _min_dist, _max_interval);
        rrt_->setDim3D(_dim_3D);
        rrt_->setNodeLimit(_node_limit);
    }

    void VisRRT::setOccupancyMap(
        std::vector<std::vector<double>> _occp_coords, 
        std::vector<double> _occp_widths,  
        std::vector<double> _occp_interval) {
        
        std::vector<RRT::occupancy_t> occupancy_map;
        for (size_t i = 0; i < _occp_coords.size(); ++i) {
            /*
            if (_occp_coords[i].size() != 2) {
                throw std::invalid_argument("Invalid occupancy map data");
            } */
            RRT::occupancy_t occ;
            occ.first = std::make_tuple(_occp_coords[i][0], _occp_coords[i][1], _occp_interval[i]);
            occ.second = _occp_widths[i];
            occupancy_map.push_back(occ);
        }
        /// Set the occupancy map in the RRT instance
        rrt_->setOccupancyMap(occupancy_map);
    }

    int VisRRT::getNodeCount() { 
        return rrt_->adjacencyList_.size(); 
    }

    bool VisRRT::isComplete() {
        return rrt_->isComplete();
    }
}

BOOST_PYTHON_MODULE(rrtDemo) {
    class_<rrt::VisRRT, boost::noncopyable>("RRT")
        .def("buildRRT", &rrt::VisRRT::buildRRT)
        .def("stepRRT", &rrt::VisRRT::stepRRT)
        .def("initializeRRT", &rrt::VisRRT::initializeRRT)
        .def("setOccupancyMap", static_cast<void (rrt::VisRRT::*)(std::vector<std::vector<double>>, std::vector<double>, std::vector<double>)>(&rrt::VisRRT::setOccupancyMap))
        .def("isComplete", &rrt::VisRRT::isComplete)
        .def("getNodeCount", &rrt::VisRRT::getNodeCount);
}

int main() {


    rrt::RRT myRRT(
        -5.0, 0.0, 5.0, 5.0,   // _range_a_x, _range_a_y, _range_b_x, _range_b_y
        0.0, 0.0, 5.0, 5.0,    // _origin_x, _origin_y, _dest_x, _dest_y
        0.8, 1.0, 0.5, 2.0,    // _max_angle_rad, _max_dist, _min_dist, _max_interval
        10.0,                  // _max_time
        true,                  // _dim
        10000                  // _node_limit
    );

    rrt::VisRRT testViz;
    testViz.initializeRRT(
        0.0, 0.0, 10.0, 10.0,   // _range_a_x, _range_a_y, _range_b_x, _range_b_y
        0.0, 0.0, 10.0, 10.0,    // _origin_x, _origin_y, _dest_x, _dest_y
        0.3, 1.0, 0.2, 1.0,    // _max_angle_rad, _max_dist, _min_dist, _max_interval
        100.0,                  // _max_time
        true,                  // _dim_3D
        10000                  // _node_limit
    );

    myRRT.buildRRT();

    while (!testViz.stepRRT()) 
    {
    }

    return 0;
}
