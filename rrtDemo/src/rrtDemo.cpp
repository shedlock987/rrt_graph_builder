
#include "rrtDemo.h"
#include "graph.h"
#include "rrt.h"
#include <iostream>
#include <boost/python.hpp>

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
        this->setOccupancyMap(_occp_coords, _occp_widths, _occp_interval);
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
        double _max_time, bool _dim_3D, int _node_limit) {
    }

    void VisRRT::setOccupancyMap(
        std::vector<std::vector<double>> _occp_coords, 
        std::vector<double> _occp_widths,  
        std::vector<double> _occp_interval) {
        
        std::vector<RRT::occupancy_t> occupancy_map;
        for (size_t i = 0; i < _occp_coords.size(); ++i) {
            if (_occp_coords[i].size() != 2) {
                throw std::invalid_argument("Invalid occupancy map data");
            }
            RRT::occupancy_t occ;
            occ.first = std::make_tuple(_occp_coords[i][0], _occp_coords[i][1], _occp_interval[i]);
            occ.second = _occp_widths[i];
            occupancy_map.push_back(occ);
        }
        /// Set the occupancy map in the RRT instance
        rrt_->setOccupancyMap(occupancy_map);
    }
}

BOOST_PYTHON_MODULE(display_RRT) {
    class_<rrt::VisRRT, boost::noncopyable>("RRT")
        .def("buildRRT", &rrt::VisRRT::buildRRT)
        .def("stepRRT", &rrt::VisRRT::stepRRT)
        .def("initializeRRT", &rrt::VisRRT::initializeRRT)
        .def("setOccupancyMap", &rrt::VisRRT::setOccupancyMap);
}

int main() {
    rrt::Node::coordinate_t origin(0.0F,0.0F,0.0F);
    rrt::Node::coordinate_t lowerLeft(-5.0F, 0.0F, 0.0F);
    rrt::Node::coordinate_t upperRight(5.0F, 5.00, 0.0F);
    rrt::Graph testGraph(3.0F,3.0F);

    rrt::RRT testRRT(lowerLeft, upperRight, origin, upperRight, 1.05F, 1.0F, 0.5F, 0.2F, 10.0F, false, 50);
    
    testRRT.buildRRT();

    //rrt::VisRRT displayRRT;
    
    //displayRRT.rrt_pimpl_->buildRRT();
    //testRRT.printGraph();

    return 0;
}
