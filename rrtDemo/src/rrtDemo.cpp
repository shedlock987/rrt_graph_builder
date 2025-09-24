#include "rrtDemo.h"
#include "graph.h"
#include "rrt.h"
#include <iostream>
#include <boost/python.hpp>
#include <boost/python/stl_iterator.hpp>
#include <boost/python/extract.hpp>
#include <vector>
#include <tuple> // For std::get on coordinate_t
#include <algorithm> // For std::find if needed, but using getIndex

using namespace boost::python;

namespace rrt
{
    using coordinate_t = Node::coordinate_t;

    VisRRT::VisRRT() : rrt_(new RRT(
        -5.0, 0.0, 5.0, 5.0, // ranges
        0.0, 0.0, 5.0, 5.0, // origin, dest
        0.8, 1.0, 0.5, 2.0, // constraints
        10.0, // _max_time
        true, // _dim_3D
        10000 // _node_limit
        ))
    {
    }

    // Optimized: coordinate_t-based ctor (no occupancy; call setOccupancyMap separately)
    VisRRT::VisRRT(coordinate_t _range_a, coordinate_t _range_b,
                   coordinate_t _origin, coordinate_t _dest,
                   double _max_angle_rad, double _max_dist,
                   double _min_dist, double _max_interval,
                   double _max_time, bool _dim_3D, int _node_limit)
        : rrt_(new RRT(_range_a, _range_b, _origin, _dest,
                       _max_angle_rad, _max_dist, _min_dist, _max_interval,
                       _max_time, _dim_3D, _node_limit))
    {
    }

    VisRRT::~VisRRT() = default;

    void VisRRT::buildRRT() {
        rrt_->buildRRT();
    }

    bool VisRRT::stepRRT() {
        return rrt_->stepRRT();
    }

    // Optimized: coordinate_t-based initializeRRT (11 args; extracts x/y for time horizon)
    void VisRRT::initializeRRT(
        coordinate_t _range_a, coordinate_t _range_b,
        coordinate_t _origin, coordinate_t _dest,
        double _max_angle_rad, double _max_dist,
        double _min_dist, double _max_interval,
        double _max_time, bool _dim_3D, int _node_limit)
    {
        // Extract x/y for boundaries + use _max_time as horizon
        double range_a_x = std::get<0>(_range_a);
        double range_a_y = std::get<1>(_range_a);
        double range_b_x = std::get<0>(_range_b);
        double range_b_y = std::get<1>(_range_b);
        rrt_->setBoundaries(range_a_x, range_a_y, range_b_x, range_b_y, _max_time);
        rrt_->setOrigin(_origin);
        rrt_->updateDestination(_dest);
        rrt_->updateConstraints(_max_angle_rad, _max_dist, _min_dist, _max_interval);
        rrt_->setDim3D(_dim_3D);
        rrt_->setNodeLimit(_node_limit);
    }    
    
    void VisRRT::setBoundaries(coordinate_t _range_a, coordinate_t _range_b) 
    {
        rrt_->setBoundaries(_range_a, _range_b);
    }

    void VisRRT::setOrigin(coordinate_t _origin) {
        rrt_->setOrigin(_origin);
    }

    void VisRRT::updateDestination(coordinate_t _dest) {
        rrt_->updateDestination(_dest);
    }

    void VisRRT::updateConstraints(double _max_angle_rad, double _max_dist,
                                   double _min_dist, double _max_interval) {
        rrt_->updateConstraints(_max_angle_rad, _max_dist, _min_dist, _max_interval);
    }

    void VisRRT::setDim3D(bool _dim_3D) {
        rrt_->setDim3D(_dim_3D);
    }

    void VisRRT::setNodeLimit(int _node_limit) {
        rrt_->setNodeLimit(_node_limit);
    }

    void VisRRT::setOccupancyMap(
        std::vector<std::vector<double>> _occp_coords,
        std::vector<double> _occp_widths,
        std::vector<double> _occp_interval) {
        std::vector<RRT::occupancy_t> occupancy_map;
        for (size_t i = 0; i < _occp_coords.size(); ++i) {
            RRT::occupancy_t occ;
            occ.first = std::make_tuple(_occp_coords[i][0], _occp_coords[i][1], _occp_interval[i]);
            occ.second = _occp_widths[i];
            occupancy_map.push_back(occ);
        }
        rrt_->setOccupancyMap(occupancy_map);
    }

    int VisRRT::getNodeCount() {
        return rrt_->adjacencyList_.size();
    }

    bool VisRRT::isComplete() {
        return rrt_->isComplete();
    }

    Node* VisRRT::getNodeAt(int idx) {
        if (idx < rrt_->adjacencyList_.size()) {
            return rrt_->adjacencyList_[idx];
        }
        return nullptr;
    }

    std::vector<int> VisRRT::getForwardIndices(int idx) {
        Node* node = getNodeAt(idx);
        if (!node) {
            return {};
        }
        std::vector<int> indices;
        for (Node* fwd : node->fwd_node_) {
            int fwd_idx = rrt_->getIndex(fwd);
            if (fwd_idx >= 0) {  // Assuming getIndex returns -1 or similar for invalid
                indices.push_back(fwd_idx);
            }
        }
        return indices;
    }
} // namespace rrt

// Converter for std::tuple<double, double, double> from Python tuples
class tuple_from_python_converter {
public:
    tuple_from_python_converter() {
        boost::python::converter::registry::push_back(
            &convertible,
            &construct,
            boost::python::type_id<std::tuple<double, double, double>>()
        );
    }

private:
    static void* convertible(PyObject* obj) {
        if (PyTuple_Check(obj) && PyTuple_Size(obj) == 3) {
            return obj;
        }
        return nullptr;
    }

    static void construct(PyObject* obj,
                          boost::python::converter::rvalue_from_python_stage1_data* data) {
        namespace python = boost::python;
        python::handle<> handle(python::borrowed(obj));
        python::tuple py_tuple(handle);
        double x = python::extract<double>(py_tuple[0]);
        double y = python::extract<double>(py_tuple[1]);
        double z = python::extract<double>(py_tuple[2]);
        void* storage = ((python::converter::rvalue_from_python_storage<std::tuple<double, double, double>>*)data)->storage.bytes;
        new (storage) std::tuple<double, double, double>(x, y, z);
        data->convertible = storage;
    }
};

// iterable_converter for vectors
class iterable_converter {
public:
    template <typename Container>
    iterable_converter& from_python() {
        boost::python::converter::registry::push_back(
            &iterable_converter::convertible,
            &iterable_converter::construct<Container>,
            boost::python::type_id<Container>());
        return *this;
    }

    static void* convertible(PyObject* object) {
        return PyObject_GetIter(object) ? object : nullptr;
    }

    template <typename Container>
    static void construct(PyObject* object, boost::python::converter::rvalue_from_python_stage1_data* data) {
        namespace python = boost::python;
        python::handle<> handle(python::borrowed(object));
        typedef python::converter::rvalue_from_python_storage<Container> storage_type;
        void* storage = reinterpret_cast<storage_type*>(data)->storage.bytes;
        typedef python::stl_input_iterator<typename Container::value_type> iterator;
        new (storage) Container(iterator(python::object(handle)), iterator());
        data->convertible = storage;
    }
};

BOOST_PYTHON_MODULE(rrtDemo) {
    using rrt::coordinate_t;

    // Register tuple converter
    tuple_from_python_converter();

    class_<rrt::Node, boost::noncopyable>("Node", no_init)
        .def("xCrdnt", &rrt::Node::xCrdnt)
        .def("yCrdnt", &rrt::Node::yCrdnt)
        .def("time", &rrt::Node::time)
        .def("backEdgeWeight", &rrt::Node::backEdgeWeight)
        ;

    class_<rrt::VisRRT, boost::noncopyable>("RRT", no_init)
        // FIXED: Use init<> instead of ctor<>
        .def(init<>()) // Default
        .def(init<coordinate_t, coordinate_t, coordinate_t, coordinate_t,
                  double, double, double, double, double, bool, int>()) // Coord-based (11 args)

        .def("buildRRT", &rrt::VisRRT::buildRRT)
        .def("stepRRT", &rrt::VisRRT::stepRRT)
        .def("initializeRRT", static_cast<void (rrt::VisRRT::*)(
            coordinate_t, coordinate_t, coordinate_t, coordinate_t,
            double, double, double, double, double, bool, int
        )>(&rrt::VisRRT::initializeRRT))
        .def("setBoundaries", static_cast<void (rrt::VisRRT::*)(coordinate_t, coordinate_t)>(&rrt::VisRRT::setBoundaries))
        .def("setOrigin", static_cast<void (rrt::VisRRT::*)(coordinate_t)>(&rrt::VisRRT::setOrigin))
        .def("updateDestination", static_cast<void (rrt::VisRRT::*)(coordinate_t)>(&rrt::VisRRT::updateDestination))
        .def("updateConstraints", &rrt::VisRRT::updateConstraints)
        .def("setDim3D", &rrt::VisRRT::setDim3D)
        .def("setNodeLimit", &rrt::VisRRT::setNodeLimit)
        .def("setOccupancyMap", static_cast<void (rrt::VisRRT::*)(std::vector<std::vector<double>>, std::vector<double>, std::vector<double>)>(&rrt::VisRRT::setOccupancyMap))
        .def("isComplete", &rrt::VisRRT::isComplete)
        .def("getNodeCount", &rrt::VisRRT::getNodeCount)
        .def("getNodeAt", &rrt::VisRRT::getNodeAt, return_value_policy<reference_existing_object>())
        // NEW: Expose getForwardIndices to return list of int indices
        .def("getForwardIndices", &rrt::VisRRT::getForwardIndices)
        ;

    iterable_converter()
        .from_python<std::vector<double>>()
        .from_python<std::vector<std::vector<double>>>()
        ;
}