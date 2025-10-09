#include "rrtDemo.h"
#include "graph.h"
#include "rrt.h"
#include <iostream>
#include <boost/python.hpp>
#include <boost/python/stl_iterator.hpp>
#include <boost/python/extract.hpp>
#include <vector>
#include <tuple> // For std::get on pose_t
#include <algorithm> // For std::find if needed, but using getIndex
#include <utility> // For std::pair

using namespace boost::python;

namespace rrt
{
using pose_t = std::tuple<double, double, double, double>; // x, y, time, heading

VisRRT::VisRRT()
    : rrt_(new RRT(
        std::vector<RRT::occupancy_t>(), // empty occupancy map
        std::get<0>(pose_t(-5.0, -5.0, 0.0, 0.0)), std::get<1>(pose_t(-5.0, -5.0, 0.0, 0.0)),
        std::get<0>(pose_t(5.0, 5.0, 10.0, 0.0)), std::get<1>(pose_t(5.0, 5.0, 10.0, 0.0)),
        std::get<0>(pose_t(0.0, 0.0, 0.0, 0.7854)), std::get<1>(pose_t(0.0, 0.0, 0.0, 0.7854)),
        std::get<0>(pose_t(5.0, 5.0, 10.0, 0.0)), std::get<1>(pose_t(5.0, 5.0, 10.0, 0.0)),
        0.8, 1.0, 0.5, 2.0, // constraints
        10.0, // max_time
        true, // dim_3D
        10000, // node_limit
        1      // max_admissible (default value)
    ))
{}

VisRRT::VisRRT(std::vector<RRT::occupancy_t> _occupancy_map,
    pose_t _range_a, pose_t _range_b,
    pose_t _origin, pose_t _dest,
    double _max_angle_rad, double _max_dist,
    double _min_dist, double _max_interval,
    double _max_time, bool _dim_3D, int _iteration_limit,
    int _max_admissible)
    : rrt_(new RRT(_occupancy_map,
        _range_a, _range_b, _origin, _dest,
        _max_angle_rad, _max_dist, _min_dist, _max_interval,
        _max_time, _dim_3D, _iteration_limit, _max_admissible))
{
}

VisRRT::VisRRT(pose_t _range_a, pose_t _range_b,
    pose_t _origin, pose_t _dest,
    double _max_angle_rad, double _max_dist,
    double _min_dist, double _max_interval,
    double _max_time, bool _dim_3D, int _iteration_limit,
    int _max_admissible)
    : rrt_(new RRT(_range_a, _range_b, _origin, _dest,
        _max_angle_rad, _max_dist, _min_dist, _max_interval,
        _max_time, _dim_3D, _iteration_limit, _max_admissible))
{
}

VisRRT::~VisRRT() = default;

void VisRRT::buildRRT() {
    rrt_->buildRRT();
}

bool VisRRT::stepRRT() {
    return rrt_->stepRRT();
}

// Optimized: pose_t-based initializeRRT (12 args; extracts x/y for time horizon, includes initial_heading)
void VisRRT::initializeRRT(
    pose_t _range_a, pose_t _range_b,
    pose_t _origin, pose_t _dest,
    double _max_angle_rad, double _max_dist,
    double _min_dist, double _max_interval,
    double _max_time, bool _dim_3D, int _iteration_limit,
    double _initial_heading)
{
    // Extract x/y for boundaries + use _max_time as horizon
    double range_a_x = std::get<0>(_range_a);
    double range_a_y = std::get<1>(_range_a);
    double range_b_x = std::get<0>(_range_b);
    double range_b_y = std::get<1>(_range_b);
    rrt_->setBoundaries(range_a_x, range_a_y, range_b_x, range_b_y, _max_time);
    rrt_->setOrigin(std::get<0>(_origin), std::get<1>(_origin), std::get<2>(_origin));
    rrt_->updateDestination(std::get<0>(_dest), std::get<1>(_dest), std::get<2>(_dest));
    rrt_->updateConstraints(_max_angle_rad, _max_dist, _min_dist, _max_interval);
    rrt_->setDim3D(_dim_3D);
    rrt_->setIterationLimit(_iteration_limit);
}

void VisRRT::setBoundaries(pose_t _range_a, pose_t _range_b)
{
    rrt_->setBoundaries(_range_a, _range_b);
}

void VisRRT::setOrigin(pose_t _origin)
{
    rrt_->setOrigin(std::get<0>(_origin), std::get<1>(_origin), std::get<2>(_origin));
}

void VisRRT::updateDestination(pose_t _dest)
{
    rrt_->updateDestination(std::get<0>(_dest), std::get<1>(_dest), std::get<2>(_dest));
}

void VisRRT::updateConstraints(double _max_angle_rad, double _max_dist,
    double _min_dist, double _max_interval)
{
    rrt_->updateConstraints(_max_angle_rad, _max_dist, _min_dist, _max_interval);
}

void VisRRT::setDim3D(bool _dim_3D)
{
    rrt_->setDim3D(_dim_3D);
}

void VisRRT::setIterationLimit(int _iteration_limit)
{
    rrt_->setIterationLimit(_iteration_limit);
}

void VisRRT::setOccupancyMap(
    std::vector<std::vector<double>> _occp_coords,
    std::vector<double> _occp_widths,
    std::vector<double> _occp_interval)
{
    std::vector<RRT::occupancy_t> occupancy_map;
    for (size_t i = 0; i < _occp_coords.size(); ++i) {
        RRT::occupancy_t occ;
        occ.first = std::make_tuple(_occp_coords[i][0], _occp_coords[i][1], _occp_interval[i], 0.0);
        occ.second = _occp_widths[i];
        occupancy_map.push_back(occ);
    }
    rrt_->setOccupancyMap(occupancy_map);
}

int VisRRT::getNodeCount()
{
    return rrt_->adjacencyList_.size();
}

bool VisRRT::isComplete()
{
    return rrt_->isComplete();
}

bool VisRRT::isAdmissible(Node* node)
{
    return rrt_->isAdmissible();
}

void VisRRT::updateInitialHeading(double _initial_heading)
{
    rrt_->updateInitialHeading(_initial_heading);
}

Node* VisRRT::getNodeAt(int idx)
{
    if (idx < static_cast<int>(rrt_->adjacencyList_.size())) {
        return rrt_->adjacencyList_[idx];
    }
    return nullptr;
}

// UPDATED: Return boost::python::list for seamless Python conversion
boost::python::list VisRRT::getForwardIndices(int idx)
{
    Node* node = getNodeAt(idx);
    if (!node) {
        return boost::python::list();
    }
    boost::python::list py_list;
    for (Node* fwd : node->fwd_node_) {
        int fwd_idx = rrt_->getIndex(fwd);
        if (fwd_idx >= 0) { // Assuming getIndex returns -1 or similar for invalid
            py_list.append(fwd_idx);
        }
    }
    return py_list;
}

} // namespace rrt

// Converter for std::tuple<double, double, double, double> from Python tuples (x, y, time, heading)
class tuple_from_python_converter {
public:
    tuple_from_python_converter() {
        boost::python::converter::registry::push_back(
            &convertible,
            &construct,
            boost::python::type_id<std::tuple<double, double, double, double>>()
        );
    }
private:
    static void* convertible(PyObject* obj) {
        if (PyTuple_Check(obj) && PyTuple_Size(obj) == 4) {
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
        double h = python::extract<double>(py_tuple[3]);
        void* storage = ((python::converter::rvalue_from_python_storage<std::tuple<double, double, double, double>>*)data)->storage.bytes;
        new (storage) std::tuple<double, double, double, double>(x, y, z, h);
        data->convertible = storage;
    }
};

// Converter for rrt::RRT::occupancy_t from Python tuples ((x, y, time, heading), width)
class occupancy_from_python_converter {
public:
    occupancy_from_python_converter() {
        boost::python::converter::registry::push_back(
            &convertible,
            &construct,
            boost::python::type_id<rrt::RRT::occupancy_t>());
    }
private:
    static void* convertible(PyObject* obj) {
        if (PyTuple_Check(obj) && PyTuple_Size(obj) == 2) {
            PyObject* first = PyTuple_GET_ITEM(obj, 0);
            if (PyTuple_Check(first) && PyTuple_Size(first) == 4) {
                return obj;
            }
        }
        return nullptr;
    }
    static void construct(PyObject* obj,
        boost::python::converter::rvalue_from_python_stage1_data* data) {
        // Directly extract using PyTuple_GET_ITEM to avoid vexing parse issues
        PyObject* py_pose_tuple = PyTuple_GET_ITEM(obj, 0);
        PyObject* py_width_obj = PyTuple_GET_ITEM(obj, 1);
        double x = PyFloat_AsDouble(PyTuple_GET_ITEM(py_pose_tuple, 0));
        double y = PyFloat_AsDouble(PyTuple_GET_ITEM(py_pose_tuple, 1));
        double t = PyFloat_AsDouble(PyTuple_GET_ITEM(py_pose_tuple, 2));
        double h = PyFloat_AsDouble(PyTuple_GET_ITEM(py_pose_tuple, 3));
        double w = PyFloat_AsDouble(py_width_obj);
        void* storage = ((boost::python::converter::rvalue_from_python_storage<rrt::RRT::occupancy_t>*)data)->storage.bytes;
        new (storage) rrt::RRT::occupancy_t(std::make_tuple(x, y, t, h), w);
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
    using rrt::pose_t;
    // Register tuple converter
    tuple_from_python_converter();
    occupancy_from_python_converter();

    class_<rrt::Node, boost::noncopyable>("Node", no_init)
        .def("xCrdnt", &rrt::Node::xCrdnt)
        .def("yCrdnt", &rrt::Node::yCrdnt)
        .def("heading", &rrt::Node::heading)
        .def("time", &rrt::Node::time)
        .def("backEdgeWeight", &rrt::Node::backEdgeWeight)
        // Optionally add .def("heading", &rrt::Node::heading) if Node has a heading() method exposed
        ;

    class_<rrt::VisRRT, boost::noncopyable>("RRT", no_init)
        .def(init<>()) // Default
        .def(init<std::vector<rrt::RRT::occupancy_t>, pose_t, pose_t, pose_t, pose_t, double, double, double, double, double, bool, int, int>())
        .def(init<pose_t, pose_t, pose_t, pose_t, double, double, double, double, double, bool, int, int>())
        .def("buildRRT", &rrt::VisRRT::buildRRT)
        .def("stepRRT", &rrt::VisRRT::stepRRT)
        .def("initializeRRT", static_cast<void (rrt::VisRRT::*)(
            pose_t, pose_t, pose_t, pose_t,
            double, double, double, double, double, bool, int, double
        )>(&rrt::VisRRT::initializeRRT))
        .def("setBoundaries", static_cast<void (rrt::VisRRT::*)(pose_t, pose_t)>(&rrt::VisRRT::setBoundaries))
        .def("setOrigin", static_cast<void (rrt::VisRRT::*)(pose_t)>(&rrt::VisRRT::setOrigin))
        .def("updateDestination", static_cast<void (rrt::VisRRT::*)(pose_t)>(&rrt::VisRRT::updateDestination))
        .def("updateConstraints", &rrt::VisRRT::updateConstraints)
        .def("setDim3D", &rrt::VisRRT::setDim3D)
        .def("setIterationLimit", &rrt::VisRRT::setIterationLimit)
        .def("setOccupancyMap", static_cast<void (rrt::VisRRT::*)(std::vector<std::vector<double>>, std::vector<double>, std::vector<double>)>(&rrt::VisRRT::setOccupancyMap))
        .def("isComplete", &rrt::VisRRT::isComplete)
        .def("getNodeCount", &rrt::VisRRT::getNodeCount)
        .def("getNodeAt", &rrt::VisRRT::getNodeAt, return_value_policy<reference_existing_object>())
        .def("getForwardIndices", &rrt::VisRRT::getForwardIndices)
        .def("isAdmissible", &rrt::VisRRT::isAdmissible)
        .def("updateInitialHeading", &rrt::VisRRT::updateInitialHeading)
        ;

    iterable_converter()
        .from_python<std::vector<double>>()
        .from_python<std::vector<std::vector<double>>>()
        .from_python<std::vector<rrt::RRT::occupancy_t>>();
}