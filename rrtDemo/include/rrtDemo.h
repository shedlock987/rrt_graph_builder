#include <vector>
#include <memory>
#include <boost/python.hpp>
#include "rrt.h"
#include "graph.h"

#ifndef rtDemo_H_
#define rtDemo_H_

namespace rrt
{
class VisRRT
{
public:
    VisRRT();
    VisRRT(pose_t _range_a, pose_t _range_b,
           pose_t _origin, pose_t _dest,
           double _max_angle_rad, double _max_dist,
           double _min_dist, double _max_interval,
           double _max_time, bool _dim_3D, int _node_limit);

    ~VisRRT();

    void buildRRT();
    bool stepRRT();

    void initializeRRT(
        pose_t _range_a, pose_t _range_b,
        pose_t _origin, pose_t _dest,
        double _max_angle_rad, double _max_dist,
        double _min_dist, double _max_interval,
        double _max_time, bool _dim_3D, int _node_limit);

    void setBoundaries(pose_t _range_a, pose_t _range_b);
    void setOrigin(pose_t _origin);
    void updateDestination(pose_t _dest);
    void updateConstraints(double _max_angle_rad, double _max_dist, double _min_dist, double _max_interval);
    void setDim3D(bool _dim_3D);
    void setNodeLimit(int _node_limit);
    void setOccupancyMap(std::vector<std::vector<double>> _occp_coords,
                         std::vector<double> _occp_widths,
                         std::vector<double> _occp_interval);
    int getNodeCount();
    bool isComplete();
    Node* getNodeAt(int idx);
    boost::python::list getForwardIndices(int idx);

private:
    std::unique_ptr<RRT> rrt_;
};
}
#endif /* rrtDemo_H_ */