#include <gtest/gtest.h>
#include <cmath>
#include <vector>
#include "rrt.h"

namespace rrt
{

    class RRT_test : public ::testing::Test
    {
        private:
        static constexpr double max_angle_rad_ = 0.8;
        static constexpr double max_dist_ = 1.0;
        static constexpr double min_dist_ = 0.5;
        static constexpr double max_interval_ = 2.0;
        static constexpr double max_time_ = 10.0;
        static constexpr bool dim_3D_ = true;
        static constexpr int node_limit_ = 1000;
        protected:
        std::shared_ptr<RRT> underTest_;  // Default Constructor
        std::vector<RRT::occupancy_t>() occupancy_map_ = {
            {{2.0, 2.0, 1.0}, 2.0},
            {{4.0, 4.0, 2.0}, 0.5}
        };

        virtual void SetUp()
        {
            underTest_ = std::make_shared<RRT>(
                occupancy_map_,
                -5.0, 0.0, 5.0, 5.0,   // _range_a_x, _range_a_y, _range_b_x, _range_b_y
                0.0, 0.0, 5.0, 5.0,    // _origin_x, _origin_y, _dest_x, _dest_y
                max_angle_rad_, max_dist_, min_dist_, max_interval_,
                max_time_, dim_3D_, node_limit_);
        }

        virtual void TearDown()
        {
        }
    };
}