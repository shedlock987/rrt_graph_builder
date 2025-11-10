#include <gtest/gtest.h>
#include <cmath>
#include <vector>
#include "graph.h"
#include "rrt.h"

namespace rrt
{
    /*
    class Node_test : public ::testing::Test
    {
        protected:
        Node* under_test_(10.0F, 10.0F, 1.0F);
        std::shared_ptr<Node*> alt_node_;
        std::shared_ptr<Node> value_node_;

        virtual void SetUp()
        {

        }

        virtual void TearDown()
        {
        }
    };
    */

    class Graph_test : public ::testing::Test
    {
        protected:
        std::shared_ptr<Graph> underTest_;  // Default Constructor

        virtual void SetUp()
        {
            underTest_ = std::make_shared<Graph>(10.0F, 10.0F);
        }

        virtual void TearDown()
        {
        }
    };

    class RRT_test : public ::testing::Test
    {
        protected:
        std::shared_ptr<RRT> rrtTest_;  // Default Constructor

        virtual void SetUp()
        {
            // use pose_t-based constructor overload (updated signature in rrt.h)
            pose_t range_a = std::make_tuple(-5.0, -5.0, 0.0, 0.0);
            pose_t range_b = std::make_tuple(5.0, 5.0, 0.0, 0.0);
            pose_t origin  = std::make_tuple(0.0, 0.0, 0.0, 0.0);
            pose_t dest    = std::make_tuple(5.0, 5.0, 0.0, 0.0);

            rrtTest_ = std::make_shared<RRT>(
                std::vector<RRT::occupancy_t>(), // optional occupancy map
                range_a, range_b,
                origin, dest,
                0.8,    // max_angle_rad
                1.0,    // max_dist
                0.5,    // min_dist
                2.0,    // max_interval
                10.0,   // max_time
                true,   // dim_3D
                100,    // iteration_limit
                1       // max_admissible
            );
        }

        virtual void TearDown()
        {
        }
    };

    TEST_F(Graph_test, ADDNODE_CREATE_WITH_BASE_CONSTRUCTOR_TEST)
    {
        underTest_->addNode(std::make_tuple(4.1, 5, 0, 0), 2.2);
        EXPECT_EQ(underTest_->adjacencyList_.size(), 2);
        EXPECT_EQ(underTest_->adjacencyList_.front()->xCrdnt(),10.0); 
        EXPECT_EQ(underTest_->adjacencyList_.front()->yCrdnt(),10.0);
        EXPECT_EQ(underTest_->adjacencyList_.front()->time(),0);
        EXPECT_EQ(underTest_->adjacencyList_.front()->backEdgeWeight(),0);

        EXPECT_EQ(underTest_->adjacencyList_.back()->xCrdnt(),4.1);
        EXPECT_EQ(underTest_->adjacencyList_.back()->yCrdnt(),5.0);
        EXPECT_EQ(underTest_->adjacencyList_.back()->time(),0.0);
        EXPECT_EQ(underTest_->adjacencyList_.back()->backEdgeWeight(),2.2);
    }

    TEST_F(Graph_test, ADDNODE_LINK_WITH_BASE_CONSTRUCTOR_TEST)
    {
        underTest_->addNode(std::make_tuple(4.1, 5, 0, 0), 2.2F);
        underTest_->addNode(underTest_->adjacencyList_.back(), std::make_tuple(8.2F, 10.0F ,1.0F, 0.0F), 4.4F);

        //Ensure proper assignment
        EXPECT_NEAR(underTest_->adjacencyList_.back()->xCrdnt(),8.2F, 0.00001F);
        EXPECT_NEAR(underTest_->adjacencyList_.back()->yCrdnt(),10.0F,0.00001F);
        EXPECT_NEAR(underTest_->adjacencyList_.back()->time(),1.0F,0.00001F);
        EXPECT_NEAR(underTest_->adjacencyList_.back()->backEdgeWeight(),4.4F,0.00001F);

        //Ensure proper adjacency list conections
        EXPECT_EQ(underTest_->adjacencyList_.front()->fwd_node_.front(), 
                    underTest_->adjacencyList_.back()->BackCnnctn());
    }

    TEST_F(Graph_test, GRAPH_BASICS)
    {
        /* Test Graph Strucure
                            0________
                           / \       |
                     1.0F /   \ 2.0F |
                         /     \     |
                        0       0    |5.0F
                  3.0F /         \   |
                      /      4.0F \  | 
                     0             \ |
                                    \|
                                     0
        */

        /* Build Test Graph */
        Node* handle;
        underTest_->addNode(std::make_tuple(1.1, 2.1, 0, 0), 1.0F);
        handle = underTest_->adjacencyList_.back();
        underTest_->addNode(handle, std::make_tuple(1.2, 2.2, 0, 0), 3.0F);
        handle = underTest_->adjacencyList_.front();
        underTest_->addNode(handle, std::make_tuple(2.1, 1.1, 0, 0), 2.0F);
        underTest_->addNode(std::make_tuple(2.2, 1.2, 0, 0), 4.0F);
        underTest_->addNode(handle, std::make_tuple(3.1F, 3.2F, 0, 0), 5.0F);

        /* Test Head-to-Tail */
        EXPECT_EQ(underTest_->adjacencyList_.front()->fwd_node_.size(), 3);
        EXPECT_EQ(underTest_->adjacencyList_.back()->BackCnnctn(), 
                  underTest_->adjacencyList_.front());
        EXPECT_NEAR(underTest_->adjacencyList_.back()->backEdgeWeight(),5.0F, 0.00001F);
    }

    TEST_F(Graph_test, GRAPH_DELETE_INTERIOR_NODE)
    {
        /* Delete Node (Delete Node #4 which has back-edge weight of 2.0F)
           Test Graph Strucure BEFORE
                            0
                           / \       
                     1.0F /   \ 2.0F 
                         /     \     
                        0   ____0   
                  3.0F /   |     \   
                      /    |      \ 4.0F
                     0     |       \ 
                      6.0F |        \
                           0         0

            Test Graph Strucure AFTER
                            0
                           //\       
                     1.0F /|  \ 4.0F 
                         / |   \    
                        0  |    \    
                  3.0F /   |     \   
                      /    |      \  
                     0     |       \ 
                      6.0F |        \
                           0         0
        */

        /* Build Test Graph */
        Node *head = underTest_->adjacencyList_.front();
        underTest_->addNode(head, std::make_tuple(1.1, 2.1, 0, 0), 1.0F);
        Node *handle = underTest_->adjacencyList_.back();
        underTest_->addNode(handle, std::make_tuple(1.2, 2.2, 0, 0), 3.0F);
        /// left side completed

        underTest_->addNode(head, std::make_tuple(2.1, 1.1, 0, 0), 2.0F);
        Node *to_be_deleted = underTest_->adjacencyList_.back();
        underTest_->addNode(to_be_deleted, std::make_tuple(2.2, 1.2, 0, 0), 4.0F);
        Node *child1 = underTest_->adjacencyList_.back();
        underTest_->addNode(to_be_deleted, std::make_tuple(3.1F, 3.2F, 0, 0), 6.0F);
        Node *child2 = underTest_->adjacencyList_.back();
        ///right side completed

        /* Sanity Check Test Graph */
        EXPECT_EQ(underTest_->adjacencyList_.size(), 6);
        EXPECT_EQ(head->fwd_node_.size(), 2);

        /* Delete the Node */
        underTest_->deleteNode(to_be_deleted);

        /* Verify Deletion */
        EXPECT_EQ(head->fwd_node_.size(), 3); //with propogation heads should now have 3 fwd connections (increased from 2)
        EXPECT_EQ(underTest_->adjacencyList_.size(), 5);
        // check that child1 and child2 are included in head's fwd_node_ list
        EXPECT_NE(std::find(head->fwd_node_.begin(), head->fwd_node_.end(), child1), head->fwd_node_.end());
        EXPECT_NE(std::find(head->fwd_node_.begin(), head->fwd_node_.end(), child2), head->fwd_node_.end());


        
    }

    TEST_F(Graph_test, GRAPH_DELETE_HEAD_NODE)
    {
        /* Test Graph Strucure BEFORE
                      HEAD->0________
                           / \       |
                     1.0F /   \ 2.0F |
                         /     \     |
                        0       0    |5.0F
                  3.0F /         \   |
                      /      4.0F \  | 
                     0             \ |
                                    \|
                                     0
          Test Graph Strucure AFTER
                        _____________
                        |            |
                        |            |
                        |  2.0F      |
              NEW HEAD->0_______0    |5.0F
                  3.0F /         \   |
                      /      4.0F \  | 
                     0             \ |
                                    \|
                                     0
        */

        /* Build Test Graph */
        Node *handle, *head, *temp;
        head = underTest_->adjacencyList_.front();
        underTest_->addNode(std::make_tuple(1.1, 2.1, 0, 0), 1.0F);
        handle = underTest_->adjacencyList_.back();
        underTest_->addNode(handle, std::make_tuple( 1.2, 2.2, 0, 0), 3.0F);
        handle = underTest_->adjacencyList_.front();
        underTest_->addNode(handle, std::make_tuple( 2.1, 1.1, 0, 0), 2.0F);
        underTest_->addNode(std::make_tuple(2.2, 1.2, 0, 0), 4.0F);
        underTest_->addNode(handle,std::make_tuple( 3.1F, 3.2F, 0, 0), 5.0F);

        /* Sanity Check Test Graph */
        EXPECT_EQ(underTest_->adjacencyList_.size(), 6);
        EXPECT_EQ(underTest_->adjacencyList_.front()->BackCnnctn(), nullptr);

        /* Delete Graph Head */
        underTest_->deleteNode(head);

        /* Verify Deletion */
        EXPECT_EQ(underTest_->adjacencyList_.size(), 5);

        /* Verify Proper Node was selected as New Head */
        /* New Head should be the node with <previously> the smallest edge*/
        EXPECT_EQ(underTest_->adjacencyList_.front()->fwd_node_.size(), 3);
        EXPECT_EQ(underTest_->adjacencyList_.front()->fwd_node_.front()->backEdgeWeight(), 3.0);
        EXPECT_EQ(underTest_->adjacencyList_.front()->fwd_node_.back()->backEdgeWeight(), 5.0);
        EXPECT_EQ(underTest_->adjacencyList_.front()->fwd_node_.at(1)->backEdgeWeight(), 2.0);
    }

    TEST_F(RRT_test, ForwardNodeTimeIsGreaterOrEqualToParent) 
    {
        
        // Build a small RRT
        for (int i = 0; i < 5; i++) {
            rrtTest_->stepRRT();
        }

        // For each node, check that all forward-connected nodes have time >= this node's time
        for (size_t parent_idx = 0; parent_idx < rrtTest_->adjacencyList_.size(); ++parent_idx) {
            const Node* parent = rrtTest_->adjacencyList_[parent_idx];
            double parent_time = parent->time();
            for (const Node* child : parent->fwd_node_) {
                // Find the index of the child in the adjacency list (if present)
                auto it = std::find(rrtTest_->adjacencyList_.begin(), rrtTest_->adjacencyList_.end(), child);
                size_t child_idx = (it != rrtTest_->adjacencyList_.end()) ? std::distance(rrtTest_->adjacencyList_.begin(), it) : static_cast<size_t>(-1);

                EXPECT_GE(child->time(), parent_time)
                    << "Child node time is less than parent node time.\n"
                    << "Parent ptr: " << parent << " (index " << parent_idx << ")\n"
                    << "Child ptr: " << child << " (index " << child_idx << ")";
            }
        }
    }

    TEST_F(RRT_test, OccupancyMapPlacement2D_CheckOccupied) 
    {

    /*
    Three nodes all connected to origin
    1. Node and endge completely outside occupied space
    2. Noed is in the occuipied Space
    3. Node is not in Occupied space but it's edge connection is in occupied space

                      0
                     /
             _______/___        
            |      /   |
            |     /0   | (Occupied Box)
            |    //    |
            |___//_____|     
               //
              //0 
             ///
            ///
           ///
           0 (Origin)
     */


    
        rrtTest_->setDim3D(false);

        /// Grid cell x[2,3] y[2,3] is occupied
        pose_t occupied_coord = std::make_tuple(2.5F, 2.5F, 0, 0.0F);
        RRT::occupancy_t occupied_space;
        occupied_space.first = occupied_coord;
        occupied_space.second = 1.0F;

        // Make occupancy_map_ contain exactly this single occupancy element
        rrtTest_->occupancy_map_ = std::vector<RRT::occupancy_t>{ occupied_space };

        // sanity checks
        ASSERT_FALSE(rrtTest_->adjacencyList_.empty());
        Node *head = rrtTest_->adjacencyList_.front();
        ASSERT_NE(head, nullptr);

        rrtTest_->addNode(head, std::make_tuple(0.5, 0.5, 0, 0), 1.0F);
        EXPECT_EQ(rrtTest_->adjacencyList_.size(), 2);
        Node *inbetween = rrtTest_->adjacencyList_.back();
  //      EXPECT_FALSE(rrtTest_->isOccupied(inbetween));
/*
        rrtTest_->addNode(head, std::make_tuple(2.5, 2.5, 0, 0), 1.0F);
        EXPECT_EQ(rrtTest_->adjacencyList_.size(), 3);
        Node *inside = rrtTest_->adjacencyList_.back();
        EXPECT_TRUE(rrtTest_->isOccupied(inside));

        rrtTest_->addNode(head, std::make_tuple(5.5, 5.5, 0, 0), 1.0F);
        EXPECT_EQ(rrtTest_->adjacencyList_.size(), 4);
        Node *edge_in_abs_out = rrtTest_->adjacencyList_.back();
        EXPECT_TRUE(rrtTest_->isOccupied(edge_in_abs_out));
        
        rrtTest_->addNode(head, std::make_tuple(2.5, 2.5, 0, 0), 1.0F);
        EXPECT_TRUE(rrtTest_->isOccupied(rrtTest_->adjacencyList_.back()));

        rrtTest_->addNode(head, std::make_tuple(5.5, 5.5, 0, 0), 1.0F);
        EXPECT_FALSE(rrtTest_->isOccupied(rrtTest_->adjacencyList_.back()));
        */
    }


/* Commented out WallOfFire test per request
TEST_F(RRT_test, WallOfFire) 
{
    // Redirect std::cout
    std::streambuf* orig_buf = std::cout.rdbuf();
    std::ostringstream capture;
    std::cout.rdbuf(capture.rdbuf());

    rrtTest_->setDim3D(false);

    pose_t occupied_coord = std::make_tuple(3.1F, 0.0F, 0, 0.0F);
    RRT::occupancy_t occupied_space;
    occupied_space.first = occupied_coord;
    occupied_space.second = 3.0F;
    rrtTest_->occupancy_map_->push_back(occupied_space);

    // Build Test Graph
    rrtTest_->buildRRT();

    // Restore std::cout
    std::cout.rdbuf(orig_buf);

    std::string output = capture.str();
    EXPECT_FALSE(rrtTest_->isAdmissible());
    EXPECT_TRUE(rrtTest_->isComplete());
}
*/


};


int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

