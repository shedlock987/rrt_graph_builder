#include <gtest/gtest.h>
#include <cmath>
#include <vector>
#include "graph.h"

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

        friend class Graph;
    };

    TEST_F(Graph_test, ADDNODE_CREATE_WITH_BASE_CONSTRUCTOR_TEST)
    {
        underTest_->addNode(std::make_tuple(4.1, 5, 0), 2.2F);
        EXPECT_EQ(underTest_->adjacencyList_.size(), 2);
        EXPECT_EQ(std::get<0>(underTest_->adjacencyList_.front()->crdnts_),10.0F);
        EXPECT_EQ(std::get<1>(underTest_->adjacencyList_.front()->crdnts_),10.0F);
        EXPECT_EQ(std::get<2>(underTest_->adjacencyList_.front()->crdnts_),0);
        EXPECT_EQ(underTest_->adjacencyList_.front()->back_edge_weight_,0);

        EXPECT_EQ(std::get<0>(underTest_->adjacencyList_.back()->crdnts_),4.1F);
        EXPECT_EQ(std::get<1>(underTest_->adjacencyList_.back()->crdnts_),5.0F);
        EXPECT_EQ(std::get<2>(underTest_->adjacencyList_.back()->crdnts_),0.0F);
        EXPECT_EQ(underTest_->adjacencyList_.back()->back_edge_weight_,2.2F);
    }

    TEST_F(Graph_test, ADDNODE_LINK_WITH_BASE_CONSTRUCTOR_TEST)
    {
        underTest_->addNode(std::make_tuple(4.1, 5, 0), 2.2F);
        underTest_->addNode(underTest_->adjacencyList_.back(), std::make_tuple(8.2F, 10.0F ,1.0F), 4.4F);

        //Ensure proper assignment
        EXPECT_NEAR(std::get<0>(underTest_->adjacencyList_.back()->crdnts_),8.2F, 0.00001F);
        EXPECT_NEAR(std::get<1>(underTest_->adjacencyList_.back()->crdnts_),10.0F,0.00001F);
        EXPECT_NEAR(std::get<2>(underTest_->adjacencyList_.back()->crdnts_),1.0F,0.00001F);
        EXPECT_NEAR(underTest_->adjacencyList_.back()->back_edge_weight_,4.4F,0.00001F);

        //Ensure proper adjacency list conections
        EXPECT_EQ(underTest_->adjacencyList_.front()->fwd_node_.front(), 
                    underTest_->adjacencyList_.back()->back_node_);
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
        underTest_->addNode(std::make_tuple(1.1, 2.1, 0), 1.0F);
        handle = underTest_->adjacencyList_.back();
        underTest_->addNode(handle, std::make_tuple(1.2, 2.2, 0), 3.0F);
        handle = underTest_->adjacencyList_.front();
        underTest_->addNode(handle, std::make_tuple(2.1, 1.1, 0), 2.0F);
        underTest_->addNode(std::make_tuple(2.2, 1.2, 0), 4.0F);
        underTest_->addNode(handle, std::make_tuple(3.1F, 3.2F, 0), 5.0F);

        /* Test Head-to-Tail */
        EXPECT_EQ(underTest_->adjacencyList_.front()->fwd_node_.size(), 3);
        EXPECT_EQ(underTest_->adjacencyList_.back()->back_node_, 
                  underTest_->adjacencyList_.front());
        EXPECT_NEAR(underTest_->adjacencyList_.back()->back_edge_weight_,5.0F, 0.00001F);
    }

    TEST_F(Graph_test, GRAPH_DELETE_INTERIOR_NODE)
    {
        /* Delete Node (Delete Node #4 which has back-edge weight of 2.0F)
           Test Graph Strucure BEFORE
                            0________
                           / \       |
                     1.0F /   \ 2.0F |
                         /     \     |
                        0   ____0    |5.0F
                  3.0F /   |     \   |
                      /    | 4.0F \  | 
                     0     |       \ |
                      6.0F |        \|
                           0         0

            Test Graph Strucure AFTER
                            0________
                           //\       |
                     1.0F /|  \ 2.0F |
                         / |   \     |
                        0  |    \    |5.0F
                  3.0F /   |     \   |
                      /    | 4.0F \  | 
                     0     |       \ |
                      6.0F |        \|
                           0         0
        */

        /* Build Test Graph */
        Node *handle, *prev, *next;
        underTest_->addNode(std::make_tuple(1.1, 2.1, 0), 1.0F);
        handle = underTest_->adjacencyList_.back();
        underTest_->addNode(handle, std::make_tuple(1.2, 2.2, 0), 3.0F);
        handle = underTest_->adjacencyList_.front();
        underTest_->addNode(handle, std::make_tuple(2.1, 1.1, 0), 2.0F);
        underTest_->addNode(std::make_tuple(2.2, 1.2, 0), 4.0F);
        underTest_->addNode(handle, std::make_tuple( 3.1F, 3.2F, 0), 5.0F);
        handle = underTest_->adjacencyList_.at(3);
        underTest_->addNode(handle, std::make_tuple( 2.3F, 1.3F, 0), 6.0F);
        next = underTest_->adjacencyList_.back();
        prev = handle->back_node_;
        next = handle->fwd_node_.back();

        /* Sanity Check Test Graph */
        EXPECT_EQ(underTest_->adjacencyList_.size(), 7);
        EXPECT_NEAR(handle->back_edge_weight_, 2.0F,0.00001F);


        /* Delete Interior Node */
        underTest_->deleteNode(handle);

        /* Verify Deletion */
        handle = underTest_->adjacencyList_.at(3);
        EXPECT_EQ(underTest_->adjacencyList_.size(), 6);
        EXPECT_NEAR(handle->back_edge_weight_, 4.0F,0.00001F);

        /* Verify forward connection Propagation */
        EXPECT_EQ(underTest_->adjacencyList_.front()->fwd_node_.size(), 4);
        EXPECT_EQ(underTest_->adjacencyList_.front()->fwd_node_.front()->back_edge_weight_, 1.0F);
        EXPECT_EQ(underTest_->adjacencyList_.front()->fwd_node_.at(1)->back_edge_weight_, 2.0F);
        EXPECT_EQ(underTest_->adjacencyList_.front()->fwd_node_.at(2)->back_edge_weight_, 5.0F);
        EXPECT_EQ(underTest_->adjacencyList_.front()->fwd_node_.back()->back_edge_weight_, 5.0F);
        
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
        underTest_->addNode(std::make_tuple(1.1, 2.1, 0), 1.0F);
        handle = underTest_->adjacencyList_.back();
        underTest_->addNode(handle, std::make_tuple( 1.2, 2.2, 0), 3.0F);
        handle = underTest_->adjacencyList_.front();
        underTest_->addNode(handle, std::make_tuple( 2.1, 1.1, 0), 2.0F);
        underTest_->addNode(std::make_tuple(2.2, 1.2, 0), 4.0F);
        underTest_->addNode(handle,std::make_tuple( 3.1F, 3.2F, 0), 5.0F);

        /* Sanity Check Test Graph */
        EXPECT_EQ(underTest_->adjacencyList_.size(), 6);
        EXPECT_EQ(underTest_->adjacencyList_.front()->back_node_, nullptr);

        /* Delete Graph Head */
        underTest_->deleteNode(head);

        /* Verify Deletion */
        EXPECT_EQ(underTest_->adjacencyList_.size(), 5);

        /* Verify Proper Node was selected as New Head */
        /* New Head should be the node with <previously> the smallest edge*/
        EXPECT_EQ(underTest_->adjacencyList_.front()->fwd_node_.size(), 3);
        EXPECT_EQ(underTest_->adjacencyList_.front()->fwd_node_.front()->back_edge_weight_, 3.0F);
        EXPECT_EQ(underTest_->adjacencyList_.front()->fwd_node_.back()->back_edge_weight_, 5.0F);
        EXPECT_EQ(underTest_->adjacencyList_.front()->fwd_node_.at(1)->back_edge_weight_, 2.0F);
    }
};



int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

