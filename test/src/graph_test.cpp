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
        std::shared_ptr<Graph> underTest2_; // Initialized COnstructor
        Node* handle_;

        virtual void SetUp()
        {
            underTest_ = std::make_shared<Graph>();
            underTest2_ = std::make_shared<Graph>(10.0F, 10.0F);
        }

        virtual void TearDown()
        {
        }
    };

    TEST_F(Graph_test, ADDNODE_CREATE_WITH_BASE_CONSTRUCTOR_TEST)
    {
        underTest_->addNode(4.1, 5, 0, 2.2F);
        EXPECT_EQ(underTest_->_adjacencyList.size(), 2);
        EXPECT_EQ(underTest_->_adjacencyList.front()->crdnts_.x_,0);
        EXPECT_EQ(underTest_->_adjacencyList.front()->crdnts_.y_,0);
        EXPECT_EQ(underTest_->_adjacencyList.front()->crdnts_.time_,0);
        EXPECT_EQ(underTest_->_adjacencyList.front()->back_edge_weight_,0);

        EXPECT_NEAR(underTest_->_adjacencyList.back()->crdnts_.x_,4.1F, 0.00001F);
        EXPECT_NEAR(underTest_->_adjacencyList.back()->crdnts_.y_,5.0F,0.00001F);
        EXPECT_NEAR(underTest_->_adjacencyList.back()->crdnts_.time_,0.0F,0.00001F);
        EXPECT_NEAR(underTest_->_adjacencyList.back()->back_edge_weight_,2.2F,0.00001F);
    }

    TEST_F(Graph_test, ADDNODE_LINK_WITH_BASE_CONSTRUCTOR_TEST)
    {
        underTest_->addNode(4.1, 5, 0, 2.2F);
        underTest_->addNode(underTest_->_adjacencyList.back(), 8.2F, 10.0F ,1.0F, 4.4F);

        //Ensure proper assignment
        EXPECT_NEAR(underTest_->_adjacencyList.back()->crdnts_.x_,8.2F, 0.00001F);
        EXPECT_NEAR(underTest_->_adjacencyList.back()->crdnts_.y_,10.0F,0.00001F);
        EXPECT_NEAR(underTest_->_adjacencyList.back()->crdnts_.time_,1.0F,0.00001F);
        EXPECT_NEAR(underTest_->_adjacencyList.back()->back_edge_weight_,4.4F,0.00001F);

        //Ensure proper adjacency list conections
        EXPECT_EQ(underTest_->_adjacencyList.front()->fwd_node_.front(), 
                    underTest_->_adjacencyList.back()->back_node_);
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
        underTest2_->addNode(1.1, 2.1, 0, 1.0F);
        handle_ = underTest2_->_adjacencyList.back();
        underTest2_->addNode(handle_, 1.2, 2.2, 0, 3.0F);
        handle_ = underTest2_->_adjacencyList.front();
        underTest2_->addNode(handle_, 2.1, 1.1, 0, 2.0F);
        underTest2_->addNode(2.2, 1.2, 0, 4.0F);
        underTest2_->addNode(handle_, 3.1F, 3.2F, 0, 5.0F);

        /* Test Head-to-Tail */
        EXPECT_EQ(underTest2_->_adjacencyList.front()->fwd_node_.size(), 3);
        EXPECT_EQ(underTest2_->_adjacencyList.back()->back_node_, 
                  underTest2_->_adjacencyList.front());
        EXPECT_NEAR(underTest2_->_adjacencyList.back()->back_edge_weight_,5.0F, 0.00001F);
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
        underTest2_->addNode(1.1, 2.1, 0, 1.0F);
        handle_ = underTest2_->_adjacencyList.back();
        underTest2_->addNode(handle_, 1.2, 2.2, 0, 3.0F);
        handle_ = underTest2_->_adjacencyList.front();
        underTest2_->addNode(handle_, 2.1, 1.1, 0, 2.0F);
        underTest2_->addNode(2.2, 1.2, 0, 4.0F);
        underTest2_->addNode(handle_, 3.1F, 3.2F, 0, 5.0F);
        EXPECT_EQ(underTest2_->_adjacencyList.size(), 6);

        /* Locate Node #4 by searching for back edge weight */
        auto idx = -1;
        std::vector<Node*> next;
        Node* prev;  
        for(const auto &iter : underTest2_->_adjacencyList)
        {
            idx++;
            if(iter->back_edge_weight_ == 2.0F)
            {
                // Grab the node we want deleted from the graph for this test
                handle_ = underTest2_->_adjacencyList[idx];

                // Grab the soon-to-be deleted node's forward graph links
                next = underTest2_->_adjacencyList[idx]->fwd_node_;
                std::sort(next.begin(), next.end());

                // Grab the upstream node
                prev = underTest2_->_adjacencyList[idx]->back_node_;
                break;
            }
        }

        /* Delete Node #4 */
        underTest2_->deleteNode(handle_);

        /* Check that Graph Size Reduced by 1 */
        EXPECT_EQ(underTest2_->_adjacencyList.size(), 5);

        /* Check that Node 4's fwd connections migrated to the 
        previous (upstream) node */
        /*----------------------------------------------------------------*/

        /* After the Deletion, get the new index of the previous node-link*/
        auto find_prev = std::find(underTest2_->_adjacencyList.begin(), underTest2_->_adjacencyList.end(), prev);
        auto prev_idx = std::distance(underTest2_->_adjacencyList.begin(), find_prev);
        
        /* Sort the forward cnnection vector to enable 'std::includes' comparison*/
        std::sort(underTest2_->_adjacencyList[prev_idx]->fwd_node_.begin(), 
                  underTest2_->_adjacencyList[prev_idx]->fwd_node_.end());

        /* Check that the deleted Node's forward connections 
           have indeed migrated to the previous Node*/
        auto check = std::includes(underTest2_->_adjacencyList[prev_idx]->fwd_node_.begin(),
                                        underTest2_->_adjacencyList[prev_idx]->fwd_node_.end(),
                                        next.begin(), next.end());
        EXPECT_EQ(check, true);
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
        Node* head = underTest2_->_adjacencyList.front();
        underTest2_->addNode(1.1, 2.1, 0, 1.0F);
        handle_ = underTest2_->_adjacencyList.back();
        underTest2_->addNode(handle_, 1.2, 2.2, 0, 3.0F);
        handle_ = underTest2_->_adjacencyList.front();
        underTest2_->addNode(handle_, 2.1, 1.1, 0, 2.0F);
        underTest2_->addNode(2.2, 1.2, 0, 4.0F);
        underTest2_->addNode(handle_, 3.1F, 3.2F, 0, 5.0F);

        auto next = head->fwd_node_;

        //auto find_head = std::find(underTest2_->_adjacencyList.begin(), underTest2_->_adjacencyList.end(), prev);
        //auto head_idx = std::distance(underTest2_->_adjacencyList.begin(), find_prev);
    }
};



int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

