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
 * @file graph.cpp
 * @brief Graph in the form of an Adjacency List
 * @author Ryan Shedlock <rmshedlock@gmail.com>
 * @version 1.0
 */

#include "graph.h"
#define WARN 0

namespace rrt
{
    
    Node::Node() 
    {
        setBackEdgeWeight(0.0F);
        setCrdnts(0.0F, 0.0F , 0.0F);
    }

    Node::Node(coordinate_t _crdnts)
    {
        /// Ensure real value
        setBackEdgeWeight(1.0F); 
        setCrdnts(_crdnts);

    }

    Node::Node(coordinate_t _crdnts, double _back_edge_weight)
    {
        setBackEdgeWeight(_back_edge_weight);
        setCrdnts(_crdnts);
    }

    Node::Node(double _x, double _y, double _time, double _back_edge_weight) 
    {
        setBackEdgeWeight(_back_edge_weight);
        setCrdnts(_x, _y, _time);
    }

    Node::Node(double _x, double _y, double _back_edge_weight)
    {
        setBackEdgeWeight(_back_edge_weight);
        setCrdnts(_x, _y, 0.0F);
    }

    Node::Node(const Node &_copy) : back_edge_weight_(_copy.back_edge_weight_), back_node_(_copy.back_node_),
                                    crdnts_(_copy.crdnts_), fwd_node_(_copy.fwd_node_)
    {
    }

    Node::~Node()
    {
        fwd_node_.clear();
        back_node_ = nullptr;
    }

    void Node::addFwdNode(Node* _cnnctn)
    {
        fwd_node_.push_back(_cnnctn);
    }

    void Node::debugPrintNode()
    {
        bool init = false;

        std::cout << std::endl;

        std::cout << this << " X:" << std::get<0>(crdnts_) << " Y:" << std::get<1>(crdnts_)  << " time:" << std::get<2>(crdnts_) 
                  << " Back Weight:" << backEdgeWeight() << std::endl;
        std::cout << "Connections:" << std::endl << std::endl;

        if(BackCnnctn() == nullptr)
        {
            std::cout << "        Back Connection ID:";
            std::cout << BackCnnctn();
            std::cout << " <--- O";
            std::cout << " ---> Fwd Connection ID:" << fwd_node_.front() << std::endl;
        }
        else
        {
            std::cout << "Back Connection ID:";
            std::cout << BackCnnctn();
            std::cout << " <--- O";
            if(fwd_node_.size() > 0)
            {
                std::cout << " ---> Fwd Connection ID:" << fwd_node_.front() << std::endl;
            }
        }


        for(const auto &iter : fwd_node_)
        {
            if(init == false)
            {
                init = true;
            }
            else
            {
                std::cout << "                                       |\n";
                std::cout << "                                       O ---> "; 
                std::cout << "Fwd Connection ID:" << iter << std::endl;    
            }  
        }
        std::cout << std::endl;
    }

    double Node::xCrdnt()
    {
        return std::get<0>(crdnts_);
    }

    double Node::yCrdnt()
    {
        return std::get<1>(crdnts_);
    }

    double Node::time()
    {
        return std::get<2>(crdnts_);
    }

    void Node::setCrdnts(double _x, double _y, double _tm)
    {
        crdnts_ = std::make_tuple(_x, _y, _tm);
    }

    void Node::setCrdnts(coordinate_t _crdnts)
    {
        crdnts_ = _crdnts;
    }

    coordinate_t Node::Crdnts()
    {
        return crdnts_;
    }

    Node* Node::BackCnnctn()
    {
        return back_node_;
    }

    void Node::setBackCnnctn(Node* _cnnctn)
    {
        back_node_ = _cnnctn;;
    }

    double Node::backEdgeWeight()
    {
        return back_edge_weight_;
    }

    void Node::setBackEdgeWeight(double _back_edge_weight)
    {
        back_edge_weight_ = _back_edge_weight;
    }

    Graph::Graph()
    {      
        addNode(std::make_tuple(0.0F, 0.0F, 0.0F), 0.0F);
        initCmplt_ = true;
    }
    
    Graph::Graph(double _xHead, double _yHead)
    {
        addNode(std::make_tuple(_xHead, _yHead, 0.0F), 0.0F);
        initCmplt_ = true;
    }

    void Graph::addNode(Node::coordinate_t _point, double _back_edge_weight)
    {
        int size;
        size = adjacencyList_.size();

        adjacencyList_.emplace_back(new Node(_point, _back_edge_weight));
        if(initCmplt_ && size >= 1)
        {
            size++;
            addEdge(adjacencyList_.at(size - 2), adjacencyList_.back());
        }
        else
        {
            initCmplt_ = true;
        }
        

    }

    void Graph::addNode(Node* _link, Node::coordinate_t _point, double _back_edge_weight)
    {
        if(initCmplt_)
        {
            adjacencyList_.emplace_back(new Node(_point, _back_edge_weight));
            addEdge(_link, adjacencyList_.back());
        }
    }

    void Graph::deleteNode(Node* _handle)
    {
        int idx = getIndex(_handle);

        /// Check that this is an actual graph 
        if(adjacencyList_.size() > 1)
        {
            /// Check if youre deleting the HEAD 
            if(adjacencyList_.at(idx)->BackCnnctn() == nullptr)
            {
                /// Find the Foward Edge with the smallest Weight
                /// This will be the new Head 

                std::vector<double> list;
                for (const auto& i : adjacencyList_.at(idx)->fwd_node_)
                {
                    list.push_back(i->backEdgeWeight());
                }

                auto temp = std::min_element(list.begin(), list.end());
                auto min_idx = std::distance(list.begin(), temp);
                auto new_head = adjacencyList_.at(idx)->fwd_node_.at(min_idx);

                /// Add Old-HEAD's Fwd Connections to New-HEAD 
                for(const auto &iter : adjacencyList_.at(idx)->fwd_node_)
                {
                    if(iter != new_head)
                    {
                        new_head->fwd_node_.push_back(iter);
                    }
                }
                new_head->setBackCnnctn(nullptr);
                new_head->setBackEdgeWeight(0.0F);

                /// Delete Old-HEAD node from adjacency list 
                adjacencyList_.erase(adjacencyList_.begin());
                
                /// Housekeeping: Make sure New-HEAD is index 0 
                int temp_head_idx = getIndex(new_head);
                Node* cpy = adjacencyList_.at(temp_head_idx);
                adjacencyList_.erase(adjacencyList_.begin() + temp_head_idx);
                adjacencyList_.insert(adjacencyList_.begin(), cpy);

                /// Delete Old-HEAD 
                temp_head_idx = getIndex(_handle);
                //std::cout << "Old Head Index: " << temp_head_idx << " Old Head: " << adjacencyList_.at(temp_head_idx] << std::endl;
                //adjacencyList_.erase(adjacencyList_.begin() + temp_head_idx + 1); 
            }
            else 
            {
                auto upstream = adjacencyList_.at(idx)->BackCnnctn();
                auto upstream_idx = getIndex(upstream);
                //std::cout << "INITIAL fwd connection size on upstream: " << upstream->fwd_node_.size() << std::endl;
                /// Migrate Deleted Node's Forward Links to the new back link 
 
                for(const auto &iter_b : adjacencyList_.at(idx)->fwd_node_)
                {
                    if(iter_b != nullptr) 
                    {
                        adjacencyList_.at(upstream_idx)->fwd_node_.push_back(iter_b);
                    }
                }
                //std::cout << "AFTER MIGRATION fwd connection size on upstream: " << upstream->fwd_node_.size() << std::endl;
                /// Update Back Links for all the forward connections
                /// aka do double linked list house keeping 
                for(const auto &iter_f : adjacencyList_.at(idx)->fwd_node_)
                {
                    iter_f->setBackCnnctn(adjacencyList_.at(idx));
                }
                
                /// Break the forward connection to the deleted node
                if(upstream != nullptr)
                {
                    /// Remove the deleted node from the upstream node's forward connections
                    auto fwd_link = std::remove(upstream->fwd_node_.begin(), upstream->fwd_node_.end(), _handle);
                    upstream->fwd_node_.erase(fwd_link);
                }
                //std::cout << "AFTER fwd connection size on upstream: " << upstream->fwd_node_.size() << std::endl;
                /// Finally, Delete the node from adjacency list 
                adjacencyList_.erase(adjacencyList_.begin() + idx);
            }
        }
    }

    int Graph::getIndex(Node* _handle)
    {
        int idx = 0;
        if(_handle != nullptr)
        {
            auto temp = std::find(adjacencyList_.begin(), adjacencyList_.end(), _handle);
            idx = std::distance(adjacencyList_.begin(), temp);
        }
        return idx;
    }

    Node::coordinate_t Graph::getCoordinate(Node* _handle)
    {

        auto idx = getIndex(_handle);
        return adjacencyList_.at(idx)->crdnts_;
    }
  
    Graph::~Graph()
    {
        for(auto &iter : adjacencyList_)
        {
            delete iter;
        }
        adjacencyList_.clear();
    }

    /// Method to add an edge to the graph
    /// Parameters: src - source vertex
    /// dest - destination vertex
    void Graph::addEdge(Node* _src, Node* _dest)
    {
        _src->fwd_node_.push_back(_dest);
        _dest->setBackCnnctn(_src);
    }

    /// Function to print the adjacency list of the graph
    void Graph::printGraph()
    {
        for(const auto &iter : adjacencyList_)
        {
            iter->debugPrintNode();
        }
    }

}