/**
 * MIT License
 *
 * Copyright (c) 2024 Ryan Shedlock
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
 * @brief Linked list to be used in an RRT Graph
 * @author Ryan Shedlock <rmshedlock@gmail.com>
 * @version 1.0
 */

#include "graph.h"
#define WARN 0

namespace rrt
{
    coordinate_t::coordinate_t() : x_(0.0F), y_(0.0F), time_(0.0F)
    {
        this->dim_ = 2;
    }
    coordinate_t::coordinate_t(double _x, double _y) : x_(_x), y_(_y), time_(0.0F)
    {
        this->dim_ = 2;
    }
    coordinate_t::coordinate_t(double _x, double _y, double _time) : x_(_x), y_(_y), time_(0.0F)
    {
        this->dim_ = 3;
    }

    int coordinate_t::getDimension()
    {
        return this->dim_;
    }
    
    Node::Node() 
    {
        this->back_edge_weight_ = 0.0F;
        this->fwd_node_.resize(1); 
        this->fwd_node_[this->fwd_node_.size()] = nullptr;
        this->back_node_ = nullptr;
        this->crdnts_.x_ = 0.0F;
        this->crdnts_.y_ = 0.0F;
        this->crdnts_.time_ = 0.0F;
    }

    Node::Node(coordinate_t * _crdnts)
    {
        this->back_edge_weight_ = 0.0F;
        this->fwd_node_.resize(1); 
        this->fwd_node_[this->fwd_node_.size()] = nullptr;
        this->back_node_ = nullptr;
        this->crdnts_.x_ = _crdnts->x_;
        this->crdnts_.y_ = _crdnts->y_;
        this->crdnts_.time_ = _crdnts->time_;

    }

    Node::Node(coordinate_t * _crdnts, double _back_edge_weight)
    {
        this->back_edge_weight_ = _back_edge_weight;
        this->fwd_node_.resize(1); 
        this->fwd_node_[this->fwd_node_.size()] = nullptr;
        this->back_node_ = nullptr;
        this->crdnts_.x_ = _crdnts->x_;
        this->crdnts_.y_ = _crdnts->y_;
        this->crdnts_.time_ = _crdnts->time_;
    }

    Node::Node(double _x, double _y, double _time, double _back_edge_weight) 
    {
        this->back_edge_weight_ = _back_edge_weight;
        this->fwd_node_.resize(1); 
        this->fwd_node_.front()= nullptr;
        this->crdnts_.x_ = _x;
        this->crdnts_.y_ = _y;
        this->crdnts_.time_ = _time;
        this->dimension_ = 3;
    }

    Node::Node(double _x, double _y, double _back_edge_weight)
    {
        this->back_edge_weight_ = _back_edge_weight;
        this->fwd_node_.resize(1); 
        this->fwd_node_.front()= nullptr;
        this->crdnts_.x_ = _x;
        this->crdnts_.y_ = _y;
        this->crdnts_.time_ = 0.0F;
        this->dimension_ = 2;
    }

    coordinate_t Node::getCoordinate()
    {
        coordinate_t temp;
        if(this != nullptr)
        {
            temp = this->crdnts_;
            std::cout << "Node->GetCoordinate x:" << temp.x_ << " y:" << temp.y_ << " time:" << temp.time_ << std::endl;
        }
        else
        {
            std::cout << "Node->GetCoordinate nullptr!!!" << std::endl;
        }
        return temp;
    }

    void Node::addFwdNode(Node* _cnnctn)
    {
        this->fwd_node_.push_back(_cnnctn);
    }

    void Node::printNode()
    {
        //If Head
        if(this->back_node_ == nullptr)
        {
            std::cout << "Node ID:HEAD" << " X:" << this->crdnts_.x_ << " Y:" << this->crdnts_.y_ << std:: endl;
            std::cout << "Back Connection/Weight:NULL/" << this->back_edge_weight_ << std:: endl;
            std::cout << "Forward Connection \n" << std::endl;
            std::cout << "   |\n   ---> ";
            std::cout << "Fwd Connection ID:NULL" << std::endl;
        }
        else
        {
            std::cout << "Node ID:" << this << " X:" << this->crdnts_.x_ << " Y:" << this->crdnts_.y_ << std:: endl;
            std::cout << "Back Connection/Weight:" << this->back_node_ << "/" << this->back_edge_weight_ << std:: endl;
            std::cout << "Forward Connection \n" << std::endl;

            for(const Node* iter : this->fwd_node_)
            {
                std::cout << "   |\n   ---> ";
                std::cout << "Fwd Connection ID:" << iter << std::endl;
            }
            std::cout << std::endl;
        }
    }

    Graph::Graph()
    {
        this->_adjacencyList.resize(1);
    }

    void Graph::addNode(double _x, double _y, double _time, double _back_edge_weight)
    {
        int size;
        this->_adjacencyList.emplace_back(new Node(_x,_y,_time, _back_edge_weight));
        size = this->_adjacencyList.size();

        if(size <= 1)
        {
            #ifdef WARN
            std::cout << "Something went wrong, Adjacentcy List Range Error\n";
            #endif
        }
        else 
        {
            this->addEdge(this->_adjacencyList[size-1], this->_adjacencyList.back());
            #ifdef WARN
                std::cout << "No connection Node specified, connecting to tail \n";
            #endif
        }
    }

    void Graph::addNode(Node* _link, double _x, double _y, double _time, double _back_edge_weight)
    {
        this->_adjacencyList.emplace_back(new Node(_x,_y,_time, _back_edge_weight));
        this->addEdge(_link, this->_adjacencyList.back());
    }

    void Graph::deleteNode(Node* _handle)
    {
        int idx = this->getIndex(_handle);
        Node* back_link = this->_adjacencyList[idx]->back_node_;

        // Migrate Deleted Node's Forward Links to the new back link
        for(const auto &iter_b : this->_adjacencyList[idx]->fwd_node_)
        {
            back_link->fwd_node_.push_back(iter_b);
        }

        // Update Back Links for all the forward connections
        // aka do double linked list house keeping
        for(const auto &iter_f : back_link->fwd_node_)
        {
            iter_f->back_node_ = back_link;
        }
        
        // Delete the node from adjacency list
        this->_adjacencyList.erase(this->_adjacencyList.begin() + idx);
    }

    int Graph::getIndex(Node* _handle)
    {
        int idx = 0;
        if(_handle != nullptr)
        {
            auto temp = std::find(this->_adjacencyList.begin(), this->_adjacencyList.end(), _handle);
            idx = std::distance(this->_adjacencyList.begin(), temp);
        }
        return idx;
    }

    coordinate_t Graph::getCoordinate(Node* _handle)
    {
        int idx = 0;
        coordinate_t crdnts;

        idx = this->getIndex(_handle);
        crdnts = this->_adjacencyList[idx]->crdnts_;
        
        return crdnts;
    }
  
    Graph::~Graph()
    {
    }

    // Method to add an edge to the graph
    // Parameters: src - source vertex
    // dest - destination vertex
    void Graph::addEdge(Node* _src, Node* _dest)
    {
        _src->addFwdNode(_dest);
        _dest->back_node_ = _src;
    }

    // Function to print the adjacency list of the graph
    void Graph::printGraph()
    {
        for(const auto &iter : this->_adjacencyList)
        {
            iter->printNode();
        }
    }

}