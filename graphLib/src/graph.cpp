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
        this->back_edge_weight_ = 0;
        this->fwd_node_.resize(1); 
        this->fwd_node_[this->fwd_node_.size()]= nullptr;
    }

    Node::Node(coordinate_t * _crdnts)
    {
        this->back_edge_weight_ = 0.001F;
        this->fwd_node_.resize(1); 
        this->fwd_node_[this->fwd_node_.size()]= nullptr;
        this->crdnts_.x_ = _crdnts->x_;
        this->crdnts_.y_ = _crdnts->y_;
        this->crdnts_.time_ = _crdnts->time_;

    }

    Node::Node(coordinate_t * _crdnts, double _back_edge_weight)
    {
        this->back_edge_weight_ = _back_edge_weight;
        this->fwd_node_.resize(1); 
        this->fwd_node_[this->fwd_node_.size()]= nullptr;
        this->crdnts_.x_ = _crdnts->x_;
        this->crdnts_.y_ = _crdnts->y_;
        this->crdnts_.time_ = _crdnts->time_;
    }

    coordinate_t Node::getCoordinate()
    {
        coordinate_t temp;
        if(this != nullptr)
        {
            temp = this->crdnts_;
        }
        return temp;
    }

    void Node::addFwdNode(Node * _cnnctn)
    {
        if(_cnnctn != nullptr)
        {
            this->fwd_node_.emplace_back(_cnnctn);
        }
    }

    Graph::Graph()
    {
        this->_adjacencyList.resize(1);
    }

    void Graph::addNode(coordinate_t * _crdnts, double _back_edge_weight)
    {
        Node tempNode(_crdnts, _back_edge_weight);
        int size = this->_adjacencyList.size();
        if(_crdnts != nullptr)
        {
            // Check if this is a new graph
            if(size < 1)
            {
                this->_adjacencyList.resize(1);
                #ifdef WARN
                    std::cout << "Initializing new RRT graph \n";
                #endif
            }
            else
            {
                // Make Node Connection
                size++;
                this->_adjacencyList.emplace_back(&tempNode);
                this->_adjacencyList[size - 1]->addFwdNode(this->_adjacencyList[size]);
                #ifdef WARN
                    std::cout << "No connection Node specified, connecting to tail \n";
                #endif
            }
        }
    }

    void Graph::addNode(Node * _link, coordinate_t * _crdnts, double _back_edge_weight)
    {
        Node tempNode(_crdnts, _back_edge_weight);

        if(_link != nullptr && _crdnts != nullptr)
        {
            this->_adjacencyList.emplace_back(&tempNode);
            _link->addFwdNode(this->_adjacencyList.back());
        }
    }

    void Graph::deleteNode(Node * _handle)
    {

    }

    int Graph::getIndex(Node* _handle)
    {
        coordinate_t crdnts;
        int idx = 0;
        if(_handle != nullptr)
        {
            auto temp = std::find(this->_adjacencyList.begin(), this->_adjacencyList.end(), _handle);
            idx = std::distance(this->_adjacencyList.begin(), temp);
        }
        return idx;
    }

    coordinate_t Graph::getCoordinate(Node * _handle)
    {
        int idx = 0;
        coordinate_t crdnts;

        if(_handle != nullptr)
        {
            idx = this->getIndex(_handle);
            crdnts = this->_adjacencyList[idx]->crdnts_;
        }
        return crdnts;
    }
  
    Graph::~Graph()
    {
    }

    // Function to add an edge to the graph
    // Parameters: src - source vertex
    // dest - destination vertex
    void Graph::addEdge(Node* _src, Node* _dest)
    {

        // Add the destination to the adjacency list of the
        // source
       // _adjList[_src].push_back(_dest);


    }

    // Function to print the adjacency list of the graph
    void Graph::printGraph()
    {
        int cnt = 0;
        coordinate_t crdnt;


        for(const auto &iter : this->_adjacencyList)
        {
            crdnt = iter->getCoordinate();
            std::cout << cnt << ":   x:" << crdnt.x_ << " y:" << crdnt.y_ << " time:" << crdnt.time_ << std::endl;
            cnt++;
        }

    }

}