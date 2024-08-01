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
 * @file linked_list.cpp
 * @brief Linked list to be used in an RRT Graph
 * @author Ryan Shedlock <rmshedlock@gmail.com>
 * @version 1.0
 */

#include "linked_list.h"
#define WARN 0

namespace rrt
{
    Node::Node(std::uint8_t _edge_count) 
    {    
        if(_edge_count <= 16)
        {
            edge_count_ = _edge_count;
        }
        else if (_edge_count <= 0)
        {
            edge_count_ = 1;

            #ifdef WARN
            std::cout << "Edge count zero, reverting to size 1" << std::endl;
            #endif
        }
        else 
        {
            edge_count_ = 16;

            #ifdef WARN
            std::cout << "Edge count exceeds range, reverting to size 16" << std::endl;
            #endif
        } 
    }

    Node::~Node() {}

    Node Node::*GetEdge(std::uint8_t _index) {}

    Linked_List::Linked_List(){}


    Linked_List::~Linked_List(){};


    void Linked_List::AddNode()
    {
        //check edge count to make sure its not exceeded
        
    };
}