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
 * @file graph.h
 * @brief Linked list to be used in an RRT Graph
 * @author Ryan Shedlock <rmshedlock@gmail.com>
 * @version 1.0
 */
#ifndef Graph_H_
#define Graph_H_

#include <stddef.h>
#include <string.h>
#include <cstdint>
#include <iostream>

namespace rrt
{
    /**
     * @brief      Structure defining a node in the linked list
     *
     */
    class Node
        {
            public:

            /**
             * @brief      Creates a storage class with the desired number of edges
             *
             */
            Node(std::uint8_t _edge_count);

            /**
             * @brief      Destroys the object.
             */
            ~Node();

            /**
             * @brief      Destroys the object.
             */
            Node *GetEdge(std::uint8_t _index);
            
            double x_;
            double y_;
            double z_;

            private:
            std::uint8_t edge_count_;
            Node *forward_edge_[16];
        };

    class Graph
        {
            public:

            /**
             * @brief      Constructs a new instance.
             *
             */
            Graph();

            /**
             * @brief      Destroys the object.
             */
            ~Graph();

            /**
             * @brief      Adds a Node
             */
            void AddNode();
            
            private:
            Node *head;
        };
}

#endif /* Graph_H_ */