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
 * @file linked_list.h
 * @brief Linked list to be used in an RRT Graph
 * @author Ryan Shedlock <rmshedlock@gmail.com>
 * @version 1.0
 */
#ifndef LINKED_LIST_H_
#define LINKED_LIST_H_

#include <stddef.h>
#include <string.h>

namespace rrt
{
    /**
     * @brief      Structure defining a node in the linked list
     *
     */
    struct Node;

    class Linked_List
        {
            public:

            /**
             * @brief      Constructs a new instance.
             *
             */
            Linked_List();

            /**
             * @brief      Destroys the object.
             */
            ~Linked_List();

            /**
             * @brief      Adds a Node
             */
            void AddNode();
            
            private:
            Node *head;
        }
}

#endif /* LINKED_LIST_H_ */