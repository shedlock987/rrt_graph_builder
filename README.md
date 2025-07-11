# rrt_graph_builder
WORK IN PROGRESS
Generates a constrained RRT graph in the form of a adjacency list. Intended for Robotic Path Planning.

# Getting Started
These instructions will help you generate the necessary documentation for using this package, and list the required dependencies.

# Documentation
The documentation for this project is Doxygen based. To generate, execute the following commands:

cd <path>/rrt_graph_builder

doxygen Doxyfile
  
# Dependencies
The follwing dependencies are required, and can be installed accordingly.

sudo apt install doxygen

sudo apt install cmake

sudo apt install libgtest-dev

sudo apt install build-essential g++ python3-dev

sudo apt install libboost-all-dev (For Visualization)

# Running the tests
To compile unit and pipeline tests, run the following script:

./scripts/run_tests.sh

The graphTest test verifies basic functionality 

graph.cpp 

## Built With

* [cmake](https://cmake.org/download/) - Build tool used for compiling this project
* [Google Test](https://github.com/google/googletest) - Unit testing framework


## Authors

* **Ryan Shedlock**

## License

This project is licensed under the MIT License - see the LICENSE file for details


