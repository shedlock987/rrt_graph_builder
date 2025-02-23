parent_path=$( cd "$(dirname "${BASH_SOURCE[0]}")" ; cd .. ; pwd -P)

#Get Project Directory
echo Building RRT graph for linux target
cd "$parent_path"

#cmake
echo Running Cmake
cmake /build .

#Build
echo Building App
make

#Done
echo Don
#Running App
echo Running App
cd test
echo enter here
echo Complete

