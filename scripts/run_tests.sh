parent_path=$( cd "$(dirname "${BASH_SOURCE[0]}")" ; pwd -P)

cd "$parent_path"
./clean.sh

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
echo Done

#Running App
echo Running tests
cd test
./graphTest
echo Complete

