parent_path=$( cd "$(dirname "${BASH_SOURCE[0]}")" ; cd .. ; pwd -P)

#Get Project Directory
echo Cleaning Build Files
cd "$parent_path"

rm -rf build

rm -rf CMakeFiles
rm -rf cmake_install.cmake
rm -rf Makefile
rm -rf CMakeCache.txt

cd graphLib
rm -rf CMakeFiles
rm -rf cmake_install.cmake
rm -rf Makefile
cd ..

cd rrtDemo
rm -rf CMakeFiles
rm -rf cmake_install.cmake
rm -rf Makefile
rm -rf rrtDemo
cd ..

cd test
rm -rf CMakeFiles
rm -rf cmake_install.cmake
rm -rf Makefile
rm -rf graphTest
rm -rf CMakeCache.txt
cd ..


rm -rf _deps
