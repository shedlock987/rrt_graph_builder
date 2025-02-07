parent_path=$( cd "$(dirname "${BASH_SOURCE[0]}")" ; pwd -P)

cd "$parent_path"
./clean_rrt_linux.sh
./build_rrt_linux.sh