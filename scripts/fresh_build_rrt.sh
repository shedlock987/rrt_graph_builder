parent_path=$( cd "$(dirname "${BASH_SOURCE[0]}")" ; pwd -P)

cd "$parent_path"
./clean.sh
./build_rrt.sh