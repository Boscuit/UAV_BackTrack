echo "Building ROS nodes"

cd Examples/ROS/UAV_BackTrack
mkdir build
cd build
cmake .. -DROS_BUILD_TYPE=Release
make -j
