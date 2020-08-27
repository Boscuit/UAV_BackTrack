# UAV_BackTrack

### Modify "predefine.h" for a customized test

```c++
#define START INDEX 10
#define STOP_INDEX 30
#define TARGET_INDEX 20
#define AUTO_ITERATION 50
#define STORE_RESULT
```

If AUTO_ITERATION is defined, the program will generate command 'p' for certain times.  Images from START_INDEX to STOP_INDEX will be orderly processed. In auto mode, by default, there will be no images shown.

if STORE_RESULT is defined, the program will store all valid estimation errors between  successfully estimated target pose and desired ground truth based on current ground truth.

### Building the nodes for Test

1. Add the path including  UAV_BackTrack project path to the ROS_PACKAGE_PATH environment variable. Open .bashrc file and add at the end the following line. Replace PATH by the folder where you cloned UAV_BackTrack:

   ```shell
   export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:PATH
   ```

2. Go to this git repository folder. Build the project with `build.sh` and `build_ros.sh`

   ```sh
   cd ${path-to-UAV_BackTrack}
   ./build.sh
   ./build_ros.sh
   ```

### Run the test

Run following command

```shell
rosrun UAV_BackTrack Test path_to_vocabulary path_to_settings path_to_image_folder path_to_times_file path_to_groundtruth
```

Example usage:

```
rosrun UAV_BackTrack Test Vocabulary/ORBvoc.txt Examples/Config/EuRoC_Mono.yaml data/cam0/data data/cam0/data.csv data/vicon0/data.csv 
```

### Visualize in Rviz

Run Rviz with configuration `Examples/Config/6_17.rviz`
