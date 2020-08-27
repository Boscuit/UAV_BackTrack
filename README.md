# UAV_BackTrack
### Building the nodes for Test

1. Add the path including  UAV_BackTrack project path to the ROS_PACKAGE_PATH environment variable. Open .bashrc file and add at the end the following line. Replace PATH by the folder where you cloned UAV_BackTrack:

  ```
  export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:PATH
  ```

```
rosrun UAV_BackTrack Test Vocabulary/ORBvoc.txt Examples/Config/EuRoC_Mono.yaml Data/cam0 Data/cam0/data.csv Data/vicon0/data.csv
```

