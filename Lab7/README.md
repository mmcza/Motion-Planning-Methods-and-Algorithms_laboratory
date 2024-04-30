# Open Motion Planning Library in ROS2

> [!NOTE]
> Based on [Dominik Belter's repository](https://github.com/dominikbelter/ompl_example_2d)

Dependencies:
```
sudo apt-get install ros-humble-nav2-map-server ros-humble-nav2-lifecycle-manager
```

Download the example_maps:
```
cd ros2_ws/src
```

```
git clone --branch humble https://github.com/dominikbelter/example_maps
```

Put the src directory from this repository inside your ros2 workspace.

## Task - Update the algorithm to find path using occupancy map and assume the size of the robot to be 10x10 cells

![RRT Algorithm](/Pictures/MIAPR_Lab7.gif)

To run the code (implemented inside [rrt_vertices.py](/Lab7/src/ompl_example_2d/src/ompl_example_2d.cpp) file) you have to type the following commands when being inside your ros2 workspace:

```
colcon build --symlink-install
```

``` 
source install/setup.bash 
```

``` 
ros2 launch ompl_example_2d ompl_example_2d.launch.py
```

As a result RViz will open and you will see how the algorithm works.