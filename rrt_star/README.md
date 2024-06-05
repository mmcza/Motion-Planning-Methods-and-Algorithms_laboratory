# RRT* (Rapidly-exploring Random Tree Star) Algorithm

> [!NOTE]
> RRT* algorithm implemented as nav2 plugin is available [here](https://github.com/mmcza/TurtleBot-RRT-Star)

> [!NOTE]
> Based on [Bartłomiej Kulecki's repository](https://github.com/BartlomiejKulecki/mapr_6_student)

Dependencies:
```
sudo apt-get install ros-humble-nav2-map-server ros-humble-nav2-lifecycle-manager
```

Put the src directory inside your ros2 workspace.

## Task - Create RRT Algorithm

![RRT* Algorithm](/Pictures/rrt_star_example.gif)

To run the code for the RRT Algorithm (implemented inside [rrt_vertices.py](/rrt_star/src/rrt_star/rrt_star/rrt_vertices.py) file) you have to type the following commands when being inside your ros2 workspace:

```
colcon build --symlink-install
```

``` 
source install/setup.bash 
```

``` 
ros2 launch rrt_star rrt_launch.py vertices:=True
```

As a result RViz will open and you will see how the RRT* Algorithm works.