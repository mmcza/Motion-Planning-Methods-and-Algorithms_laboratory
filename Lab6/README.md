# RRT (Rapidly-exploring Random Tree) Algorithm

> [!NOTE]
> Based on [Bartłomiej Kulecki's repository](https://github.com/BartlomiejKulecki/mapr_6_student)

Dependencies:
```
sudo apt-get install ros-humble-nav2-map-server ros-humble-nav2-lifecycle-manager
```

Put the src directory inside your ros2 workspace.

## Task - Create RRT Algorithm

![RRT Algorithm](/Pictures/MIAPR_Lab6_RRT.gif)

To run the code for the RRT Algorithm (implemented inside [rrt_vertices.py](/Lab6/src/mapr_6_student/mapr_6_student/rrt_vertices.py) file) you have to type the following commands when being inside your ros2 workspace:

```
colcon build --symlink-install
```

``` 
source install/setup.bash 
```

``` 
ros2 launch mapr_6_student rrt_launch.py vertices:=True
```

As a result RViz will open and you will see how the RRT Algorithm works.

Inside the [rrt_vertices.py](/Lab6/src/mapr_6_student/mapr_6_student/rrt_vertices.py) script you can change self.step and find what difference it makes if the distance between last known point is shorter or longer (the algorithm will find different paths!!!).