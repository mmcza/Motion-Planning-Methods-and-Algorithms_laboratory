# A Star (A*) Algorithm

> [!NOTE]
> Based on [Bartłomiej Kulecki's repository](https://github.com/BartlomiejKulecki/mapr_5_student.git)

Dependencies:
```
sudo apt-get install ros-humble-nav2-map-server ros-humble-nav2-lifecycle-manager
```

Put the src directory inside your ros2 workspace.

## Task - Create A Star (A*) Algorithm

![A Star Algorithm](/Pictures/MIAPR_Lab5_A_star.gif)

To run the code for the A* Algorithm (implemented inside [astar.py](/Lab5/src/mapr_5_student/mapr_5_student/astar.py) file) you have to type the following commands when being inside your ros2 workspace:

```
colcon build 
```

``` 
source install/setup.bash 
```

``` 
ros2 launch mapr_5_student astar_launch.py
```

As a result RViz will open and you will see how the A* Algorithm works.

Inside the [astar.py](/Lab5/src/mapr_5_student/mapr_5_student/astar.py) script you can change the metric used for heuristic. There are 3 different metrics implemented:
- Manhattan Distance = $|x_{a}-x_{b}|+|y_{a}-y_{b}|$
- Euclidean Distance = $\sqrt{(x_{a}-x_{b})^{2}+(y_{a}-y_{b})^{2}}$
- Chebyshev Distance = $\max (|x_{a}-x_{b}|,|y_{a}-y_{b}|)$