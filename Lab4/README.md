# Breadth-First Search (BFS) and Depth-First Search (DFS) 

> [!NOTE]
> Based on [Bartłomiej Kulecki's repository](https://github.com/BartlomiejKulecki/mapr_4_student.git)

Dependencies:
```
sudo apt-get install ros-humble-nav2-map-server ros-humble-nav2-lifecycle-manager
```

Put the src directory inside your ros2 workspace.

## Task 1 - Depth-First Search

![DFS Algorithm](/Pictures/Lab4_DFS.gif)

To run the code for the DFS Algorithm (implemented inside [dfs.py](/Lab4/src/mapr_4_student/mapr_4_student/dfs.py) file) you have to type the following commands when being inside your ros2 workspace:

``` colcon build ```

``` source install/setup.bash ```

``` ros2 launch mapr_4_student dfs_launch.py ```

As a result RViz will open and you will see how the DFS Algorithm works.

## Task 2 - Breadth-First Search

![BFS Algorithm](/Pictures/Lab4_BFS.gif)

To run the code for the BFS Algorithm (implemented inside [bfs.py](/Lab4/src/mapr_4_student/mapr_4_student/bfs.py) file) you have to type the following commands when being inside your ros2 workspace:

``` colcon build ```

``` source install/setup.bash ```

``` ros2 launch mapr_4_student bfs_launch.py ```

As a result RViz will open and you will see how the BFS Algorithm works.

The BFS algorithm allows you to find the optimal route to certain point and DFS might do it but only if in each step it goes in correct way.