import rclpy
import time
from mapr_4_student.grid_map import GridMap


class BFS(GridMap):
    def __init__(self):
        super(BFS, self).__init__()
        ###  IF YOU NEED SOME ADDITIONAL FILEDS IN BFS OBJECT YOU CAN INITIALIZED THEM HERE
        self.point_dictionary = {}
        self.directions = [[0, -1],[-1, 0],[0, 1],[1, 0]]
        self.previous_iteration = []
        self.end_point_found = False
    def search(self):
        if not self.point_dictionary:
            self.point_dictionary[self.start] = [self.start]
            self.map.data[self.map.info.width*(self.start[1])+self.start[0]] = 50
            self.previous_iteration.append(self.start)
        while not self.end_point_found:
            points_to_check = self.previous_iteration
            self.previous_iteration = []
            for point in points_to_check:
                for direction in self.directions:
                    if self.map.data[self.map.info.width*(point[1]+direction[1])+point[0]+direction[0]] == 0:
                       self.map.data[self.map.info.width*(point[1]+direction[1])+point[0]+direction[0]] = 50
                       self.point_dictionary[(point[0]+direction[0], point[1]+direction[1])] = point
                       self.previous_iteration.append((point[0]+direction[0], point[1]+direction[1]))
                       if self.previous_iteration[-1] == self.end:
                           self.end_point_found = True
                           break
            self.publish_visited()
            if self.end_point_found:
                break
            if not self.previous_iteration:
                break
        if self.end_point_found:
            path = [self.end]
            last_point = self.end
            while last_point != self.start:
                last_point = self.point_dictionary[last_point]
                path.append(last_point)
            path.append(self.start)
            self.publish_path(path)

        ### YOUR CODE GOES BELOW
        #
        #
        #
        # IMPLEMENT BREADTH FIRST SEARCH:
        # * save your search in self.map.data
        # * use self.publish_visited() to publish the map every time you visited a new cell
        # * let 100 represent walls, 50 visited cells (useful for visualization)
        # * start point is in self.start
        # * end point is in self.end
        #
        #
        ### YOUR CODE GOES ABOVE
        #pass  # DELETE THIS LINE IF THE CODE INSERTED


def main(args=None):
    rclpy.init(args=args)
    bfs = BFS()
    while not bfs.data_received():
        bfs.get_logger().info("Waiting for data...")
        rclpy.spin_once(bfs)
        time.sleep(0.5)

    bfs.get_logger().info("Start graph searching!")
    bfs.publish_visited()
    time.sleep(1)
    bfs.search()

if __name__ == '__main__':
    main()
