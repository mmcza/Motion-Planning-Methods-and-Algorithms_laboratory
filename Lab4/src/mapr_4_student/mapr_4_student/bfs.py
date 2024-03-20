import rclpy
import time
from mapr_4_student.grid_map import GridMap


class BFS(GridMap):
    def __init__(self):
        super(BFS, self).__init__()
        ###  IF YOU NEED SOME ADDITIONAL FILEDS IN BFS OBJECT YOU CAN INITIALIZED THEM HERE

    def search(self):
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
        pass  # DELETE THIS LINE IF THE CODE INSERTED


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
