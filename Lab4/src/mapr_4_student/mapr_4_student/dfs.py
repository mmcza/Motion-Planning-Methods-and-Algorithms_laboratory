import rclpy
import time
from mapr_4_student.grid_map import GridMap


class DFS(GridMap):
    def __init__(self):
        super(DFS, self).__init__()
        ###  IF YOU NEED SOME ADDITIONAL FIELDS IN DFS OBJECT YOU CAN INITIALIZE THEM HERE
        self.point_list = []
        self.directions = [[0, -1],[-1, 0],[0, 1],[1, 0],[0, 0]]
	
    def search(self):
        if not self.point_list:
            self.point_list.append(self.start)
            self.map.data[self.map.info.width*(self.point_list[-1][1])+self.point_list[-1][0]] = 50
        while self.point_list[-1] != self.end:
                for i, direction in enumerate(self.directions):
                   if i < 4:
                      if self.map.data[self.map.info.width*(self.point_list[-1][1]+direction[1])+self.point_list[-1][0]+direction[0]] == 0:
                        self.point_list.append((self.point_list[-1][0]+direction[0], self.point_list[-1][1]+direction[1]))
                        self.map.data[self.map.info.width*(self.point_list[-1][1])+self.point_list[-1][0]] = 50
                        break  
                   else:
                      self.point_list.pop(-1)              
                #self.get_logger().info(self.point_list)        
                self.publish_visited()
        self.publish_path(self.point_list)
    	   
           
        ### YOUR CODE GOES BELOW
        #
        #
        #
        # IMPLEMENT DEPTH FIRST SEARCH:
        # * save your search in self.map.data
        # * use self.publish_visited() to publish the map every time you visited a new cell
        # * let 100 represent walls, 50 visited cells (useful for visualization)
        # * start point is in self.start
        # * end point is in self.end
        #
        #
        ### YOUR CODE GOES ABOVE
        #print(self.map)
        #pass  # DELETE THIS LINE IF THE CODE IS INSERTED


def main(args=None):
    rclpy.init(args=args)
    dfs = DFS()
    while not dfs.data_received():
        dfs.get_logger().info("Waiting for data...")
        rclpy.spin_once(dfs)
        time.sleep(0.5)

    dfs.get_logger().info("Start graph searching!")
    dfs.publish_visited()
    time.sleep(1)
    dfs.search()

if __name__ == '__main__':
    main()
