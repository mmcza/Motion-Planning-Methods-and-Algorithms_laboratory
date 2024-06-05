import rclpy
import time
from rrt_star.grid_map import GridMap
import numpy as np

np.random.seed(444)


class RRT(GridMap):
    def __init__(self):
        super(RRT, self).__init__()
        self.cost = {}
        self.step = 0.05
        self.neighborhood = 0.1
        self.max_iteration = 2000
        self.i = 0

    def check_if_valid(self, a, b):
        """
        Checks if the segment connecting a and b lies in the free space.
        :param a: point in 2D
        :param b: point in 2D
        :return: boolean
        """
        in_free_space = True
        points_between = np.linspace(a, b, num=100)
        for point in points_between:
            point = np.int_(point/self.resolution).tolist()
            
            if self.map.data[(point[1], point[0])] > 0 or point[0] > self.width/self.resolution or point[1] > self.height/self.resolution or point[0] < 0 or point[1]<0:
                in_free_space = False
                break

        return in_free_space

    def random_point(self):
        """
        Draws random point in 2D
        :return: point in 2D
        """
        x = self.width * np.random.random()
        y = self.height * np.random.random()
        return np.array([x, y])

    def find_closest(self, pos):
        """
        Finds the closest vertex in the graph to the pos argument

        :param pos: point id 2D
        :return: vertex from graph in 2D closest to the pos
        """

        closest_dist = self.width + self.height
        closest = None
        for key, value in self.parent.items():
            dist = np.linalg.norm(key - pos)
            if dist < closest_dist:
                closest = key
                closest_dist = dist
        return closest

    def new_pt(self, pt, closest):
        """
        Finds the point on the segment connecting closest with pt, which lies self.step from the closest (vertex in graph)

        :param pt: point in 2D
        :param closest: vertex in the tree (point in 2D)
        :return: point in 2D
        """
        direction_vector = closest - pt
        normalized_vector = direction_vector / np.linalg.norm(direction_vector)
        scaled_vector = normalized_vector * self.step
        pt = closest - scaled_vector
        return pt
    
    def find_and_update_neighbors(self, new_pt):
        for point in self.parent.keys():
            if np.linalg.norm(point - new_pt) < self.neighborhood:
                cost_diff = self.cost[tuple(point)] - (self.cost[tuple(new_pt)] + np.linalg.norm(point - new_pt))
                if cost_diff>0.0:
                    if self.check_if_valid(point, new_pt):
                        self.parent[tuple(point)] = tuple(new_pt)
                        self.cost[tuple(point)]-= cost_diff
                        points_to_update = []
                        points_to_update.append(point)
                        while True:
                            #self.get_logger().info("I'm in loop")
                            #self.get_logger().info(str(points_to_update))
                            num_of_points = len(points_to_update)
                            for key, value in self.parent.items():
                                if (value in points_to_update) and (key not in points_to_update):
                                    points_to_update.append(key)
                                    self.cost[tuple(key)] -= cost_diff
                            if len(points_to_update) == num_of_points:
                                break
    				

    def search(self):
        """
        RRT search algorithm for start point self.start and desired state self.end.
        Saves the search tree in the self.parent dictionary, with key value pairs representing segments
        (key is the child vertex, and value is its parent vertex).
        Uses self.publish_search() and self.publish_path(path) to publish the search tree and the final path respectively.
        """
        found_solution = False
        self.parent[self.start] = None
        self.cost[self.start] = 0.0
        # while not found_solution:
        
        while self.i < self.max_iteration:
            rand_pt = self.random_point()
            closest_point_to_rand_pt = self.find_closest(rand_pt)
            some_new_pt = self.new_pt(rand_pt, closest_point_to_rand_pt)
            is_valid = self.check_if_valid(closest_point_to_rand_pt, tuple(some_new_pt))
            if is_valid:
                self.parent[tuple(some_new_pt)] = tuple(closest_point_to_rand_pt)
                self.cost[tuple(some_new_pt)] = self.cost[tuple(closest_point_to_rand_pt)] + np.linalg.norm(some_new_pt - closest_point_to_rand_pt)
                self.find_and_update_neighbors(some_new_pt)
                self.publish_search()
                self.i += 1
        
        can_connect_to_endpoint = {}
        for point in self.parent.keys():
            if self.check_if_valid(point, self.end):
                cost = self.cost[point] + np.linalg.norm(np.array(self.end) - np.array(point))
                can_connect_to_endpoint[tuple(point)] = cost
        
        if can_connect_to_endpoint:
            point_with_lowest_cost = min(can_connect_to_endpoint, key=can_connect_to_endpoint.get)
            self.parent[self.end] = tuple(point_with_lowest_cost)

            path = [self.end]
            last_point = self.end
            #print(self.cost)
            #print(last_point)
            while last_point is not None:
                last_point = self.parent[tuple(last_point)]
                if last_point is None:
                    break
                path.append(last_point)
            path.append(self.start)
            self.publish_path(path)
        else:
            self.get_logger().info("Couldnt find a path")

def main(args=None):
    rclpy.init(args=args)
    rrt = RRT()
    while not rrt.data_received():
        rrt.get_logger().info("Waiting for data...")
        rclpy.spin_once(rrt)
        time.sleep(0.5)

    rrt.get_logger().info("Start graph searching!")
    time.sleep(1)
    rrt.search()


if __name__ == '__main__':
    main()
