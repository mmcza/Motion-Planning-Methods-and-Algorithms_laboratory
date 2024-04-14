import rclpy
import time
from mapr_6_student.grid_map import GridMap
import numpy as np

np.random.seed(444)


class RRT(GridMap):
    def __init__(self):
        super(RRT, self).__init__()
        self.step = 0.05

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

    def search(self):
        """
        RRT search algorithm for start point self.start and desired state self.end.
        Saves the search tree in the self.parent dictionary, with key value pairs representing segments
        (key is the child vertex, and value is its parent vertex).
        Uses self.publish_search() and self.publish_path(path) to publish the search tree and the final path respectively.
        """
        found_solution = False
        self.parent[self.start] = None
        while not found_solution:
            rand_pt = self.random_point()
            closest_point_to_rand_pt = self.find_closest(rand_pt)
            some_new_pt = self.new_pt(rand_pt, closest_point_to_rand_pt)
            is_valid = self.check_if_valid(closest_point_to_rand_pt, tuple(some_new_pt))
            if is_valid:
                self.parent[tuple(some_new_pt)] = tuple(closest_point_to_rand_pt)
                self.publish_search()
            can_connect_to_endpoint = self.check_if_valid(tuple(some_new_pt), self.end)
            if can_connect_to_endpoint:
                found_solution = True
                self.parent[self.end] = some_new_pt

                path = [self.end]
                last_point = self.end
                #print(last_point)
                while last_point is not None:
                    last_point = self.parent[tuple(last_point)]
                    if last_point is None:
                        break
                    path.append(last_point)
                path.append(self.start)
                self.publish_path(path)

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
