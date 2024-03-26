import rclpy
import time
from mapr_5_student.grid_map import GridMap
import heapq as pq
import math

class ASTAR(GridMap):
    def __init__(self):
        super(ASTAR, self).__init__('astar_node')
        self.end_point_found = False
        self.directions = [[0, -1],[-1, 0],[0, 1],[1, 0]]
        self.h = []
        self.point_dictionary = {}

    def heuristics(self, pos):
        ### Uncomment line with selected metric

        # Manhattan metric
        # distance = abs(pos[0] - self.end[0]) + abs(pos[1] - self.end[1])
        # Euclidean distance
        distance = math.sqrt((pos[0] - self.end[0])**2 + (pos[1] - self.end[1])**2)
        # Chebyshev distance
        # distance = max(abs(pos[0] - self.end[0]), abs(pos[1] - self.end[1]))
        return distance

    def search(self):
        pq.heappush(self.h, (self.heuristics(self.start) + 0, self.start, 0, self.start))
        self.point_dictionary[self.start]=[0, self.start]
        while self.h and not self.end_point_found:
            point = pq.heappop(self.h)
            self.cost = []
            if point[1] == self.end:
                self.end_point_found = True
                break
            for direction in self.directions:
                # update point_dictionary if position wasn't checked before or if previous cost was higher than new possible one
                if (point[1][0]+direction[0], point[1][1]+direction[1]) in self.point_dictionary:
                    if self.point_dictionary[(point[1][0]+direction[0], point[1][1]+direction[1])][0] > point[2]+1:
                        self.point_dictionary[(point[1][0]+direction[0], point[1][1]+direction[1])] = [point[2]+1, point[1]]
                else:
                    self.point_dictionary[(point[1][0]+direction[0], point[1][1]+direction[1])] = [point[2]+1, point[1]]

                # add neighbour to comparision if it is empty (wasn't checked before and is not a wall)
                if self.map.data[self.map.info.width*(point[1][1]+direction[1])+point[1][0]+direction[0]] == 0:
                    pq.heappush(self.cost, (self.heuristics((point[1][0]+direction[0], point[1][1]+direction[1]))+point[0]+1, (point[1][0]+direction[0], point[1][1]+direction[1])))
            
            # add element with lowest cost from all possible neighbours
            if self.cost:
                point_to_visit = pq.heappop(self.cost)
                pq.heappush(self.h, (point_to_visit[0], point_to_visit[1], self.point_dictionary[point_to_visit[1]][0], self.point_dictionary[point_to_visit[1]][1]))
                self.map.data[self.map.info.width*(point_to_visit[1][1])+point_to_visit[1][0]] = 50
                #self.get_logger().info(self.point_list)        
            self.publish_visited()        
        
        # create path if end point was found
        if self.end_point_found:
            path = [self.end]
            last_point = self.end
            while last_point != self.start:
                last_point = self.point_dictionary[last_point][1]
                path.append(last_point)
            path.append(self.start)
            self.publish_path(path)


def main(args=None):
    rclpy.init(args=args)
    astar = ASTAR()
    while not astar.data_received():
        astar.get_logger().info("Waiting for data...")
        rclpy.spin_once(astar)
        time.sleep(0.5)

    astar.get_logger().info("Start graph searching!")
    astar.publish_visited()
    time.sleep(1)
    astar.search()

if __name__ == '__main__':
    main()