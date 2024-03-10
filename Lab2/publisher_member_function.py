import rclpy
from rclpy.node import Node
from rclpy import qos
import copy
from nav_msgs.msg import OccupancyGrid

class MapSubscriber(Node):

   def __init__(self):
       super().__init__('map_subscriber')
       qos_profile = qos.QoSProfile(depth=10)
       qos_profile.durability = qos.DurabilityPolicy.TRANSIENT_LOCAL
       self.publisher_ = self.create_publisher(OccupancyGrid, 'map_copy', qos_profile)
       self.subscription = self.create_subscription(
           OccupancyGrid,
           'map',
           self.listener_callback,
           qos_profile)
       self.subscription  # prevent unused variable warning

   def listener_callback(self, msg):
       map_cpy = OccupancyGrid()
       map_cpy = copy.deepcopy(msg)
       for i in range(map_cpy.info.width):
           for j in range(map_cpy.info.height):
               if self.hasNeighbor(i, j, map_cpy):
                   map_cpy.data[i + j * map_cpy.info.width] = 100
       self.get_logger().info('I heard a map')
       self.publisher_.publish(map_cpy)
       self.get_logger().info('Publishing map')

   def hasNeighbor(self, col, row, map):
       for i in range(-1, 2):
           for j in range(-1, 2):
               if (col + i >= 0 and col + i < map.info.width and
                       row + j >= 0 and row + j < map.info.height):
                   if (map.data[col + i + (row + j) * map.info.width]) > 50:
                       return True
       return False


def main(args=None):
   rclpy.init(args=args)

   map_subscriber = MapSubscriber()
   rclpy.spin(map_subscriber)

   # Destroy the node explicitly (optional - otherwise it will be
   # done automatically when the garbage collector destroys the node object)
   map_subscriber.destroy_node()
   rclpy.shutdown()


if __name__ == '__main__':
   main()
