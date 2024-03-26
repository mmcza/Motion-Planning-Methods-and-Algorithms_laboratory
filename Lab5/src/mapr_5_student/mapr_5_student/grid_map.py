from copy import deepcopy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from visualization_msgs.msg import Marker
import time
from rclpy import qos


class GridMap(Node):
    def __init__(self, node_name='graph_search'):
        super().__init__(node_name)
        self.map = None
        self.start = None
        self.end = None     

        qos_profile = qos.QoSProfile(depth=10)
        qos_profile.durability = qos.DurabilityPolicy.TRANSIENT_LOCAL

        self.sub_map = self.create_subscription(OccupancyGrid, 'map', self.map_callback, qos_profile)
        self.sub_start_pt = self.create_subscription(Marker, 'point_start', self.set_start, 10)
        self.sub_end_pt = self.create_subscription(Marker, 'point_end', self.set_end, 10)

        self.pub_map = self.create_publisher(OccupancyGrid, 'map_visited', 10)
        self.pub_path= self.create_publisher(Path, 'path', 10)

        self.get_logger().info("Object initialized!")


    def data_received(self):
        if self.map is None or self.start is None or self.end is None:
            return False
        else:
            return True

    def map_callback(self, data):
        self.map = deepcopy(data)
        self.map.data = list(self.map.data)

    def get_marker_xy(self, marker):
        while self.map is None:
            time.sleep(0.5)
        mul = 1. / self.map.info.resolution
        x = int(marker.pose.position.x * mul)
        y = int(marker.pose.position.y * mul)
        return x, y

    def set_start(self, data):
        x, y = self.get_marker_xy(data)
        self.start = (x, y)

    def set_end(self, data):
        x, y = self.get_marker_xy(data)
        self.end = (x, y)

    def publish_visited(self, delay=0.5):
        self.pub_map.publish(self.map)
        time.sleep(delay)

    def publish_path(self, path):
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        for p in path:
            pose = PoseStamped()
            pose.pose.position.x = self.map.info.resolution * p[0] + 0.05
            pose.pose.position.y = self.map.info.resolution * p[1] + 0.05
            pose.pose.position.z = 0.0
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 1.0
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            path_msg.poses.append(pose)
        self.pub_path.publish(path_msg)

    def search(self):
        return NotImplementedError()
