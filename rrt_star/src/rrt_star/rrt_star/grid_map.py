from copy import copy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import OccupancyGrid, Path
from visualization_msgs.msg import Marker, MarkerArray
from rclpy import qos
import time


class GridMap(Node):
    def __init__(self, node_name='graph_search'):
        super().__init__(node_name)
        self.map = None
        self.start = None
        self.end = None
        self.resolution = None
        self.width = None
        self.height = None
        self.parent = {}

        qos_profile = qos.QoSProfile(depth=10)
        qos_profile.durability = qos.DurabilityPolicy.TRANSIENT_LOCAL

        self.sub_map = self.create_subscription(OccupancyGrid, 'map', self.map_callback, qos_profile)
        self.sub_start_pt = self.create_subscription(Marker, 'point_start', self.set_start, 10)
        self.sub_end_pt = self.create_subscription(Marker, 'point_end', self.set_end, 10)
        
        self.pub_search = self.create_publisher(Marker, 'search', 10)
        self.pub_path= self.create_publisher(Path, 'path', 10)

        self.get_logger().info("Object initialized!")


    def data_received(self):
        if self.map is None or self.start is None or self.end is None:
            return False
        else:
            return True
        

    def map_callback(self, data):
        self.resolution = data.info.resolution
        self.width = data.info.width * self.resolution
        self.height = data.info.height * self.resolution
        # self.get_logger().info(f"{data.info.width}, {data.info.height}")
        # self.get_logger().info(f"{self.resolution}, {self.width}, {self.height}")
        map = np.array(data.data)
        map = np.reshape(map, (data.info.height, data.info.width))
        #map = np.reshape(map, (data.info.width, data.info.height))
        #map = np.transpose(map)
        self.map = map

    def get_marker_xy(self, marker):
        x = marker.pose.position.x
        y = marker.pose.position.y
        return x, y

    def set_start(self, data):
        x, y = self.get_marker_xy(data)
        self.start = (x, y)

    def set_end(self, data):
        x, y = self.get_marker_xy(data)
        self.end = (x, y)

    def publish_search(self, delay=0.01):
        marker = Marker()
        def add_point(p):
            pt = Point()
            pt.x = p[0]
            pt.y = p[1]
            pt.z = 0.0
            marker.points.append(pt)
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.color.r = 0.
        marker.color.g = 0.
        marker.color.b = 1.
        marker.color.a = 0.5
        marker.scale.x = 0.1 * self.resolution
        for k, v in self.parent.items():
            if v is None: continue
            add_point(k)
            add_point(v)
        self.pub_search.publish(marker)
        time.sleep(delay)

    def publish_path(self, path):
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        for p in path:
            pose = PoseStamped()
            pose.pose.position.x = p[0]
            pose.pose.position.y = p[1]
            pose.pose.position.z = 0.001
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
