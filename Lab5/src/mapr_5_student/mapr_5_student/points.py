import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker


class PointsPublisher(Node):
    def __init__(self):
        super().__init__('points_publisher')
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.pt_publishers = list()
        self.markers = list()

    def add_point(self, x, y, name, color):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = name
        marker.id = 0
        marker.type = Marker.CUBE
        # marker.action = Marker.ADD
        marker.pose.position.x = x + 0.05
        marker.pose.position.y = y + 0.05
        marker.pose.position.z = 0.0  # shift sphere up
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.01
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 0.5

        self.markers.append(marker)
        self.pt_publishers.append(self.create_publisher(Marker, 'point_' + name, 10))


    def timer_callback(self):
        for point, publisher in zip(self.markers, self.pt_publishers):
            point.header.stamp = self.get_clock().now().to_msg()
            publisher.publish(point)


def main(args=None):
    rclpy.init(args=args)
    points_publisher = PointsPublisher()
    points_publisher.add_point(0.7, 0.7, "start", (0.0, 1.0, 0.0))
    points_publisher.add_point(0.5, 1.3, "end", (1.0, 0.0, 0.0))
    rclpy.spin(points_publisher)
    points_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
