import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from tf2_ros import TransformBroadcaster
import math
import numpy as np
from rclpy.duration import Duration

class SimpleSim(Node):
    def __init__(self):
        super().__init__('simple_sim_node')
        
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)
        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Twist, 'cmd_vel', self.cmd_callback, 10)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        
        # GÜVENLİK: Son emir zamanı
        self.last_cmd_time = self.get_clock().now()
        
        self.timer = self.create_timer(0.1, self.update)
        self.get_logger().info("Robot Guvenlik Freni (Watchdog) ile Baslatildi!")

    def cmd_callback(self, msg):
        self.linear_vel = msg.linear.x
        self.angular_vel = msg.angular.z
        # Her emir geldiğinde sayacı sıfırla
        self.last_cmd_time = self.get_clock().now()

    def get_distance_to_wall(self, angle_rad):
        ray_angle = self.theta + angle_rad
        ray_angle = math.atan2(math.sin(ray_angle), math.cos(ray_angle))
        
        dists = []
        if math.cos(ray_angle) != 0:
            d_east = (5.0 - self.x) / math.cos(ray_angle)
            d_west = (-5.0 - self.x) / math.cos(ray_angle)
            if d_east > 0: dists.append(d_east)
            if d_west > 0: dists.append(d_west)
            
        if math.sin(ray_angle) != 0:
            d_north = (5.0 - self.y) / math.sin(ray_angle)
            d_south = (-5.0 - self.y) / math.sin(ray_angle)
            if d_north > 0: dists.append(d_north)
            if d_south > 0: dists.append(d_south)

        return min(dists) if dists else 13.0

    def update(self):
        dt = 0.1
        
        # --- WATCHDOG KONTROLÜ ---
        # 1.0 saniyeden uzun süredir emir gelmediyse dur
        if (self.get_clock().now() - self.last_cmd_time).nanoseconds > 10.0 * 1e9:
            self.linear_vel = 0.0
            self.angular_vel = 0.0

        # Hareket Güncellemesi
        self.x += self.linear_vel * math.cos(self.theta) * dt
        self.y += self.linear_vel * math.sin(self.theta) * dt
        self.theta += self.angular_vel * dt

        now = self.get_clock().now()
        tf_time = (now + Duration(seconds=0.1)).to_msg()
        scan_time = now.to_msg()

        # TF Yayınları
        t = TransformStamped()
        t.header.stamp = tf_time
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = math.sin(self.theta / 2.0)
        t.transform.rotation.w = math.cos(self.theta / 2.0)
        self.tf_broadcaster.sendTransform(t)

        t_link = TransformStamped()
        t_link.header.stamp = tf_time
        t_link.header.frame_id = 'base_footprint'
        t_link.child_frame_id = 'base_link'
        t_link.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t_link)

        t_laser = TransformStamped()
        t_laser.header.stamp = tf_time
        t_laser.header.frame_id = 'base_link'
        t_laser.child_frame_id = 'laser_frame'
        t_laser.transform.translation.x = 0.2
        t_laser.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t_laser)

        # Odometry
        odom = Odometry()
        odom.header.stamp = scan_time
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = t.transform.rotation
        self.odom_pub.publish(odom)

        # Görselleştirme Marker
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = scan_time
        marker.ns = "robot_shape"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.scale.x = 0.5; marker.scale.y = 0.3; marker.scale.z = 0.2
        marker.color.a = 1.0; marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 1.0
        self.marker_pub.publish(marker)

        # Lidar Taraması
        num_readings = 360
        scan = LaserScan()
        scan.header.stamp = scan_time
        scan.header.frame_id = 'laser_frame'
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = (scan.angle_max - scan.angle_min) / num_readings
        scan.range_min = 0.1
        scan.range_max = 12.0
        ranges = []
        for i in range(num_readings):
            angle = scan.angle_min + i * scan.angle_increment
            dist = self.get_distance_to_wall(angle)
            dist += np.random.normal(0, 0.02)
            ranges.append(dist)
        scan.ranges = ranges
        self.scan_pub.publish(scan)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleSim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()