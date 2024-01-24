import rclpy 
import time
from rclpy.node import Node
from std_msgs.msg import Int8MultiArray, String
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data, QoSProfile
from geometry_msgs.msg import Twist


class Avoidance(Node):
    def __init__(self):
        super().__init__('edi_avoidance')
        self.qos = QoSProfile(depth=10)
        self.twist = Twist()
        self.i = 0

        self.scan_sub = self.create_subscription(LaserScan, "/scan", self.lidar_callback, qos_profile=qos_profile_sensor_data)
        self.twist_pub = self.create_publisher(Twist, "/base_controller/cmd_vel_unstamped", 10)
        self.avoid_sub = self.create_subscription(String, "/obstacle_avoid", self.avoid, 10)


    def lidar_callback(self, msg):
        
        LidarData = [msg.ranges[187], msg.ranges[156], msg.ranges[125], msg.ranges[94], msg.ranges[63]] 

        print(msg.ranges[121])
        self.left_rate = 0
        self.right_rate = 0
        
        if LidarData[0]!=0.0:
            self.left_rate = LidarData[1]/LidarData[0]
        if LidarData[4]!=0.0:
            self.right_rate = LidarData[3]/LidarData[4]
        

    def avoid(self, msg):

        aaa = msg.data
        if self.left_rate > self.right_rate:
            self.publish_twist(2.0, -4.0)
            # print("LEFT")
        
        elif self.left_rate < self.right_rate:
            self.publish_twist(2.0, 4.0)
            # print("RIGHT")

        elif abs(self.left_rate-self.right_rate)<0.2:
            # print("GO")
            self.publish_twist(2.0, 0.0)

        
    def publish_twist(self, linear_x, angular_z):
        self.twist.linear.x = linear_x
        self.twist.angular.z = angular_z
        self.twist_pub.publish(self.twist)



def main(args=None):
    rclpy.init(args=args)
    avoid = Avoidance()
    rclpy.spin(avoid)
    avoid.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()