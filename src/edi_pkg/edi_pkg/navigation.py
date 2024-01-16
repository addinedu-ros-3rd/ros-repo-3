import rclpy
import math
import time
import numpy as np
from copy import deepcopy
from nav2_simple_commander.robot_navigator import BasicNavigator
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PointStamped, PoseWithCovarianceStamped, Twist
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Int8MultiArray, String
from nav2_simple_commander.robot_navigator import TaskResult


pencil_location = [[4.528031826019287, -0.007635989226400852, -0.001373291015625],
                   [4.229101657867432, 5.375986576080322, -0.001434326171875],
                   [4.6302056312561035, 12.080599784851074, -0.001434326171875]]

ball_location = [[4.528031826019287, -0.007635989226400852, -0.001373291015625],
                   [4.229101657867432, 5.375986576080322, -0.001434326171875],
                   [4.769007682800293, 14.327844619750977, -0.001434326171875]]

home = [[4.229101657867432, 5.375986576080322, -0.001434326171875],
        [4.528031826019287, -0.007635989226400852, -0.001373291015625],
        [0.0984344482421875, -0.032880719751119614, 0.008453369140625]]


class Navigation(Node):

    def __init__(self):
        super().__init__('edi_navigation')
        
        self.nav = BasicNavigator()
        self.nav.waitUntilNav2Active()
        self.get_logger().info("SUCCESS NAV2")

        self.task_complete_pub = self.create_publisher(String, '/task_complete', 10)
        self.vel_sub = self.create_subscription(Twist, '/cmd_vel', self.vel_callback, 10)
        self.amcl_pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_pose_callback, 10)
        self.bipoom_info_sub = self.create_subscription(Int8MultiArray, "/bipoom_info", self.bipoom_callback, 10)
        # self.current_pose_sub = self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.calc_diff_pose, 10)
    
        self.pre_X = 0.0
        self.pre_Y = 0.0
        self.vel = 0.0

        self.get_logger().info("SUCCESS LOAD NAVIGATION")

        
    def goToBipoom(self, bipoom_number):
        self.bipoom_points = []
        route = []

        bipoom_pose = PoseStamped()
        
        bipoom_pose.header.frame_id = 'map'
        bipoom_pose.header.stamp = self.nav.get_clock().now().to_msg()
        
        bipoom_pose.pose.orientation.z = 0.998839558403812
        bipoom_pose.pose.orientation.w = 0.04816156732995615

        if bipoom_number == 1:
            route = pencil_location
        else:
            route = ball_location

        for pt in route:
            bipoom_pose.pose.position.x = pt[0]
            bipoom_pose.pose.position.y = pt[1]
            bipoom_pose.pose.position.z = pt[2]
            self.bipoom_points.append(deepcopy(bipoom_pose))

        nav_start = self.nav.get_clock().now()
        self.nav.followWaypoints(self.bipoom_points)

        # self.nav.goToPose(bipoom_pose)
        self.getFeedback(self.bipoom_points)


    def goToHome(self):
        self.home_points = []
        home_pose = PoseStamped()

        home_pose.header.frame_id = 'map'
        home_pose.header.stamp = self.nav.get_clock().now().to_msg()

        home_pose.pose.orientation.z = 0.998839558403812
        home_pose.pose.orientation.w = 0.04816156732995615

        for pt in home:
            home_pose.pose.position.x = pt[0]
            home_pose.pose.position.y = pt[1]
            home_pose.pose.position.z = pt[2]
            self.home_points.append(deepcopy(home_pose))

        nav_start = self.nav.get_clock().now()
        self.nav.followWaypoints(self.home_points)
        
        # self.nav.goToPose(home_pose)
        self.getFeedback(self.home_points)


    def getFeedback(self, waypoints):
        i = 0
        while not self.nav.isTaskComplete():
            i += 1
            feedback = self.nav.getFeedback()
    
            if feedback and i % 5 ==0:
                self.get_logger().info('Executing current waypoint: ' + str(feedback.current_waypoint + 1) + '/' + str(len(waypoints)))
                
        result = self.nav.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info("goal succed")
        elif result == TaskResult.CANCELED:
            self.get_logger().info('goal was canceled')
        elif result == TaskResult.FAILED:
            self.get_logger().info('goal failed')


    def vel_callback(self, msg):
        self.vel = msg.linear.x


    def bipoom_callback(self, msg):
        bipoom_info = msg.data
        self.goToBipoom(bipoom_info[0])
        self.goToHome()
        
        msg = String()
        msg.data = "success_{0}".format(bipoom_info[1])
        self.task_complete_pub.publish(msg)
    

    def amcl_pose_callback(self, data):
        pose_current = data

        q = [0, 0, 0, 0]
        q[0] = pose_current.pose.pose.orientation.x
        q[1] = pose_current.pose.pose.orientation.y
        q[2] = pose_current.pose.pose.orientation.z
        q[3] = pose_current.pose.pose.orientation.w

        self.degree = self.convert_degree(euler_from_quaternion(q))
        self.calculate_odometry(self.degree[2])
        

    def calculate_odometry(self, yaw):
        
        self.cur_X = self.pre_X + 0.1 * self.vel * np.cos(yaw*np.pi / 180)
        self.cur_Y = self.pre_Y + 0.1 * self.vel * np.sin(yaw*np.pi / 180)

        # print("X : ", self.cur_X, "Y: ", self.cur_Y, "vel: ", self.vel, "yaw : ", yaw)
        
        self.pre_X = self.cur_X
        self.pre_Y = self.cur_Y


    def convert_degree(self, input):
        return np.array(input) * 180. / np.pi
    

    # def convert_quaternion(self):
    #     to_radian = np.pi / 180
    #     tmp = [0, 0, 0]
    #     quaternion_from_euler(tmp[0], tmp[1], tmp[2])


def main(args=None):
    rclpy.init(args=args)
    navigation = Navigation()
    rclpy.spin(navigation)
    navigation.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

