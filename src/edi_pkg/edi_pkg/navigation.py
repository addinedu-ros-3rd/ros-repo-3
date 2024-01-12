import rclpy
import math
import time
import numpy as np
from nav2_simple_commander.robot_navigator import BasicNavigator
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PointStamped, PoseWithCovarianceStamped, Twist
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Int8MultiArray
from nav2_simple_commander.robot_navigator import TaskResult
from rclpy.duration import Duration

bipoom_location = [[1.7631229162216187, 0.018305590376257896, 0.0064697265625],
                  [1.8413116931915283, -0.7576633095741272, 0.0064697265625]]
home = [0.4821498990058899, 0.4071018695831299, 0.004547119140625]

class Navigation(Node):

    def __init__(self):
        super().__init__('edi_navigation')
        
        self.nav = BasicNavigator()
        self.nav.waitUntilNav2Active()
        self.get_logger().info("SUCCESS NAV2")

        self.quantile_time = 0.95
        
        self.goal_pose = PoseStamped()

        self.vel_sub = self.create_subscription(Twist, '/cmd_vel', self.vel_callback, 10)
        self.amcl_pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.callback, 10)
        self.bipoom_info_sub = self.create_subscription(Int8MultiArray, "/bipoom_info", self.bipoom_callback, 10)
        # self.current_pose_sub = self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.calc_diff_pose, 10)
    
        self.pre_X = 0.0
        self.pre_Y = 0.0
        self.vel = 0.0

        self.get_logger().info("SUCCESS LOAD NAVIGATION")

        
    def goToBipoom(self, bipoom_number):
        bipoom_pose = PoseStamped()
        
        bipoom_pose.header.frame_id = 'map'
        bipoom_pose.header.stamp = self.nav.get_clock().now().to_msg()
        
        bipoom_pose.pose.position.x = bipoom_location[bipoom_number-1][0]
        bipoom_pose.pose.position.y = bipoom_location[bipoom_number-1][1]
        bipoom_pose.pose.position.z = bipoom_location[bipoom_number-1][2]

        self.goal = [bipoom_location[bipoom_number-1][0],
                    bipoom_location[bipoom_number-1][1]]
        
        bipoom_pose.pose.orientation.x = 0.0
        bipoom_pose.pose.orientation.y = 0.0
        bipoom_pose.pose.orientation.z = 0.9764293082836897
        bipoom_pose.pose.orientation.w = 0.21583745255315445

        self.nav.goToPose(bipoom_pose)
        self.getFeedback()


    def goToHome(self):
        home_pose = PoseStamped()

        home_pose.header.frame_id = 'map'
        home_pose.header.stamp = self.nav.get_clock().now().to_msg()
        
        home_pose.pose.position.x = home[0]
        home_pose.pose.position.y = home[1]
        home_pose.pose.position.z = home[2]

        self.goal_pose.pose.position.x = home[0]
        self.goal_pose.pose.position.y = home[1]

        home_pose.pose.orientation.x = 0.0
        home_pose.pose.orientation.y = 0.0
        home_pose.pose.orientation.z = 0.9764293082836897
        home_pose.pose.orientation.w = 0.21583745255315445

        self.nav.goToPose(home_pose)
        self.getFeedback()


    def getFeedback(self):
        i = 0
        while not self.nav.isTaskComplete():
            i += 1
            feedback = self.nav.getFeedback()
    
            if feedback and i % 5 ==0:
                self.get_logger().info('Distance remaining: ' + '{:.2f}'.format(feedback.distance_remaining) + ' meters.')
                
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
        self.goToBipoom(bipoom_info[1])
        self.goToHome()
        

    def callback(self, data):
        
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


    # def calc_diff_pose(self, data):
    #     self.current_pose = data
        
    #     distance = math.sqrt((self.current_pose.pose.pose.position.x - self.goal_pose.pose.position.x)**2 +\
    #                          (self.current_pose.pose.pose.position.y - self.goal_pose.pose.position.y)**2)
    #     if distance < 0.02:
    #         output_msg = "Edi Arrived  Goal Point. "
    #         self.get_logger().info(output_msg)
            

def main(args=None):
    rclpy.init(args=args)
    navigation = Navigation()
    rclpy.spin(navigation)
    navigation.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

