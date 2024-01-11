import rclpy
import numpy as np
from nav2_simple_commander.robot_navigator import BasicNavigator
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PointStamped, PoseWithCovarianceStamped, Twist
from nav2_simple_commander.robot_navigator import TaskResult
from rclpy.duration import Duration
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Int8MultiArray

bipoom_location = [[1.7631229162216187, 0.018305590376257896, 0.0064697265625],
                  [1.8413116931915283, -0.7576633095741272, 0.0064697265625]]
home = [0.4821498990058899, 0.4071018695831299, 0.004547119140625]

class Navigation(Node):

    def __init__(self):
        super().__init__('edi_navigation_node')
        
        self.nav = BasicNavigator()
        self.nav.waitUntilNav2Active()
        self.get_logger().info("SUCCESS NAV2")

        self.clicked_point = PointStamped()
        self.goal_pose = PoseStamped()
        self.pose_current = PoseWithCovarianceStamped()

        # self.clicked_point_sub = self.create_subscription(PointStamped, '/clicked_point', self.clicked_point_callback, 10)
        self.vel_sub = self.create_subscription(Twist, '/cmd_vel', self.vel_callback, 10)
        self.amcl_pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.callback, 10)
        self.bipoom_info_sub = self.create_subscription(Int8MultiArray, "/bipoom_info", self.bipoom_callback, 10)
        
        self.pre_X = 0.0
        self.pre_Y = 0.0
        self.vel = 0.0

        self.get_logger().info("Success load navigation")

        
    def goToPose(self, bipoom_number):
        
        self.goal_pose.header.frame_id = 'map'
        self.goal_pose.header.stamp = self.nav.get_clock().now().to_msg()
        
        self.goal_pose.pose.position.x = bipoom_location[bipoom_number-1][0]
        self.goal_pose.pose.position.y = bipoom_location[bipoom_number-1][1]
        self.goal_pose.pose.position.z = bipoom_location[bipoom_number-1][2]

        self.goal_pose.pose.orientation.x = 0.0
        self.goal_pose.pose.orientation.y = 0.0
        self.goal_pose.pose.orientation.z = 0.0
        self.goal_pose.pose.orientation.w = 1.0

        self.nav.goToPose(self.goal_pose)

        # i = 0
        # while not self.nav.isTaskComplete():
        #     i += 1
        #     feedback = self.nav.getFeedback()
            

        #     if feedback and i % 5 ==0:
        #         print('Distance remaining: ' + '{:.2f}'.format(feedback.distance_remaining) + ' meters.')
                

        #         if Duration.from_msg(feedback.navigation_time) > Duration(seconds=15.0):
        #             self.nav.cancelTask()

        result = self.nav.getResult()
        if result == TaskResult.SUCCEEDED:
            print("goal succed")    
        elif result == TaskResult.CANCELED:
            print('goal was canceled')
        elif result == TaskResult.FAILED:
            print('goal failed')

    
    def vel_callback(self, msg):
        self.vel = msg.linear.x


    def bipoom_callback(self, msg):
        bipoom_info = msg.data
        self.goToPose(bipoom_info[1])
        

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


def main(args=None):
    rclpy.init(args=args)
    navigation = Navigation()
    rclpy.spin(navigation)
    navigation.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

