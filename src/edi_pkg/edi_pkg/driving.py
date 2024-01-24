import rclpy
import cv2
import math
import numpy as np
from cv_bridge import CvBridge 
from rclpy.node import Node
from ultralytics import YOLO
from ultralytics.utils.plotting import Annotator
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data, QoSProfile
from sensor_msgs.msg import LaserScan


class Driving(Node):
    def __init__(self):
        super().__init__('edi_driving')
        self.qos = QoSProfile(depth=10)
        self.bridge = CvBridge()

       
        self.img_sub = self.create_subscription(CompressedImage, "image_raw/compressed", self.camera_callback, 10)
        self.yolo = YOLO("/home/soomin/deeplearning-repo-4/ros_dl/src/haejo_pkg/model/yolov8n.pt")
        self.get_logger().info("SUCCESS LOAD DRIVING MODEL")

        self.scan_sub = self.create_subscription(LaserScan, "/scan", self.lidar_callback, qos_profile=qos_profile_sensor_data)
        
        self.twist_pub = self.create_publisher(Twist, "/base_controller/cmd_vel_unstamped", 10)

        self.labels = self.yolo.names
        self.colors = [[np.random.randint(0, 255) for _ in range(3)] for _ in range(len(self.labels))]
        self.object_center = (320,240)
        self.people = False


    def camera_callback(self, msg):
        self.img = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
        
        phone_results = self.yolo(self.img, stream=True)

        for r in phone_results:
            
            annotator = Annotator(self.img)
            boxes = r.boxes
            for idx, box in enumerate(boxes):
                b = box.xyxy[0]
                c = box.cls
                conf = box.conf
            
                if int(c) == 62 or int(c) == 63 or int (c) == 0:
                    
                    if int(c) == 0:
                        self.people = True

                    color = self.colors[int(c)]
                    
                    if int(c) == 0:
                        label = "person{0}".format(round(conf.item(),2))
                    else:
                        label = "obstacle{0}".format(round(conf.item(),2))

                    annotator.box_label(b, label, color)

                    b_list = b.tolist()
                    
                    # distance = math.sqrt(pow((b_list[0] - b_list[2]), 2) + pow((b_list[1] - b_list[3]), 2))
                    self.object_center = (int((b_list[0]+b_list[2])/2), int((b_list[1]+b_list[3])/2))
                    
                    # if distance > 160:
                    cv2.putText(self.img, f"center:{self.object_center}", self.object_center, cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
                
        self.img = annotator.result()


    def lidar_callback(self, msg):
        
        LidarData = msg.ranges[121]

        if LidarData <=1.0 and self.people == True and LidarData >0:
            cv2.putText(self.img, f"People!!!", (320,50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)
            self.publish_twist(0.0, -1.0)

        elif LidarData <= 1.0 and LidarData >0 and self.object_center[0] < 320:
            cv2.putText(self.img, f"ROTATE RIGHT", (320,50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)
            self.people = False
            self.publish_twist(0.0, 0.5)
            
        elif LidarData <=1.0 and LidarData >0 and self.object_center[0] > 320:
            cv2.putText(self.img, f"ROTATE LEFT", (320,50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)
            self.people = False
            self.publish_twist(0.0, -0.5)
        
        else:
            self.people = False
            cv2.putText(self.img, f"distance :{round(LidarData,2)}", (320,50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)

        cv2.imshow('ETTI Autonomous Driving', self.img)
        if cv2.waitKey(1) == 'q':
            cv2.destroyAllWindows()
        
    
    def publish_twist(self, linear_x, angular_z):
        self.twist = Twist()
        
        self.twist.linear.x = linear_x
        self.twist.angular.z = angular_z
        self.twist_pub.publish(self.twist)
        
        
def main(args=None):
    rclpy.init(args=args)
    driving = Driving()
    rclpy.spin(driving)
    driving.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == "__main__":
    main()