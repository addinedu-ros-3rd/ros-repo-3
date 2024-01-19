import rclpy
import cv2
import math
import numpy as np
from cv_bridge import CvBridge 
from rclpy.node import Node
from ultralytics import YOLO
from ultralytics.utils.plotting import Annotator
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Twist


class Tracking(Node):
    def __init__(self):
        super().__init__('edi_tracking')

        self.bridge = CvBridge()
        self.twist = Twist()
        
        self.img_sub = self.create_subscription(CompressedImage, "/image_raw/compressed", self.callback, 10)
        self.twist_pub = self.create_publisher(Twist, "/base_controller/cmd_vel_unstamped", 10)
        self.yolo = YOLO("/home/soomin/deeplearning-repo-4/ros_dl/src/haejo_pkg/model/yolov8n.pt")

        self.labels = self.yolo.names
        self.colors = [[np.random.randint(0, 255) for _ in range(3)] for _ in range(len(self.labels))]
        
        self.i_error_a = 0
        self.d_error_a = 0
        self.pre_center = 0


    def callback(self, msg):
        
        img = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
        
        person_results = self.yolo(img, stream=True)

        for r in person_results:
            
            annotator = Annotator(img)
            boxes = r.boxes
            # print(boxes)
            conf_list = []
            xyxy_list = []
            cls_list = []

            for idx, box in enumerate(boxes):
                conf_list.append(box.conf)
                xyxy_list.append(box.xyxy[0])
                cls_list.append(box.cls)

            max_conf_list = []
            max_xyxy_list = []

            if 0 in cls_list:
                for idx, c in enumerate(cls_list):
                    if int(c) == 0:
                        max_conf_list.append(conf_list[idx])
                        max_xyxy_list.append(xyxy_list[idx])

                max_conf_list = [float(conf) for conf in max_conf_list]
                index = np.argmax(max_conf_list)

                b = max_xyxy_list[index]

                color = self.colors[0]
                annotator.box_label(b, self.yolo.names[0], color)

                b_list = b.tolist()
                
                distance = math.sqrt(pow((b_list[0] - b_list[2]), 2) + pow((b_list[1] - b_list[3]), 2))
                self.object_center = (b_list[0] + b_list[2])/2
                
                # print("Distacne : ", distance)

                if distance > 160 or distance < 200:
                    self.publish_twist(3.0, self.get_controls(self.object_center, -1/500, 0, 0))
                    self.pre_center = self.object_center

                elif distance > 200:
                    self.publish_twist(0.0, 0.0)

            else:
                if self.pre_center > 0 and self.pre_center < 160 :
                    self.publish_twist(0.2, -1.0)

                elif self.pre_center > 160:
                    self.publish_twist(0.2, 1.0)
                
                elif self.pre_center == 0:
                    self.publish_twist(0.0, 0.0)
                

        img = annotator.result()

        cv2.imshow('Window Title', img)
        if cv2.waitKey(1) == ord('q'):
            cv2.destroyAllWindows()
    

    def get_controls(self, x,  Kp_a, Ki_a, Kd_a):

        p_error_a = x - 160
        self.i_error_a += p_error_a
        curr_d_error_a = p_error_a - self.d_error_a
        angular = Kp_a*p_error_a + Ki_a*self.i_error_a + Kd_a*curr_d_error_a
        
        if angular > 4.0:
            angular = 4.0

        if angular < -4.0:
            angular = -4.0
        
        return angular


    def publish_twist(self, linear_x, angular_z):
        self.twist.linear.x = linear_x
        self.twist.angular.z = angular_z
        self.twist_pub.publish(self.twist)

        
        
def main(args=None):
    rclpy.init(args=args)
    tracking = Tracking()
    rclpy.spin(tracking)
    cv2.destroyAllWindows()
    tracking.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()