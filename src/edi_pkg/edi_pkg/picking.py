import rclpy
import cv2
import math
import numpy as np
from cv_bridge import CvBridge 
from rclpy.node import Node
from ultralytics import YOLO
from ultralytics.utils.plotting import Annotator
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Int32MultiArray


class Picking(Node):
    def __init__(self):
        super().__init__('edi_picking')
        self.bridge = CvBridge()
        
        self.img_sub = self.create_subscription(CompressedImage, "/color/image_raw/compressed", self.color_callback, 10)
        self.depth_sub = self.create_subscription(Image, "/depth/image_rect_raw", self.depth_callback, 10)
        self.dist_pub = self.create_publisher(Int32MultiArray, "/depth/object_distance", 10)
        
        self.yolo = YOLO("/home/soomin/ros-repo-3/src/edi_pkg/edi_pkg/utils/model.pt")

        self.labels = self.yolo.names
        self.colors = [[np.random.randint(0, 255) for _ in range(3)] for _ in range(len(self.labels))]
        self.distance = 0
        self.object_center = (0, 0)
        self.center_list = []
        self.dist_msg = Int32MultiArray()
        self.dist_list = []


    def color_callback(self, msg):
        self.img = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")

        phone_results = self.yolo(self.img, stream=True)

        for r in phone_results:
            
            annotator = Annotator(self.img)
            boxes = r.boxes
            # print(boxes)
            for idx, box in enumerate(boxes):
                b = box.xyxy[0]
                c = box.cls
                conf = box.conf
                
                color = self.colors[int(c)]
                label = "{0}{1}".format(self.yolo.names[int(c)], round(conf.item(), 3))
                annotator.box_label(b, label, color)
                
                b_list = b.tolist()
                self.object_center = (int((b_list[0] + b_list[2])/2), int((b_list[1] + b_list[3])/2))
                self.center_list.append(self.object_center)

        self.img = annotator.result()
    
    
    def depth_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        img = cv2.resize(img, (640, 480))

        if self.center_list != []:
            for center in self.center_list:
                distance = img[center[1]][center[0]]
                self.dist_list.append(int(distance))
        
                cv2.circle(self.img, center, 5, (0,0,0), -1)
                text = "dis: {0}cm".format(distance//10)
                cv2.putText(self.img, text, center, cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            self.dist_msg.data = self.dist_list
            self.dist_pub.publish(self.dist_msg)

            self.center_list = []
        
        cv2.imshow('color', self.img)
        if cv2.waitKey(1) == 'q':
            cv2.destroyAllWindows()

        cv2.imshow('depth', img)
        if cv2.waitKey(1) == 'q':
            cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    picking = Picking()
    rclpy.spin(picking)
    cv2.destroyAllWindows()
    picking.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()