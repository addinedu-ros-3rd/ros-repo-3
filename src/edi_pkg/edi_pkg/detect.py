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


class Detect(Node):
    def __init__(self):
        super().__init__('edi_detect')
        self.bridge = CvBridge()
        
        self.img_sub = self.create_subscription(CompressedImage, "/image_raw/compressed", self.callback, 10)
        self.avoid_pub = self.create_publisher(String, "/obstacle_avoid", 10)
        self.yolo = YOLO("/home/soomin/deeplearning-repo-4/ros_dl/src/haejo_pkg/model/yolov8n.pt")

        self.labels = self.yolo.names
        self.colors = [[np.random.randint(0, 255) for _ in range(3)] for _ in range(len(self.labels))]


    def callback(self, msg):
        img = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # print(img.shape[1])
        
        phone_results = self.yolo(img, stream=True)

        for r in phone_results:
            
            annotator = Annotator(img)
            boxes = r.boxes
            for idx, box in enumerate(boxes):
                b = box.xyxy[0]
                c = box.cls
            
                if int(c) == 62 or int(c) == 63:
                    
                    color = self.colors[int(c)]
                    annotator.box_label(b, self.yolo.names[int(c)], color)

                    b_list = b.tolist()
                    
                    distance = math.sqrt(pow((b_list[0] - b_list[2]), 2) + pow((b_list[1] - b_list[3]), 2))
                    
                    if distance > 160:
                        cv2.putText(img, f"{distance}", (100,100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
                        msg = String()
                        msg.data = "avoid"
                        self.avoid_pub.publish(msg)
                        print("0000000000000000000000000000000")

        img = annotator.result()

        cv2.imshow('Window Title', img)
        if cv2.waitKey(1) == 'q':
            cv2.destroyAllWindows()
        
        
def main(args=None):
    rclpy.init(args=args)
    detect = Detect()
    rclpy.spin(detect)
    detect.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()