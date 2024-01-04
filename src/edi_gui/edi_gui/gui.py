import rclpy
import sys

from rclpy.node import Node
from std_msgs.msg import Int8MultiArray
from edi_msg.srv import Bipoom

from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5 import uic
from PyQt5.QtCore import *

GUI_PATH = "/home/soomin/ros-repo-3/src/edi_gui/edi_gui/edi.ui"
from_class = uic.loadUiType(GUI_PATH)[0]

class WindowClass(QMainWindow, from_class):

    def __init__(self):
        super().__init__()
        self.setupUi(self)
        
        self.gui = GUI()
        self.isPencilbtnOn = False
        self.isBallbtnOn = False

        self.pencil_btn.clicked.connect(self.click_pencil)
        self.ball_btn.clicked.connect(self.click_ball)


    def click_pencil(self):
        self.gui.SendInfoToDB("pencil")
        

    def click_ball(self):
        self.gui.SendInfoToDB("ball")
        
    
    def shutdown_ros(self):
        self.gui.destroy_node()

        rclpy.shutdown()


class GUI(Node):
    def __init__(self):
        super().__init__('edi_gui_node')

        self.pub = self.create_publisher(Int8MultiArray, "/bipoom_code", 10)


    def SendInfoToDB(self, name):
        msg = Int8MultiArray()
        code = [0, 0]

        if (name == "pencil"):
            code[0] = 1
        
        elif (name == "ball"):
            code[0] = 2

        msg.data = code
        
        self.pub.publish(msg)
    

def main(args=None):

    rclpy.init(args=None)
    app = QApplication(sys.argv)
    myWindow = WindowClass()
    myWindow.show()
    app.aboutToQuit.connect(myWindow.shutdown_ros)
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
