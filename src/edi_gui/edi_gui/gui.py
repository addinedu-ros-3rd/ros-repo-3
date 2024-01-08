import rclpy
import sys
from rclpy.node import Node
from std_msgs.msg import Int8MultiArray, Int32

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
        text, ok = QInputDialog.getText(self, 'Bipoom - Pencil', 'Counts:')

        if text and ok:
            self.gui.SendInfoToDB("pencil", int(text))
                

    def click_ball(self):   
        text, ok = QInputDialog.getText(self, 'Bipoom - Ball', 'Counts:')

        if text and ok:
            self.gui.SendInfoToDB("ball", int(text))

            # if not self.gui.is_exist:
            #     QMessageBox.warning(self, 'Bipoom - Warning', 'Fill the Balls')
    

    def shutdown_ros(self):
        self.gui.destroy_node()

        rclpy.shutdown()


class GUI(Node):
    
    def __init__(self):
        super().__init__('edi_gui')

        self.pub = self.create_publisher(Int8MultiArray, "/bipoom_code", 10)
        self.sub = self.create_subscription(Int32, "/bipoom_exist", self.is_exist_callback, 10)
        

    def SendInfoToDB(self, name, counts):
        msg = Int8MultiArray()
        code = [0, 0]

        if (name == "pencil"):
            code[0] = 1
            code[1] = counts
        
        elif (name == "ball"):
            code[0] = 2
            code[1] = counts

        msg.data = code
        
        self.pub.publish(msg)


    def is_exist_callback(self, msg):
        self.is_exist = msg.data
        print(self.is_exist)

        if not self.is_exist:
            QMessageBox.warning(self, 'Bipoom - Warning', 'Fill the equipments')


def main(args=None):

    rclpy.init(args=None)
    app = QApplication(sys.argv)
    myWindow = WindowClass()
    myWindow.show()
    app.aboutToQuit.connect(myWindow.shutdown_ros)
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
