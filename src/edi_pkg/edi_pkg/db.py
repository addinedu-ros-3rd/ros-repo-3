import mysql.connector
import rclpy 
from rclpy.node import Node
from std_msgs.msg import Int8MultiArray, Int32
from datetime import datetime


class DB(Node):

    def __init__(self):
        super().__init__("edi_db")
        
        self.db_con = mysql.connector.connect(
            host = "database-1.cmdlhagy2um9.ap-northeast-2.rds.amazonaws.com",
            port = 3306,
            user = "admin", 
            password = "dltnals123$",
            database = "edi"
        )
        self.sub = self.create_subscription(Int8MultiArray, "/bipoom_code", self.bipoom_callback, 10)
        self.pub = self.create_publisher(Int8MultiArray, "/bipoom_exist", 10)
        self.bipoom_info_pub = self.create_publisher(Int8MultiArray, "/bipoom_info", 10)
        self.cur = self.db_con.cursor()

        self.get_logger().info("SUCCESS LOAD DB")


    def bipoom_callback(self, msg):
        info = msg.data
        is_exist = Int8MultiArray()

        name = info[0]
        counts = info[1]

        self.cur.execute("select * from bipoom_table")
        result = self.cur.fetchall()

        current_count = result[name-1][2] - counts

        if (current_count < 0):
            is_exist.data = [0, 0]
            self.pub.publish(is_exist)
        else:
            is_exist.data = [1, 0]
            self.pub.publish(is_exist)

            # print("a: ", result[name-1][2], "b: ", counts)
            self.cur.execute("insert into book_table (order_name, book_time) values('bipoom_{0}_{1}', '{2}')".format(name, counts, datetime.now()))
            self.cur.execute("update bipoom_table set counts = {0} where id = {1}".format(current_count, name))
            self.db_con.commit()  # 이 코드가 있어야 DB에 적용됨
            self.get_logger().info("success insert")
            self.send_bipoom_info()


    def send_bipoom_info(self):

        # 1은 창고 2는 청소모드 0은 충전소
        msg = Int8MultiArray()
        
        self.cur.execute("select * from book_table")
        result = self.cur.fetchall()
        
        if not (result == []):
            bipoom_info = result[0][1].split("_")[1]
           
            msg.data = [1, int(bipoom_info)]
        else:
            msg.data = [0, 0] 
        
        self.bipoom_info_pub.publish(msg)
        self.get_logger().info("success send bipoom info")


    def __del__(self):
        self.db_con.close()


def main(args=None):
    rclpy.init(args=None)
    db = DB()
    rclpy.spin(db)
    db.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
