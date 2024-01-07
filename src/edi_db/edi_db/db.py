import mysql.connector
import rclpy 
from rclpy.node import Node
from std_msgs.msg import Int8MultiArray
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

        self.cur = self.db_con.cursor()

    def bipoom_callback(self, msg):
        info = msg.data

        name = info[0]
        counts = info[1]

        self.cur.execute("select * from bipoom_table")
        result = self.cur.fetchall()

        current_count = result[name-1][2] - counts
        # print("a: ", result[name-1][2], "b: ", counts)
        self.cur.execute("insert into book_table (order_name, book_time) values('bipoom_{0}_{1}', '{2}')".format(name, counts, datetime.now()))
        self.cur.execute("update bipoom_table set counts = {0} where id = {1}".format(current_count, name))
        self.db_con.commit()  # 이 코드가 있어야 DB에 적용됨
        self.get_logger().info("success insert")

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
