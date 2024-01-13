import mysql.connector
import rclpy 
from rclpy.node import Node
from std_msgs.msg import Int8MultiArray, String
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
        
        self.task_complete_sub = self.create_subscription(String, "/task_complete", self.task_complete_callback, 10)
        self.sub = self.create_subscription(Int8MultiArray, "/bipoom_code", self.bipoom_callback, 10)
        self.pub = self.create_publisher(Int8MultiArray, "/bipoom_exist", 10)
        self.bipoom_info_pub = self.create_publisher(Int8MultiArray, "/bipoom_info", 10)
        self.clean_info_pub = self.create_publisher(Int8MultiArray, "/clean_info", 10)

        self.cur = self.db_con.cursor()
        self.get_logger().info("SUCCESS LOAD DB")
        # self.do_task()
        

    def do_task(self):

        msg = Int8MultiArray()

        self.cur.execute("SELECT * FROM book_table WHERE (start_time IS NULL) ORDER BY id LIMIT 1;")
        result = self.cur.fetchall()

        if not (result == []):
            task_id = result[0][0] # task id
            task_info = result[0][1].split("_")[0] # bipoom or clean
            
            if task_info == "bipoom":
                bipoom_info = result[0][1].split("_")[1] # pencil or ball
                msg.data = [int(bipoom_info), task_id] 

                self.bipoom_info_pub.publish(msg)
                self.get_logger().info("SUCCESS SEND BIPOOM INFO")
                
            else:
                clean_info = result[0][1].split("_")[1] # clean publish
                msg.data = [int(clean_info), task_id]

                self.clean_info_pub.publish(msg)
                self.get_logger().info("SUCCESS SEND CLEAN INFO")

            self.cur.execute("update book_table set start_time = '{0}' where id = {1}".format(datetime.now(), task_id))
            self.db_con.commit()
            self.get_logger().info("SUCCESS INSERT START_TIME")

        else:
            msg.data = [0, 0] 


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

            self.cur.execute("insert into book_table (order_name, book_time) values('bipoom_{0}_{1}', '{2}')".format(name, counts, datetime.now()))
            self.cur.execute("update bipoom_table set counts = {0} where id = {1}".format(current_count, name))
            self.db_con.commit()  # 이 코드가 있어야 DB에 적용됨
            self.get_logger().info("SUCCESS UPDATE BIPOOM INFO")
            self.do_task()


    def task_complete_callback(self, msg):

        task_complete_info = msg.data
        id = task_complete_info.split("_")[1]

        self.cur.execute("update book_table set end_time = '{0}' where id = {1}".format(datetime.now(), int(id)))
        self.db_con.commit()
        self.get_logger().info("SUCCESS INSERT END_TIME")
        self.do_task()


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









    # def send_bipoom_info(self):

    #     # 1은 창고 2는 청소모드 0은 충전소
    #     msg = Int8MultiArray()
        
    #     self.cur.execute("SELECT * FROM book_table WHERE (start_time IS NULL) AND (order_name LIKE 'b%') ORDER BY id LIMIT 1;")
    #     result = self.cur.fetchall()
        
    #     if not (result == []):
    #         bipoom_info = result[0][1].split("_")[1]
    #         task_id = result[0][0]
           
    #         msg.data = [1, int(bipoom_info), task_id]
    #     else:
    #         msg.data = [0, 0] 
        
    #     self.task_info_pub.publish(msg)
    #     self.get_logger().info("SUCCESS SEND BIPOOM INFO")


    # def send_clean_info(self):

    #     # 1은 창고 2는 청소모드 0은 충전소
    #     msg = Int8MultiArray()
        
    #     self.cur.execute("SELECT * FROM book_table WHERE (start_time IS NULL) AND (order_name LIKE 'c%') ORDER BY id LIMIT 1;")
    #     result = self.cur.fetchall()
        
    #     if not (result == []):
    #         clean_info = result[0][1].split("_")[1]
    #         task_id = result[0][0]
           
    #         msg.data = [1, int(clean_info), task_id]
    #     else:
    #         msg.data = [0, 0] 
        
    #     self.task_info_pub.publish(msg)
    #     self.get_logger().info("SUCCESS SEND BIPOOM INFO")