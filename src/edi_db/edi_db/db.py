import mysql.connector
import time
import rclpy 
from rclpy.node import Node

class DB(Node):
    def __init__(self):
        super().__init__("edi_db")

        
        self.db_con = mysql.connector.connect(
            host = "database-1.cmdlhagy2um9.ap-northeast-2.rds.amazonaws.com",
            port = 3306,
            user = "admin", 
            password = "dltnals123$",
            database = "armbase"
        )


        cur = self.db_con.cursor()
        cur.execute("CREATE TABLE bipoom_table (code int, name varchar(16), number int)")
    

        self.db_con.close()



def main():
    print('Hi from edi_db.')


if __name__ == '__main__':
    main()
