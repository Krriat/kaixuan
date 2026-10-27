import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32

import serial
import time
import threading


class STS3215Node(Node):

    def __init__(self):

        super().__init__('sts3215_node')


        # 串口配置
        self.ser = serial.Serial(
            '/dev/ttyACM0',
            1000000,
            timeout=0.1
        )


        # 舵机ID
        self.id = 3


        # 串口读写锁
        self.lock = threading.Lock()



        # 接收目标位置
        self.sub = self.create_subscription(
            Int32,
            '/sts3215/position',
            self.position_callback,
            10
        )


        # 发布当前位置
        self.pub = self.create_publisher(
            Int32,
            '/sts3215/state',
            10
        )


        # 10Hz读取位置
        self.timer = self.create_timer(
            0.1,
            self.read_position
        )


        self.get_logger().info(
            'STS3215 node started'
        )



    # STS3215校验
    def checksum(self, data):

        return (~sum(data)) & 0xff



    # -------------------------------
    # 写位置
    # -------------------------------
    def move(self, position):

        cmd = [
            0xff,
            0xff,
            self.id,
            0x09,
            0x03,
            0x2a,

            position & 0xff,
            (position >> 8) & 0xff,

            0x00,
            0x00,

            0xe8,
            0x03
        ]


        cmd.append(
            self.checksum(cmd[2:])
        )


        with self.lock:

            self.ser.write(
                bytes(cmd)
            )


        self.get_logger().info(
            f"send position: {position}"
        )



    # ROS2位置指令回调
    def position_callback(self, msg):

        self.move(msg.data)



    # -------------------------------
    # 读取当前位置
    # -------------------------------
    def read_position(self):


        # 读取地址0x38，长度2
        cmd = bytes([
            0xff,
            0xff,
            self.id,
            0x04,
            0x02,
            0x38,
            0x02,
            0xbc
        ])


        with self.lock:

            self.ser.write(cmd)

            time.sleep(0.01)

            data = self.ser.read(8)



        if len(data) == 8:


            # 返回:
            # FF FF ID LEN ERROR POS_L POS_H CHECK

            if data[4] == 0:


                position = (
                    data[5]
                    |
                    (data[6] << 8)
                )


                msg = Int32()

                msg.data = position


                self.pub.publish(msg)



    # 关闭串口
    def destroy_node(self):

        if self.ser.is_open:
            self.ser.close()

        super().destroy_node()



def main():

    rclpy.init()


    node = STS3215Node()


    try:

        rclpy.spin(node)


    except KeyboardInterrupt:

        pass


    finally:

        node.destroy_node()

        rclpy.shutdown()



if __name__ == '__main__':

    main()