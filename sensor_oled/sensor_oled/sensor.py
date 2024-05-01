import rclpy
from bfc_msgs.msg import Button
import time

import serial
import binascii

import os

serial_port = serial.Serial(
    port="/dev/ttyTHS0",
    baudrate=115200,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
)

serial_port.flush()

strategyNumber = 0
kill = 0 

def main(args=None):
    global strategyNumber
    global kill
    
    rclpy.init(args=args)

    node = rclpy.create_node('button_imu')

    pub_button = node.create_publisher(Button, 'button', 10)
    msg_button = Button()
    dataIn = []
    lastState = 0
    data = ""
    try:
        while rclpy.ok():
            time.sleep(0.05)
            if serial_port.inWaiting() > 0:
                data = serial_port.read()
                dataStr = data.decode("utf-8")
                print("dataStr : ",dataStr)
                if dataStr != '\x00':
                    dataIn.append(dataStr)
                    
                if len(dataIn) == 2:
                    if dataIn[1] != '\x00' or dataIn[1] != '*' or dataIn[1] != '#':
                        if dataIn[0] == '*':
                            kill = int(dataIn[1])
                            dataIn.clear()
                        elif dataIn[0] == '#':
                            strategyNumber = int(dataIn[1])
                            dataIn.clear()
                        dataIn.clear()
                print(kill, strategyNumber)
            msg_button.kill = kill
            msg_button.strategy = strategyNumber
            pub_button.publish(msg_button)
                    
    except KeyboardInterrupt:
        print("Exiting Program")

    finally:
        serial_port.close()
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        node.destroy_node()
        rclpy.shutdown() 
        pass

if __name__ == '__main__':
    main()
