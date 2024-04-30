import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from bfc_msgs.msg import Button
from bfc_msgs.msg import Coordination
from bfc_msgs.msg import HeadMovement
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Imu

import Adafruit_SSD1306   # This is the driver chip for the Adafruit PiOLED
from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont
import time

class oled(Node):

    def __init__(self):
        super().__init__('oled_sub')
        self.sub_imu = self.create_subscription(
            Imu,
            'imu',
            self.imu_callback,
            10)

        self.sub_button = self.create_subscription(
            Button,
            'button',
            self.button_callback,
            10)
        
        self.sub_volt = self.create_subscription(
            Int32,
            'voltage',
            self.voltage_callback,
            10)

        self.sub_state = self.create_subscription(
            Coordination,
            'coordination',
            self.state_callback,
            10)

        self.sub_head = self.create_subscription(
            HeadMovement,
            'head',
            self.head_callback,
            10)

        self.sub_ball = self.create_subscription(
            Int32MultiArray,
            'vision',
            self.ball_callback,
            10)


        # prevent unused variable warning
        self.sub_imu
        self.sub_ball
        self.sub_head
        self.sub_state
        self.sub_volt
        self.sub_button
        self.ball_y = ""
        self.ball_x = ""
        self.tilt = ""
        self.pan = ""
        self.state = ""
        self.voltage = ""
        self.strategy = ""
        self.condition = ""
        self.r = ""
        self.p = ""
        self.y = ""





    def timer_callback(self):
        disp = Adafruit_SSD1306.SSD1306_128_64(rst=None, i2c_bus=8, gpio=1)
        disp.begin()
        disp.display()
        width = disp.width
        height = disp.height
        image = Image.new('1', (width, height))
        draw = ImageDraw.Draw(image)
        draw.rectangle((0, 0, width, height), outline=0, fill=0)
        padding = -2
        top = padding
        _bottom = height-padding
        x = 0
        font = ImageFont.load_default()
        # print(r, p, y, strategy, condition)
        # oled
        # Draw a black filled box to clear the image.
        time.sleep(0.05)
        draw.rectangle((0, 0, width, height), outline=0, fill=0)
        draw.text((x, top), "BARELANG 1",  font=font, fill=255)
        draw.text((x, top+8), "STRATEGY : " + str(self.strategy),  font=font, fill=255)
        draw.text((x, top+16), "CONDITION : " + str(self.condition),  font=font, fill=255)
        draw.text((x, top+24), "RPY : " + self.r + ',' + self.p + ',' + self.y,  font=font, fill=255)
        draw.text((x, top+32), "VOLTAGE : " + self.voltage,  font=font, fill=255)
        draw.text((x, top+40), "STATE : " + self.state,  font=font, fill=255)
        draw.text((x, top+48), "HEAD : " + self.pan + ", " + self.tilt,  font=font, fill=255)
        draw.text((x, top+56), "BALL : " + self.ball_x + ", " + self.ball_y,  font=font, fill=255)
        # Display image.
        # Set the SSD1306 image to the PIL image we have made, then dispaly
        disp.image(image)
        disp.display()



    def ball_callback(self, msg):
        self.ball_y = str(msg.data[1])
        self.ball_x = str(msg.data[0])

    def head_callback(self, msg):
        self.pan = str(round(msg.pan, 2))
        self.tilt = str(round(msg.tilt, 2))

    def state_callback(self, msg):
        self.state = str(msg.state)

    def voltage_callback(self, msg):
        self.voltage = str(msg.data)

    def button_callback(self, msg):
        self.strategy = msg.strategy
        self.condition = msg.kill

    def imu_callback(self, msg):
        self.r = str(int(msg.angular_velocity.x))
        self.p = str(int(msg.angular_velocity.y))
        self.y = str(int(msg.angular_velocity.z))



def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()