import rclpy
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

disp = Adafruit_SSD1306.SSD1306_128_64(rst=None, i2c_bus=8, gpio=1)

# # Initialize library.
disp.begin()

# Clear display.
disp.clear()
disp.display()

# Create blank image for drawing.
# Make sure to create image with mode '1' for 1-bit color.
width = disp.width
height = disp.height
image = Image.new('1', (width, height))

# Get drawing object to draw on image.
draw = ImageDraw.Draw(image)

# Draw a black filled box to clear the image.
draw.rectangle((0, 0, width, height), outline=0, fill=0)

# Draw some shapes.
# First define some constants to allow easy resizing of shapes.
padding = -2
top = padding
bottom = height-padding
# Move left to right keeping track of the current x position for drawing shapes.
x = 0
# Load default font.
font = ImageFont.load_default()

r = ""
p = ""
y = ""
condition = 0
last_condition = 0
strategy = ""
voltage = ""
state = ""
pan = ""
tilt = ""
ball_x = ""
ball_y = ""
def immu_callback(msg):
    global r, p ,y
    r = str(int(msg.angular_velocity.x))
    p = str(int(msg.angular_velocity.y))
    y = str(int(msg.angular_velocity.z))

def button_callback(msg):
    global condition
    global strategy
    strategy = msg.strategy
    condition = msg.kill
    """try:
	    if condition == 0:
	    	draw.rectangle((0, 0, width, height), outline=0, fill=0)
	    elif condition == 1:
	    	draw.rectangle((0, 0, width, height), outline=0, fill=255)
	    disp.image(image)
	    disp.display()
    except KeyboardInterrupt:
        print("Exiting Program")
    except Exception as exception_error:
        print("Error occurred. Exiting Program")
        print("Error: " + str(exception_error))
    finally:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        pass"""

def voltage_callback(msg):
    global voltage
    voltage = str(msg.data)

def state_callback(msg):
    global state
    state = str(msg.state)

def head_callback(msg):
    global pan, tilt
    pan = str(round(msg.pan, 2))
    tilt = str(round(msg.tilt, 2))

def ball_callback(msg):
    global ball_x, ball_y
    ball_y = str(msg.data[1])
    ball_x = str(msg.data[0])

def main(args=None):
    global r, p, y, strategy, condition, voltage, state, ball_x, ball_y, last_condition
    rclpy.init(args=args)
    node = rclpy.create_node('oled')

    sub_imu = node.create_subscription(
        Imu, 'imu', immu_callback, 10
    )

    sub_button = node.create_subscription(
        Button, 'button', button_callback, 10
    )

    sub_volt = node.create_subscription(
        Int32, 'voltage', voltage_callback, 10
    )

    sub_state = node.create_subscription(
        Coordination, 'coordination', state_callback, 10
    )

    sub_head = node.create_subscription(
        HeadMovement, 'head', head_callback, 10
    )

    sub_ball = node.create_subscription(
        Int32MultiArray, 'vision', ball_callback, 10
    )

    #rclpy.spin(node)
    #node.destroy_node()
    #rclpy.shutdown()

    try:
        while rclpy.ok():   
            # print(r, p, y, strategy, condition)
            # oled
            # Draw a black filled box to clear the image.
            time.sleep(0.5)
            if condition != last_condition:
            	if condition == 0:
            		draw.rectangle((0, 0, width, height), outline=0, fill=0)
            		last_condition = condition
            	elif condition == 1:
            		draw.text((x+25, top), "BARELANG FC",  font=font, fill=255)
            		draw.text((x+31.5, top+8), "POLIBATAM",  font=font, fill=255)
            		last_condition = condition
            	disp.image(image)
            	disp.display()
            rclpy.spin_once(node)

    except KeyboardInterrupt:
        print("Exiting Program")

    except Exception as exception_error:
        print("Error occurred. Exiting Program")
        print("Error: " + str(exception_error))

    finally:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        node.destroy_node()
        rclpy.shutdown() 
        pass

if __name__ == '__main__':
    main()
