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
#font = ImageFont.load_default()

# Load a larger TrueType font (size 20)
large_font = ImageFont.truetype("/home/tegra/bfc_ros2/src/sensor_oled/sensor_oled/MANBORT.otf", 10)

r = ""
p = ""
y = ""
r1 = ""
p1 = ""
y1 = ""
condition_new = 0
#strategy_new = 0
last_condition = 0
strategy_new = ""
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
    
def immunew_callback(msg):
    global r1, p1 ,y1
    r1 = str(int(msg.angular_velocity.x))
    p1 = str(int(msg.angular_velocity.y))
    y1 = str(int(msg.angular_velocity.z))
    
def buttonnew_callback(msg):
    global condition_new
    global strategy_new
    strategy_new = msg.strategy
    condition_new = msg.kill

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
    
    sub_imu_new = node.create_subscription(
        Imu, 'imu_bno', immunew_callback, 10
    )

    sub_button = node.create_subscription(
        Button, 'button', button_callback, 10
    )
    
    sub_button_new = node.create_subscription(
        Button, 'button_new', buttonnew_callback, 10
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

    try:
        while rclpy.ok():   
            #print(r, p, y)
            #print(condition_new)
            #print(strategy_new)

            # Menyiapkan teks yang ingin ditampilkan di layar OLED
            text_to_display = "\nSTRATEGY :  {strategy}  \nCONDITION :  {condition}  \nRPY : {roll} ,  {pitch},  {yaw}".format(
            strategy=strategy_new, condition=condition_new, roll=r1, pitch=p1, yaw=y1)

            # Menggambar teks pada layar OLED
            draw.rectangle((0, 0, width, height), outline=0, fill=0)
            draw.text((x+13, top), "BARELANG FC 1", font=large_font, fill=25)
            draw.text((x, top), text_to_display, font=large_font, fill=5)
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
