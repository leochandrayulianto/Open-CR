import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

import cv2
import numpy

def nothing(x):
    pass

# Creating a window with black image
img = numpy.zeros((300, 512, 3), numpy.uint8)
cv2.namedWindow('image')
cv2.createTrackbar('panKp', 'image', 0, 100, nothing)
cv2.createTrackbar('panKd', 'image', 0, 1000, nothing)
cv2.createTrackbar('tiltKp', 'image', 0, 100, nothing)
cv2.createTrackbar('tiltKd', 'image', 0, 1000, nothing)

def main(args=None):
    rclpy.init(args=args)
    node = Node('trackbar')

    pub = node.create_publisher(
        Float64MultiArray, 'trackbar', 1
    )

    msg = Float64MultiArray()

    while rclpy.ok():
        # show image
        cv2.imshow('image', img)
        # for button pressing and changing
        k = cv2.waitKey(1) & 0xFF
        if k == 27:
            break
        # get current positions of all Three trackbars
        panKp = cv2.getTrackbarPos('panKp', 'image')
        panKd = cv2.getTrackbarPos('panKd', 'image')
        tiltKp = cv2.getTrackbarPos('tiltKp', 'image')
        tiltKd = cv2.getTrackbarPos('tiltKd', 'image')
        msg.data = [
            float(panKp) / 100,
            float(panKd) / 1000000,
            float(tiltKp) / 100,
            float(tiltKd) / 1000000
        ]
        pub.publish(msg)
        print(
            float(panKp) / 100,
            float(panKd) / 1000000,
            float(tiltKp) / 100,
            float(tiltKd) / 1000000
        )
        mapImage = numpy.zeros((800,1100,3), numpy.uint8)
        mapImage[:] = (0, 255, 0)
        cv2.rectangle(mapImage, (100,100), (1000,700), (255,255,255), 3) # Garis Luar
        cv2.rectangle(mapImage, (40,530), (100,270), (255,255,255), 3) # Garis Luar Gawang Kiri
        cv2.rectangle(mapImage, (1000,530), (1060,270), (255,255,255), 3) # Garis Luar Gawang Kiri
        cv2.rectangle(mapImage, (100,650), (200,150), (255,255,255), 3) # Garis Luar Gawang Kiri
        cv2.rectangle(mapImage, (900,650), (1000,150), (255,255,255), 3) # Garis Luar Gawang Kiri
        cv2.line(mapImage, (550,100), (550,700), (255,255,255), 3) # Garis Tengah
        cv2.circle(mapImage, (550,400), 75, (255,255,255), 3) # Lingkaran Tengah
        cv2.circle(mapImage, (250,400), 3, (255,255,255), 5)
        cv2.circle(mapImage, (850,400), 3, (255,255,255), 5)
        cv2.line(mapImage, (100,200), (1000,200), (0,0,0), 1)
        cv2.line(mapImage, (100,300), (1000,300), (0,0,0), 1)
        cv2.line(mapImage, (100,400), (1000,400), (0,0,0), 1)
        cv2.line(mapImage, (100,500), (1000,500), (0,0,0), 1)
        cv2.line(mapImage, (100,600), (1000,600), (0,0,0), 1)
                        
        cv2.line(mapImage, (200,100), (200,700), (0,0,0), 1)
        cv2.line(mapImage, (300,100), (300,700), (0,0,0), 1)
        cv2.line(mapImage, (400,100), (400,700), (0,0,0), 1)
        cv2.line(mapImage, (500,100), (500,700), (0,0,0), 1)
        cv2.line(mapImage, (600,100), (600,700), (0,0,0), 1)
        cv2.line(mapImage, (700,100), (700,700), (0,0,0), 1)
        cv2.line(mapImage, (800,100), (800,700), (0,0,0), 1)
        cv2.line(mapImage, (900,100), (900,700), (0,0,0), 1)

        smallMapImage = cv2.resize(mapImage, (640,480), interpolation = cv2.INTER_AREA)
        #cv2.imshow("Barelang Localization", smallMapImage)
        # rclpy.spin_once(node)
    rclpy.shutdown()
    # close the window
    cv2.destroyAllWindows()
if __name__ == '__main__':
    main()
