#!/usr/bin/python

import rospy
import math
from ardrone_autonomy.msg import Navdata
from std_msgs.msg import Float32


global ax
global ay
global az

def fax(mssg):
	global ax
	global ay
	global az
	ax = mssg.ax
	ay = mssg.ay
	az = mssg.az

def main():
	global ax
	global ay
	global az
	ax = 0
	ay = 0
	az = 0
	rospy.init_node('python_ardrone')
	Sax = rospy.Subscriber('/ardrone/navdata', Navdata, fax)
	Pfi = rospy.Publisher("~fi", Float32, queue_size=30)
	Pteta = rospy.Publisher("~teta", Float32, queue_size=30)
	delay = rospy.Rate(15)
	fi = Float32()
	teta = Float32()
	while not rospy.is_shutdown():
		fi.data = math.atan2(ay, ax)
		teta.data = -0*math.atan2(ax, math.sqrt(ay*ay+az*az))
		Pfi.publish(fi)
		Pteta.publish(teta)
		delay.sleep()

def __init__(self):
    
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.camera_callback) 
        self.bridge_object = CvBridge()
        self.speed_pub = rospy.Publisher ("/cmd_vel", Twist, queue_size=1)     


def camera_callback(self,data):
        try:
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        height, width, channels = cv_image.shape
        descentre = 160
        rows_to_watch = 20
        crop_img = cv_image[(height)/2+descentre:(height)/2+(descentre+rows_to_watch)][1:width]
        hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
        upper_red=np.array([26,23,190])
        lower_red=np.array([6,3,170]) 

        mask = cv2.inRange(hsv, lower_red, upper_red)
       
        m = cv2.moments(mask, False)
        try:
            cx, cy = m['m10']/m['m00'], m['m01']/m['m00']
        except ZeroDivisionError:
            cy, cx = height/2, width/2

        error_x = cx - width / 2;
        speed_cmd = Twist();
        speed_cmd.linear.x = 0.2;
        speed_cmd.angular.z = -error_x / 100;
        
        self.speed_pub.publish(speed_cmd)

      

if __name__=='__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
