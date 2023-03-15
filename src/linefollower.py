#!/usr/bin/env python
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist


class Follower:
  def __init__(self):
    self.bridge = cv_bridge.CvBridge()
    cv2.namedWindow("window", 1)
    self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
                                      Image, self.image_callback)
    self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    self.twist = Twist()
    self.isStart = True

  
  
  def image_callback(self, msg):
    image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_yellow = numpy.array([ 20,  100,  100])
    upper_yellow = numpy.array([30, 255, 255])
    h, w, d = image.shape
    
    # only detect yellow area nearby
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    search_top = (int)(3*h/4)
    search_bot = (int)(3*h/4 + 20)
    mask[0:search_top, 0:w] = 0
    mask[search_bot:h, 0:w] = 0
    
    # detect any yellow area in front
    mask_0 = cv2.inRange(hsv, lower_yellow, upper_yellow)
    search_left = (int)(9*h/20)
    search_right = (int)(11*h/20)
    mask_0[0:h, 0:search_left] = 0
    mask_0[0:h, search_right:w] = 0
    
    M = cv2.moments(mask)
    M_0 = cv2.moments(mask_0)
    
    print(self.isStart)
    
    # try to move to yellow line at the beginning
    if self.isStart:
      if M['m00'] > 0:
        self.isStart = False
      elif M_0['m00'] > 0:
        self.twist.linear.x = 0.2
        self.twist.angular.z = 0
        self.vel_pub.publish(self.twist)  
      else:
        self.twist.linear.x = 0
        self.twist.angular.z = -1
        self.vel_pub.publish(self.twist)
      
    else:  
     
      # follow the line
      if M['m00'] > 0:
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
        err = cx - w/2
        self.twist.linear.x = 0.2
        self.twist.angular.z = -float(err) / 300
        self.vel_pub.publish(self.twist) 
      # double back along the line
      else:
        self.twist.linear.x = 0
        self.twist.angular.z = -1
        self.vel_pub.publish(self.twist)
      
    cv2.imshow("window", image)
    cv2.waitKey(3)

rospy.init_node('follower')
follower = Follower()
rospy.spin()
