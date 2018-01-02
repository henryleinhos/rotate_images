#!/usr/bin/env python

from __future__ import print_function

import roslib
roslib.load_manifest('rotate_images')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

DEFAULT_IMAGE_TOPIC = '/ps3eye/left/image_raw'
DEFAULT_ROTATE_ANGLE = -90.0

class image_converter:

  def __init__(self,topic,angle):
    self.image_pub = rospy.Publisher(topic+'/rotated',Image, queue_size=2)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber(topic,Image,self.callback, queue_size=2)

    self.angle = angle

    print(self.angle)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    (nr,nc,channels) = cv_image.shape

    ## smooth rotation by arbitrary angle.  
    ## (need to resize image to accomodate)
    ## center = (nc/2, nr/2)
    ## M = cv2.getRotationMatrix2D(center, -90.0, 1.0)
    ## rotated = cv2.warpAffine(cv_image, M, (nc, nr))
    transp = cv2.transpose(cv_image)
    
    if self.angle == 90.0:
      rotated = cv2.flip(transp,flipCode=0)
    elif self.angle == -90.0:
      rotated = cv2.flip(transp,flipCode=1)
    
    cv2.imshow("rotated image", rotated)
    cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(rotated, "bgr8"))
    except CvBridgeError as e:
      print(e)

def main(args):
  rospy.init_node('image_rotate', anonymous=True)
  # get image topic
  topic = rospy.get_param('~image_topic', DEFAULT_IMAGE_TOPIC)
  angle = rospy.get_param('~image_angle', DEFAULT_ROTATE_ANGLE)

  # initialize the cv converter object
  ic = image_converter(topic,angle)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

