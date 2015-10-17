#!/usr/bin/env python
import roslib
roslib.load_manifest('baxter_examples')
import sys
import rospy
import cv
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
 
class image_converter:
 
  def __init__(self):
    self.image_pub = rospy.Publisher("/robot/xdisplay",Image)

    #cv2.namedWindow("Image window", 1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/cameras/head_camera/image",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv(data, "bgr8")
    except CvBridgeError, e:
      print e

    #(rows,cols,channels) = cv_image.shape
    #if cols > 60 and rows > 60 :
    #  cv.circle(cv_image, (50,50), 10, 255)

    #cv2.imshow("Image window", cv_image)
    #cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError, e:
      print e

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv2.destroyAllWindows()

if __name__ == '__main__':
  main(sys.argv)

