#!/usr/bin/env python
import rospy
import math

from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan

class scan_handler(object):
  def __init__(self):
    self.range = 15.0 * math.pi / 180.0
    self.front_range = Float32()
    self.left_range = Float32()
    self.right_range = Float32()

    # Init ROS node
    rospy.init_node('scan_handler')
    
    # Publishers
    self.frontPub = rospy.Publisher("wheelchair/front", Float32, queue_size=10)
    self.leftPub = rospy.Publisher("wheelchair/left", Float32, queue_size=10)
    self.rightPub = rospy.Publisher("wheelchair/right", Float32, queue_size=10)

    
    # Subscribers
    rospy.Subscriber('zed/scan', LaserScan, self.scanCB)

  def Start(self):
    while not rospy.is_shutdown():
      rospy.sleep(1)
  
  def scanCB(self, scan):
    angle_increment = scan.angle_increment
    thres = int(self.range / angle_increment)

    # measurement of the right side
    r_min = 255.0
    for measurement in scan.ranges[:thres]:
      if math.isnan(measurement):
        continue
      if measurement < r_min:
        r_min = measurement
    self.right_range.data = r_min
    self.rightPub.publish(self.right_range)
    
    # measurement of the right side
    f_min = 255.0
    for measurement in scan.ranges[thres:-thres]:
      if math.isnan(measurement):
        continue
      if measurement < f_min:
        f_min = measurement
    self.front_range.data = f_min
    self.frontPub.publish(self.front_range)

    # measurement of the right side
    l_min = 255.0
    for measurement in scan.ranges[-thres:]:
      if math.isnan(measurement):
        continue
      if measurement < l_min:
        l_min = measurement
    self.left_range.data = l_min
    self.leftPub.publish(self.left_range)


if __name__ == '__main__':
  scan_hl = scan_handler() 
  scan_hl.Start()
