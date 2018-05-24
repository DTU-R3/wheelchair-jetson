#!/usr/bin/env python
import rospy
import math
import tf
import tf2_ros
import tf2_msgs.msg

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

class zed_footprint_odom(object):
  def __init__(self):
    self.pre_odo = Odometry()
    self.pre_odo_get = False

    # Init ROS node
    rospy.init_node('zed_odom_tf')
    
    # Publishers
    self._OdometryTransformBroadcaster = tf.TransformBroadcaster()
    self._OdometryPublisher = rospy.Publisher("odom", Odometry, queue_size=10)
    
    # Subscribers
    rospy.Subscriber('zed/odom', Odometry, self.zedCB)

  def Start(self):
    while not rospy.is_shutdown():
      rospy.sleep(1)
  
  def zedCB(self, odo):
    ros_now = rospy.Time.now()
    self._OdometryTransformBroadcaster.sendTransform(
      (odo.pose.pose.position.x, odo.pose.pose.position.y, 0),
      (odo.pose.pose.orientation.x, odo.pose.pose.orientation.y, odo.pose.pose.orientation.z, odo.pose.pose.orientation.w),
      ros_now,
      "zed_footprint",
      "odom"
    )
    
    # Set the frame and position for the odometry
    odometry = odo
    odometry.header.frame_id = "odom"
    odometry.header.stamp = ros_now
    odometry.child_frame_id = "zed"
    odometry.pose.pose.position.z = 0

    # Set the velocity for the odometry
    if self.pre_odo_get:
      # time between two measurement, in ms
      t_diff = str((odo.header.stamp - self.pre_odo.header.stamp) / 1000000)
      t = float(t_diff) / 1000
      if t==0:
        return
      odo_euler = tf.transformations.euler_from_quaternion((odo.pose.pose.orientation.x, odo.pose.pose.orientation.y, odo.pose.pose.orientation.z, odo.pose.pose.orientation.w))
      pre_odo_euler = tf.transformations.euler_from_quaternion((self.pre_odo.pose.pose.orientation.x, self.pre_odo.pose.pose.orientation.y, self.pre_odo.pose.pose.orientation.z, self.pre_odo.pose.pose.orientation.w))
      vel_x = (odo.pose.pose.position.x - self.pre_odo.pose.pose.position.x) / t
      vel_y = (odo.pose.pose.position.y - self.pre_odo.pose.pose.position.y) / t
      odo.twist.twist.linear.x = math.sqrt( vel_x**2 + vel_y**2 )
      ang_vel_z = (odo_euler[2] - pre_odo_euler[2]) / t
      odo.twist.twist.angular.z = ang_vel_z
    self._OdometryPublisher.publish(odometry)
    self.pre_odo = odo
    self.pre_odo_get = True

if __name__ == '__main__':
  zed_footprint = zed_footprint_odom() 
  zed_footprint.Start()
