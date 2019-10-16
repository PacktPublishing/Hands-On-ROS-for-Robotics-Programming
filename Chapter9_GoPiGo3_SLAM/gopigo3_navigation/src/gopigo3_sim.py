#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64

def main():
  pub_right = rospy.Publisher("float64/right", Float64, queue_size=10)
  pub_left = rospy.Publisher("float64/left", Float64, queue_size=10)
  rospy.init_node('gopigo3_sim', anonymous=True)
  right_i = 0
  left_i = 0
  r = rospy.Rate(10) # 100hz

  while not rospy.is_shutdown():
    pub_right.publish(right_i)
    pub_left.publish(left_i)
    right_i += 1
    left_i += 0.8
    r.sleep()

if __name__ == "__main__":
  main()
