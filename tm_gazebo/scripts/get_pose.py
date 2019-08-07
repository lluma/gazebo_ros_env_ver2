import os
import rospy
import math
import tf
import geometry_msgs.msg
from argparse import ArgumentParser

parser = ArgumentParser()
parser.add_argument("--type", "-t", help="Specify the pose type.", default="rpy")

class PoseReader:
  
  def __init__(self):
    
    self.tf_listener = tf.TransformListener()
    self.rate = rospy.Rate(10.0)

  def start(self):
    
    while not rospy.is_shutdown():
      
      try:
        (trans, rot) = self.tf_listener.lookupTransform('/tf', rospy.Time(0))
      except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        continue
      
      print ('Tran:', trans)
      print ('Rot:', rot)
      print ('---------------------------------------------------------')
      self.rate.sleep()
    

def main(args):
  rospy.init_node('tf_listener')
  reader = PoseReader()
  reader.start()
  rospy.spin()

if __name__ == "__main__":
  args = parser.parse_args()
  main(args)
