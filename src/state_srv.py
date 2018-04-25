#!/usr/bin/env python
import rospy

from geometry_msgs.msg import Vector3, TwistStamped
from eraserbot.srv import StateSrv, StateSrvResponse


class StateService:
  #Callback for when an image is received
  def stateReceived(self, message):
    #Save the image in the instance variable
    self.state.x = message.twist.linear.x
    self.state.y = message.twist.linear.y
    self.state.z = message.twist.angular.z

    #Print an alert to the console
    #print(rospy.get_name() + ":Image received!")

  #When another node calls the service, return the last image
  def getState(self, request):
    #Print an alert to the console
    #print("Image requested!")

    #Return the last image
    return StateSrvResponse(self.state)

  def __init__(self):
    #Create an instance variable to store the last image received
    self.state = Vector3();

    #Initialize the node
    rospy.init_node('state_listener')

    #Subscribe to the image topic
    rospy.Subscriber("/odometry", TwistStamped, self.stateReceived)

    #Create the service
    rospy.Service('current_state', StateSrv, self.getState)

  def run(self):
    rospy.spin()

#Python's syntax for a main() method
if __name__ == '__main__':
  node = StateService()
  node.run()
