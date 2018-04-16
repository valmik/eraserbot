#!/user/bin/env python

# http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20broadcaster%20%28Python%29

import rospy
import tf
from geometry_msgs.msg import TwistStamped

def handlePose(msg):
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z), 
        tf.transformations.quaternion_from_euler(msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z),
        msg.header.stamp, "eraserbot", msg.header.frame_id)

if __name__ == '__main__':
    rospy.init_node('odometry_broadcaster')
    rospy.Subscriber('/odometry', TwistStamped, handlePose)
    rospy.spin()


    