#!/user/bin/env python

# http://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20broadcaster%20%28Python%29

import rospy
import tf
import tf2_ros
from geometry_msgs.msg import TwistStamped, TransformStamped

"""
This subscribes to the odom topic and broadcasts to the tf2 tree
"""

def handlePose(msg, br):


    t = TransformStamped()
    t.header.stamp = msg.header.stamp
    t.header.frame_id = msg.header.frame_id
    t.child_frame_id = "eraserbot"
    t.transform.translation.x = msg.twist.linear.x
    t.transform.translation.y = msg.twist.linear.y
    t.transform.translation.z = msg.twist.linear.z
    q = tf.transformations.quaternion_from_euler(msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]


    br.sendTransform(t)

if __name__ == '__main__':
    rospy.init_node('odometry_broadcaster')
    br = tf2_ros.TransformBroadcaster()
    rospy.Subscriber('/odometry', TwistStamped, handlePose, br)
    rospy.spin()


    