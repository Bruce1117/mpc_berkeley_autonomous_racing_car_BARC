#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import rospy
from barc.msg import camera_out
from geometry_msgs.msg import Point
def camerapub():
    # Initialize the ROS node
    rospy.init_node('talker', anonymous=True)

    # Create a publisher that publishes messages of type String on the 'chatter' topic
    cam_pub = rospy.Publisher('/camera_out', camera_out, queue_size=10)

    # Set the publishing rate (10 Hz in this case)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        # Create a String message
        message = camera_out()
        point = Point()
        point.x = 0
        point.y = 0
        point.z = 0
        for i in range(5):
            point.x += 0.1
            point.y += 0.1
            point.z += 0.1
            message.s.append(point)
        message.curv_s = [0.2, 0.5, 0.7, 0.9, 1.1]
        message.ey = 0.25
        # Publish the message
        rospy.loginfo(message)
        cam_pub.publish(message)

        # Sleep to maintain the publishing rate
        rate.sleep()



if __name__ == "__main__":
    try:
        camerapub()
    except rospy.ROSInterruptException:
        pass
