#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from collections import deque
import threading

# Define the buffer
buffer_size = 10
buffer = deque(maxlen=buffer_size)

# Define a lock for thread-safe operations on the buffer
buffer_lock = threading.Lock()

# Define the publisher
pub = rospy.Publisher('/joint_states', JointState, queue_size=10)

def callback(data):
    # This function is called whenever a new joint state is published
    with buffer_lock:
        buffer.append(data)
        # buffer[-1].header.stamp = rospy.Time.now()
        # pub.publish(buffer[-1])  # Publish the most recent joint state
        # print(len(buffer))

def listener():
    # Initialize the node
    rospy.init_node('buffered_joint_state_publisher', anonymous=True)

    # Subscribe to the joint state topic
    rospy.Subscriber('/joint_states_raw', JointState, callback)

    # Keep the node running until it's shut down
    # rospy.spin()

if __name__ == '__main__':
    listener()

    r = rospy.Rate(5)
    while not rospy.is_shutdown():
        if len(buffer) > 0:
            with buffer_lock:
                buffer[-1].header.stamp = rospy.Time.now()
                pub.publish(buffer[-1])
        r.sleep()