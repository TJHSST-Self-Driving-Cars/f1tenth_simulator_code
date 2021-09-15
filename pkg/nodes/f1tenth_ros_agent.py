#!/usr/bin/env python3

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
import numpy as np
import os
import time
import rosgraph
import socket

from pkg.drivers import DisparityExtender as Driver

"""
NOTE: Following code enables F1Tenth - Docker - ROS integration.  
Please don't change it, unless you know what you're doing
"""


class ROSRunner:
    def __init__(self, driver, agent_name):
        self.driver = driver
        self.agent_name = agent_name
        self.pub_drive = None
        self.roscore_started = False
        self.last_healthy_time = None

    def lidar_callback(self, data):
        ranges = np.asarray(data.ranges)
        speed, angle = self.driver.process_lidar(ranges)

        self.last_healthy_time = time.time()

        # create message & publish
        msg = AckermannDriveStamped()
        msg.drive.speed = speed
        msg.drive.steering_angle = angle
        self.pub_drive.publish(msg)


    def run(self):
        rospy.init_node('gym_agent_%s' % self.agent_name, anonymous=True)
        self.pub_drive = rospy.Publisher('/%s/drive' % self.agent_name, AckermannDriveStamped, queue_size=5)

        # start listening
        rospy.Subscriber('/%s/scan' % self.agent_name, LaserScan, self.lidar_callback)
        rospy.sleep(3)

        while not rospy.core.is_shutdown():
            current_time = time.time()
            
            # if it passed more than 15 seconds since last sensor
            # try to connect to ROS, if you cannot
            if self.last_healthy_time and current_time > self.last_healthy_time + 1:
                try:
                    rosgraph.Master('/rostopic').getPid()
                except socket.error:
                    return
                else:
                    self.last_healthy_time = time.time()
            
            rospy.rostime.wallsleep(0.04)


if __name__ == "__main__":
    agent_name = os.environ.get("F1TENTH_AGENT_NAME")
    runner = ROSRunner(Driver(), agent_name)
    
    print ("Starting Agent: %s" % agent_name)
    
    # launch
    runner.run()

    print ("Agent run finished")
    

