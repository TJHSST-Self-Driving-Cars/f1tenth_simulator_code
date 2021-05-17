#!/usr/bin/env python3

import rospy
from f1tenth_gym_ros.msg import RaceInfo

MAX_LAP_COUNT = 2


class RaceInfoPrinter(object):

    def __init__(self):
        self.ended = False
        self.subscription = None
        self.final_data = None

    def run(self):
        rospy.init_node('race_info_printer', anonymous=True)
        self.subscription = rospy.Subscriber('/race_info', RaceInfo, self.print_race_info)

        while not self.ended and not rospy.core.is_shutdown():
            rospy.rostime.wallsleep(0.5)

        data = self.final_data
        print("EGO - Laps: %s, Time: %s, Collision: %s" % (
            data.ego_lap_count, data.ego_elapsed_time, data.ego_collision
        ))

    def print_race_info(self, data):
        """
        :param data:
            Header header
            float32 ego_lap_count
            float32 opp_lap_count
            float32 ego_elapsed_time
            float32 opp_elapsed_time
            bool ego_collision
            bool opp_collision
        :return: None
        """
        if data.ego_collision or data.ego_lap_count >= MAX_LAP_COUNT:
            self.ended = True
            self.final_data = data

            # we don't need this anymore
            self.subscription.unregister()


if __name__ == "__main__":
    # start listening
    race_info_printer = RaceInfoPrinter()
    race_info_printer.run()
