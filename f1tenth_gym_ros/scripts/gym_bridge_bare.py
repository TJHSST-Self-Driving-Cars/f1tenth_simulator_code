#!/usr/bin/env python
import rospy
import os
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

from f1tenth_gym_ros.msg import RaceInfo

from tf2_ros import transform_broadcaster

import numpy as np
import gym


class GymBridge(object):
    def __init__(self):
        # get env vars
        self.race_scenario = os.environ.get('RACE_SCENARIO')
        if self.race_scenario is None:
            print('Race scenario not set! Using single car timed trial as default.')
            self.race_scenario = 0
        else:
            self.race_scenario = int(self.race_scenario)
        self.ego_id = os.environ.get('EGO_ID')
        self.opp_id = os.environ.get('OPP_ID')
        if self.opp_id is None:
            print('Opponent id not set! Using opp_id as default.')
            self.opp_id = 'opp_id'

        # initialize parameters

        # Topic Names
        self.race_info_topic = rospy.get_param('race_info_topic')
        self.ego_scan_topic = '/' + self.ego_id + '/scan'
        self.ego_drive_topic = '/' + self.ego_id + '/drive'
        if self.race_scenario:
            # 2 car grand prix, need extra topics
            self.opp_scan_topic = '/' + self.opp_id + '/scan'
            self.opp_drive_topic = '/' + self.opp_id + '/drive'

        # Map
        self.map_path = os.environ.get('RACE_MAP_PATH')
        self.map_img_ext = os.environ.get('RACE_MAP_IMG_EXT')

        # C++ backend
        exec_dir = os.environ.get('F1TENTH_EXEC_DIR')
        # exec_dir = rospy.get_param('executable_dir')

        # Scan simulation params
        scan_fov = rospy.get_param('scan_fov')
        scan_beams = rospy.get_param('scan_beams')
        self.angle_min = -scan_fov / 2.
        self.angle_max = scan_fov / 2.
        self.angle_inc = scan_fov / scan_beams

        # Vehicle parameters
        wheelbase = 0.3302
        mass = 3.74
        l_r = 0.17145
        I_z = 0.04712
        mu = 0.523
        h_cg = 0.074
        cs_f = 4.718
        cs_r = 5.4562

        # init gym backend
        self.env = gym.make('f110_gym:f110-v0')
        self.env.init_map(self.map_path, self.map_img_ext, False, False)
        self.env.update_params(mu, h_cg, l_r, cs_f, cs_r, I_z, mass, exec_dir, double_finish=True)

        driver_count = 2 if self.race_scenario > 0 else 1
        poses = [[-1.25 + (i * 0.75), 0., np.radians(90)] for i in range(driver_count)]
        if driver_count == 1:
            poses.append([200.0, 200.0, 0.0])

        # calculate initial state
        initial_state = {
            'x': [poses[0][0], poses[1][0]],
            'y': [poses[0][1], poses[1][1]],
            'theta': [poses[0][2], poses[1][2]],
        }
        self.ego_pose = poses[0]
        self.opp_pose = poses[1]

        self.obs, _, self.done, _ = self.env.reset(initial_state)

        self.ego_requested_steer = 0.0
        self.ego_requested_speed = 0.0
        self.ego_drive_published = False

        self.opp_requested_steer = 0.0
        self.opp_requested_speed = 0.0
        self.opp_drive_published = False

        # keep track of latest sim state
        self.ego_scan = list(self.obs['scans'][0])
        self.opp_scan = list(self.obs['scans'][1])

        # keep track of collision
        self.ego_collision = False
        self.opp_collision = False

        # publishers
        self.ego_scan_pub = rospy.Publisher(self.ego_scan_topic, LaserScan, queue_size=1)
        self.info_pub = rospy.Publisher(self.race_info_topic, RaceInfo, queue_size=1)
        if self.race_scenario:
            self.opp_scan_pub = rospy.Publisher(self.opp_scan_topic, LaserScan, queue_size=1)

        # subs
        self.drive_sub = rospy.Subscriber(self.ego_drive_topic, AckermannDriveStamped, self.drive_callback,
                                          queue_size=1)
        if self.race_scenario:
            self.opp_drive_sub = rospy.Subscriber(self.opp_drive_topic, AckermannDriveStamped, self.opp_drive_callback,
                                                  queue_size=1)

        # Timer
        self.timer = rospy.Timer(rospy.Duration(0.004), self.timer_callback)
        self.drive_timer = rospy.Timer(rospy.Duration(0.01), self.drive_timer_callback)

    def update_sim_state(self):
        self.ego_scan = list(self.obs['scans'][0])
        if self.race_scenario:
            self.opp_scan = list(self.obs['scans'][1])

    def drive_callback(self, drive_msg):
        self.ego_requested_speed = drive_msg.drive.speed
        self.ego_requested_steer = drive_msg.drive.steering_angle
        self.ego_drive_published = True

    def opp_drive_callback(self, opp_drive_msg):
        self.opp_requested_speed = opp_drive_msg.drive.speed
        self.opp_requested_steer = opp_drive_msg.drive.steering_angle
        self.opp_drive_published = True

    def drive_timer_callback(self, timer):
        if self.race_scenario:
            # two car
            if self.ego_drive_published and self.opp_drive_published:
                action = {'ego_idx': 0,
                          'speed': [self.ego_requested_speed, self.opp_requested_speed],
                          'steer': [self.ego_requested_steer, self.opp_requested_steer]
                }
                self.obs, step_reward, self.done, info = self.env.step(action)
                self.update_sim_state()
        else:
            # single car
            if self.ego_drive_published:
                action = {'ego_idx': 0, 'speed': [self.ego_requested_speed, 0.], 'steer': [self.ego_requested_steer, 0.]}
                self.obs, step_reward, self.done, info = self.env.step(action)
                self.update_sim_state()

    def timer_callback(self, timer):
        ts = rospy.Time.now()

        def generate_scan_message(name, ranges):
            scan = LaserScan()
            scan.header.stamp = ts
            scan.header.frame_id = '%s/laser' % name
            scan.angle_min = self.angle_min
            scan.angle_max = self.angle_max
            scan.angle_increment = self.angle_inc
            scan.range_min = 0.
            scan.range_max = 30.
            scan.ranges = ranges
            return scan

        scan = generate_scan_message("ego_racecar", self.ego_scan)
        self.ego_scan_pub.publish(scan)

        if self.race_scenario:
            scan = generate_scan_message("opp_racecar", self.opp_scan)
            self.ego_scan_pub.publish(scan)

        # pub race info
        self.publish_race_info(ts)

    def publish_race_info(self, ts):
        info = RaceInfo()
        info.header.stamp = ts
        if not self.ego_collision:
            self.ego_collision = self.obs['collisions'][0]
        if not self.opp_collision:
            self.opp_collision = self.obs['collisions'][1]
        info.ego_collision = self.ego_collision
        info.opp_collision = self.opp_collision
        info.ego_elapsed_time = self.obs['lap_times'][0]
        info.opp_elapsed_time = self.obs['lap_times'][1]
        info.ego_lap_count = self.obs['lap_counts'][0]
        info.opp_lap_count = self.obs['lap_counts'][1]
        self.info_pub.publish(info)


if __name__ == '__main__':
    rospy.init_node('gym_bridge')
    gym_bridge = GymBridge()
    rospy.spin()
