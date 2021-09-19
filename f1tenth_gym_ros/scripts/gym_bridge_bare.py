#!/usr/bin/env python3
import rospy
import os
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance
from ackermann_msgs.msg import AckermannDriveStamped
from tf.transformations import quaternion_from_euler

from f1tenth_gym_ros.msg import RaceInfo
from f1tenth_gym_ros.msg import Observation

current_dir = os.path.abspath(os.path.dirname(__file__))
package_dir = os.path.abspath(os.path.join(current_dir, ".."))

import numpy as np
import gym
import time


def _to_odometry_msg(pose_x, pose_y, pose_theta, linear_vel_x, linear_vel_y, angular_vel_z, **kwargs):
    pose = PoseWithCovariance()
    pose.pose.position.x = pose_x
    pose.pose.position.y = pose_y
    quat = quaternion_from_euler(0., 0., pose_theta)
    pose.pose.orientation.x = quat[0]
    pose.pose.orientation.y = quat[1]
    pose.pose.orientation.z = quat[2]
    pose.pose.orientation.w = quat[3]

    twist = TwistWithCovariance()
    twist.twist.linear.x = linear_vel_x
    twist.twist.linear.y = linear_vel_y
    twist.twist.angular.z = angular_vel_z
    return pose, twist


def _to_scan_msg(ranges, angle_min, angle_max, angle_inc):
    scan = LaserScan()
    scan.angle_min = angle_min
    scan.angle_max = angle_max
    scan.angle_increment = angle_inc
    scan.range_min = 0.
    scan.range_max = 30.
    scan.ranges = ranges
    return scan


class Agent(object):
    def __init__(self, id, drive_callback, scan_fov, scan_beams):
        self.id = id

        # params
        self.angle_min = -scan_fov / 2.
        self.angle_max = scan_fov / 2.
        self.angle_inc = scan_fov / scan_beams

        # observations
        self.collision = False
        self.ego_obs = {}
        self.opp_obs = {}

        # motors
        self.requested_steer = 0.0
        self.requested_speed = 0.0
        self.drive_published = False

        # topics
        self.observations_topic = '/%s/observations' % self.id
        self.drive_topic = '/%s/drive' % self.id
        self.observations_pub = rospy.Publisher(self.observations_topic, Observation, queue_size=1)
        self.drive_sub = rospy.Subscriber(self.drive_topic, AckermannDriveStamped, drive_callback, queue_size=1)

    def update_observersations(self, ego_obs, opp_obs):
        self.ego_obs = ego_obs
        self.opp_obs = opp_obs
        self.collision = ego_obs['collision']

    def publish_observations(self, ts):
        observation = Observation()
        observation.header.stamp = ts
        observation.header.frame_id = 'agent_%s/observation' % self.id
        observation.ranges = self.ego_obs['scan']

        # calculate pose & twist
        observation.ego_pose, observation.ego_twist = _to_odometry_msg(**self.ego_obs)
        if self.opp_obs:
            observation.opp_pose, observation.opp_twist = _to_odometry_msg(**self.opp_obs)

        # send!
        self.observations_pub.publish(observation)

        # FIXME: Send /%/scan too.


class GymBridge(object):

    def __init__(self):
        # get env vars
        self.race_scenario = int(os.environ.get('RACE_SCENARIO', 0))
        self.agents = []

        # this is filled when match is finished
        self.info = {}

        # Scan simulation params
        scan_fov, scan_beams = rospy.get_param('scan_fov'), rospy.get_param('scan_beams')
        self.agents.append(Agent(os.environ.get("EGO_ID"), self.drive_callback, scan_fov, scan_beams))
        if self.race_scenario > 0:
            self.agents.append(Agent(os.environ.get('OPP_ID'), self.opp_drive_callback, scan_fov, scan_beams))

        # Topic Names
        self.race_info_topic = rospy.get_param('race_info_topic')

        # Map
        self.map_path = os.environ.get('RACE_MAP_PATH')
        self.map_img_ext = os.environ.get('RACE_MAP_IMG_EXT')

        # publishers
        self.info_pub = rospy.Publisher(self.race_info_topic, RaceInfo, queue_size=1)

        # Launch
        driver_count = 2 if self.race_scenario > 0 else 1
        self.env = gym.make('f110_gym:f110-v0',
                            map=self.map_path[:-5],
                            map_ext=self.map_img_ext, num_agents=driver_count)

        # init gym backend
        # specify starting positions of each agent
        if driver_count == 1:
            poses = np.array([[0.8007017, -0.2753365, 4.1421595]])
        elif driver_count == 2:
            poses = np.array([
                [0.8007017, -0.2753365, 4.1421595],
                [0.8162458, 1.1614572, 4.1446321],
            ])
        else:
            raise ValueError("Max 2 drivers are allowed")

        self.obs, _, self.done, _ = self.env.reset(poses=np.array(poses))

        if os.environ.get("DISPLAY"):
            self.env.render()

        self._update()

        # Timer
        self.timer = rospy.Timer(rospy.Duration(0.004), self.timer_callback)
        self.drive_timer = rospy.Timer(rospy.Duration(0.02), self.drive_timer_callback)

    def spin(self):
        print("Starting F1Tenth Bridge")

        # run until challenge completes or ros shuts down
        while not self.done and not rospy.core.is_shutdown():
            rospy.rostime.wallsleep(0.04)

            if os.environ.get("DISPLAY"):
                self.env.render()

        print("Shutting down F1Tenth Bridge")

    def _update(self):
        # get keys from observations
        # and redirects them to agents in singular form
        keys = {
            'scans': 'scan',
            'poses_x': 'pose_x',
            'poses_y': 'pose_y',
            'poses_theta': 'pose_theta',
            'linear_vels_x': 'linear_vel_x',
            'linear_vels_y': 'linear_vel_y',
            'ang_vels_z': 'angular_vel_z',
            'collisions': 'collision',
            'lap_times': 'lap_time',
            'lap_counts': 'lap_count'
        }

        # if single agent, we won't have opp obs
        agent_0_obs = {single: self.obs[multi][0] for multi, single in keys.items()}
        if len(self.agents) == 1:
            return self.agents[0].update_observersations(agent_0_obs, None)

        # if multi agent, we have opp obs
        agent_1_obs = {single: self.obs[multi][1] for multi, single in keys.items()}
        self.agents[1].update_observersations(agent_1_obs, agent_0_obs)

    def drive_callback(self, drive_msg):
        self.agents[0].requested_speed = drive_msg.drive.speed
        self.agents[0].requested_steer = drive_msg.drive.steering_angle
        self.agents[0].drive_published = True

    def opp_drive_callback(self, drive_msg):
        self.agents[1].requested_speed = drive_msg.drive.speed
        self.agents[1].requested_steer = drive_msg.drive.steering_angle
        self.agents[1].drive_published = True

    def drive_timer_callback(self, timer):
        published = all([a.drive_published for a in self.agents])

        # until all agents started publishing, we wait
        if not published:
            return

        # update simulation
        actions = [[a.requested_steer, a.requested_speed] for a in self.agents]
        self.obs, _, self.done, _ = self.env.step(np.array(actions))

        # update scan data
        self._update()

        # if match is completed, we set the bridge.info property
        if self.done:
            info = {
                "ego_collision": self.agents[0].collision,
                "ego_elapsed_time": float(self.obs['lap_times'][0]),
                "ego_lap_count": float(self.obs['lap_counts'][0]),
            }

            if len(self.agents) > 1:
                info["opp_collision"] = self.agents[1].collision
                info["opp_elapsed_time"] = float(self.obs['lap_times'][1])
                info["opp_lap_count"] = float(self.obs['lap_counts'][1])
            self.info = info

    def timer_callback(self, timer):
        # once match is completed, stop publishing information
        if self.done:
            return

        ts = rospy.Time.now()
        for i, agent in enumerate(self.agents):
            agent.publish_observations(ts)

        # pub race info
        self.publish_race_info(ts)

    def publish_race_info(self, ts):
        info = RaceInfo()
        info.header.stamp = ts
        info.ego_collision = self.agents[0].collision
        info.ego_elapsed_time = self.obs['lap_times'][0]
        info.ego_lap_count = self.obs['lap_counts'][0]
        info.opp_collision = self.agents[1].collision if len(self.agents) > 1 else 0
        info.opp_elapsed_time = self.obs['lap_times'][1] if len(self.agents) > 1 else 0.0
        info.opp_lap_count = self.obs['lap_counts'][1] if len(self.agents) > 1 else 0
        self.info_pub.publish(info)


if __name__ == '__main__':
    rospy.init_node('gym_bridge')
    gym_bridge = GymBridge()
    gym_bridge.spin()

    # once we're here, we know that competition is completed, so we publish info to API
    print(gym_bridge.info)

    time.sleep(1)
