#!/usr/bin/env python

import rospy
import sys
from geometry_msgs.msg import TwistStamped, PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Bool, Int32, Float32

import math
import copy
from enum import Enum

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 50
SAFETY_BUFFER = 0.5
UNKNOWN = -1
MAX_DISTANCE = sys.maxint
FREQUENCY = 10.0


class State(Enum):
    ACCELERATION = 1
    DECELERATION = 2


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.logwarn("### WaypointUpdater Initialization ....")

        rospy.Subscriber('/current_pose', PoseStamped, self.current_position_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/obstacle_waypoints', PoseStamped, self.obstacle_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        self.final_waypoints = None

        self.lane = None
        self.number_of_waypoints = None

        # Declaration of variables
        self.current_position = self.current_velocity_in_mps = self.next_stopline_waypoint = self.current_state = self.state_changed = None

        self.acceleration_limit_in_mps = rospy.get_param('~accel_limit', 1.)
        self.deceleration_limit_max_in_mps = -rospy.get_param('~decel_limit', -5.)
        self.deceleration_limit_min_in_mps = min(1.0, -rospy.get_param('~decel_limit', -5.) / 2.)
        self.max_velocity_in_mps = rospy.get_param('/waypoint_loader/velocity') / 3.6

        self.initVariables()
        self.loop()

    def initVariables(self):
        self.current_position = None
        self.current_velocity_in_mps = None
        self.next_stopline_waypoint = UNKNOWN
        self.current_state = State.ACCELERATION
        self.state_changed = True
        self.final_waypoints = None

    def loop(self):
        rate = rospy.Rate(FREQUENCY)
        while not rospy.is_shutdown():
            self.publish_final_wps()
            rate.sleep()

    def is_waypoint_behind(self, waypoint_index):
        dx = self.current_position.x - self.lane.waypoints[waypoint_index].pose.pose.position.x
        dy = self.current_position.y - self.lane.waypoints[waypoint_index].pose.pose.position.y
        nx = self.lane.waypoints[(waypoint_index + 1) % self.number_of_waypoints].pose.pose.position.x - \
             self.lane.waypoints[waypoint_index].pose.pose.position.x
        ny = self.lane.waypoints[(waypoint_index + 1) % self.number_of_waypoints].pose.pose.position.y - \
             self.lane.waypoints[waypoint_index].pose.pose.position.y
        dp = dx * nx + dy * ny
        return dp > 0.0

    def get_closest_waypoint_index(self):
        minimal_distance = MAX_DISTANCE
        waypoints = self.lane.waypoints
        waypoint_index = UNKNOWN

        for i in range(self.number_of_waypoints):
            distance = self.distance_of_positions(self.current_position, waypoints[i].pose.pose.position)
            if distance < minimal_distance:
                minimal_distance = distance
                waypoint_index = i

        if self.is_waypoint_behind(waypoint_index):
            waypoint_index = (waypoint_index + 1) % self.number_of_waypoints

        return waypoint_index

    def start_acceleration(self, lane, current_waypoint_index):
        acceleration = self.acceleration_limit_in_mps
        current_velocity = self.current_velocity_in_mps
        target_velocity = self.current_velocity_in_mps
        i = 0
        while target_velocity < self.max_velocity_in_mps or i < LOOKAHEAD_WPS:
            start_position = self.current_position
            end_position = self.lane.waypoints[
                (current_waypoint_index + i) % self.number_of_waypoints].pose.pose.position
            distance = self.distance_of_positions(start_position, end_position)
            target_velocity = math.sqrt(current_velocity ** 2.0 + 2.0 * acceleration * distance)
            if target_velocity > self.max_velocity_in_mps:
                target_velocity = self.max_velocity_in_mps
            current_waypoint = copy.deepcopy(
                self.lane.waypoints[(current_waypoint_index + i) % self.number_of_waypoints])
            current_waypoint.twist.twist.linear.x = target_velocity
            lane.waypoints.append(current_waypoint)
            i += 1

    def start_deceleration(self, lane, current_waypoint_index):
        current_velocity = self.current_velocity_in_mps
        target_velocity = self.current_velocity_in_mps
        distance = self.distance_of_positions(self.current_position, self.lane.waypoints[
            self.next_stopline_waypoint].pose.pose.position) - SAFETY_BUFFER
        acceleration = current_velocity ** 2.0 / (2.0 * distance)

        i = 0
        while target_velocity > 0.0 or i < LOOKAHEAD_WPS:
            start_position = self.current_position
            end_position = self.lane.waypoints[
                (current_waypoint_index + i) % self.number_of_waypoints].pose.pose.position
            distance = self.distance_of_positions(start_position, end_position)
            target_velocity_exp = current_velocity ** 2.0 - 2.0 * acceleration * distance
            if target_velocity_exp <= 0:
                target_velocity = 0
            else:
                target_velocity = math.sqrt(target_velocity_exp)
            current_waypoint = copy.deepcopy(
                self.lane.waypoints[(current_waypoint_index + i) % self.number_of_waypoints])
            current_waypoint.twist.twist.linear.x = target_velocity
            lane.waypoints.append(current_waypoint)
            i += 1

    def continue_with_current_state(self, lane, waypoint_index, cv):
        j = 0
        while j < len(self.final_waypoints):
            if self.final_waypoints[j].pose.pose.position == self.lane.waypoints[waypoint_index].pose.pose.position:
                break
            j += 1
        for i in range(j, len(self.final_waypoints)):
            current_waypoint = copy.deepcopy(self.lane.waypoints[(waypoint_index + i - j) % self.number_of_waypoints])
            current_waypoint.twist.twist.linear.x = self.final_waypoints[i].twist.twist.linear.x
            lane.waypoints.append(current_waypoint)
        for i in range(len(lane.waypoints), LOOKAHEAD_WPS):
            current_waypoint = copy.deepcopy(self.lane.waypoints[(waypoint_index + i) % self.number_of_waypoints])
            current_waypoint.twist.twist.linear.x = cv
            lane.waypoints.append(current_waypoint)

    def publish_final_wps(self):
        if self.lane is None or self.current_position is None or self.current_velocity_in_mps is None:
            return

        waypoint_index = self.get_closest_waypoint_index()
        lane = Lane()
        lane.header.stamp = rospy.Time.now()

        # Handle state changes
        if self.current_state == State.ACCELERATION:
            if self.next_stopline_waypoint != UNKNOWN:
                start_position = self.current_position
                end_position = self.lane.waypoints[self.next_stopline_waypoint].pose.pose.position
                brake_distance = self.distance_of_positions(start_position, end_position) - SAFETY_BUFFER

                min_brake_distance = 0.5 * self.current_velocity_in_mps ** 2 / self.deceleration_limit_max_in_mps
                max_brake_distance = 0.5 * self.current_velocity_in_mps ** 2 / self.deceleration_limit_min_in_mps
                if max_brake_distance >= brake_distance >= min_brake_distance:
                    self.current_state = State.DECELERATION
                    self.state_changed = True

        elif self.current_state == State.DECELERATION:
            if self.next_stopline_waypoint == UNKNOWN:
                self.current_state = State.ACCELERATION
                self.state_changed = True

        else:
            rospy.logerr("WaypointUpdater: A state doesn't exist.")

        # Handle states
        if self.current_state == State.ACCELERATION and self.state_changed:
            self.start_acceleration(lane, waypoint_index)
        elif self.current_state == State.ACCELERATION and not self.state_changed:
            self.continue_with_current_state(lane, waypoint_index, self.max_velocity_in_mps)
        elif self.current_state == State.DECELERATION and self.state_changed:
            self.start_deceleration(lane, waypoint_index)
        elif self.current_state == State.DECELERATION and not self.state_changed:
            self.continue_with_current_state(lane, waypoint_index, 0)
        else:
            rospy.logerr("WaypointUpdater: A state doesn't exist.")

        self.state_changed = False
        self.final_waypoints = copy.deepcopy(lane.waypoints)
        self.final_waypoints_pub.publish(lane)

    # Helper

    def distance_of_positions(self, pos1, pos2):
        return math.sqrt((pos1.x - pos2.x) ** 2 + (pos1.y - pos2.y) ** 2 + (pos1.z - pos2.z) ** 2)

    def distance_of_waypoints(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)
        for i in range(wp1, wp2 + 1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    # Callbacks

    def current_position_cb(self, msg):
        self.current_position = msg.pose.position

    def waypoints_cb(self, waypoints):
        self.lane = waypoints
        self.number_of_waypoints = len(self.lane.waypoints)

    def traffic_cb(self, msg):
        self.next_stopline_waypoint = msg.data
        if self.next_stopline_waypoint != UNKNOWN and self.number_of_waypoints is not None:
            self.next_stopline_waypoint = (self.next_stopline_waypoint - 5 + self.number_of_waypoints) % self.number_of_waypoints

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        # rospy.loginfo("Obstacle received: %s", msg.pose.position)
        pass

    def current_velocity_cb(self, msg):
        self.current_velocity_in_mps = msg.twist.linear.x


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
