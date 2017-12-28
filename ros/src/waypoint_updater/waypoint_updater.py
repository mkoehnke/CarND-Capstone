#!/usr/bin/env python

import rospy
import sys
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint, TrafficLightArray, TrafficLight
from std_msgs.msg import Int32

import yaml
import math

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

LOOKAHEAD_WPS = 50 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        # Subscribers
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size=1)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb, queue_size=1)
        rospy.Subscriber('/obstacle_waypoint', Int32, self.obstacle_cb, queue_size=1)
        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb, queue_size=1)
        rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.tla_cb, queue_size=1)

        # Publisher
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.closest_wp = -1

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)
        self.stop_line_positions = self.config['stop_line_positions']
        self.lights = []
        self.traffic_light_behaviour = False
        self.light_state = TrafficLight.UNKNOWN
        self.light_change = False
        self.light_wp = -1

        # Main loop
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.get_waypoints()
            rate.sleep()

    def copy_waypoint(self, waypoint):
        p = Waypoint()
        p.pose.pose.position.x = waypoint.pose.pose.position.x
        p.pose.pose.position.y = waypoint.pose.pose.position.y
        p.pose.pose.position.z = waypoint.pose.pose.position.z
        p.pose.pose.orientation.x = waypoint.pose.pose.orientation.x
        p.pose.pose.orientation.y = waypoint.pose.pose.orientation.y
        p.pose.pose.orientation.z = waypoint.pose.pose.orientation.z
        p.pose.pose.orientation.w = waypoint.pose.pose.orientation.w
        p.twist.twist.linear.x = waypoint.twist.twist.linear.x
        p.twist.twist.linear.y = waypoint.twist.twist.linear.x
        p.twist.twist.linear.z = waypoint.twist.twist.linear.x
        p.twist.twist.angular.x = waypoint.twist.twist.angular.x
        p.twist.twist.angular.y = waypoint.twist.twist.angular.x
        p.twist.twist.angular.z = waypoint.twist.twist.angular.x
        return p

    def get_waypoints(self):
        self.final_waypoints = []
        if hasattr(self, 'base_waypoints') and hasattr(self, 'current_pose') and hasattr(self, 'current_velocity'):
            #wp_list = self.base_waypoints.waypoints
            #wp_len = len(wp_list)
            #curr_pose = self.current_pose.pose.position

            # Find closest waypoint and distance to current position
            closest_waypoint, closest_distance = \
                self.get_closest_waypoint_and_distance(self.current_pose.pose.position, self.base_waypoints.waypoints)

            # Update final waypoints
            self.update_final_waypoints(closest_waypoint, self.base_waypoints.waypoints)


            # Handle traffic lights

            '''
            
            light = None
            # Check traffic light status
            if hasattr(self, 'traffic_waypoint'):
                if self.traffic_waypoint > wp_len:
                    if self.light_state != TrafficLight.UNKNOWN:
                        self.light_change = True
                    self.light_state = TrafficLight.UNKNOWN
                    self.light_wp = -1
                elif self.traffic_waypoint > 0:
                    if self.light_state != TrafficLight.RED:
                        self.light_change = True
                    self.light_state = TrafficLight.RED
                    self.light_wp = self.traffic_waypoint
                else:
                    if self.light_state != TrafficLight.GREEN:
                        self.light_change = True
                    self.light_state = TrafficLight.GREEN
                    self.light_wp = -self.traffic_waypoint

            if self.light_state == TrafficLight.RED:
                rospy.logwarn('SAN: FOUND RED LIGHT')
                # Find traffic light index
                car_pose = self.final_waypoints[0].pose.pose.position
                closest_wp = -1
                closest_dist = 999999
                for i in range(len(self.lights)):
                    light_pose = self.lights[i].pose.pose.position
                    distance = math.sqrt((car_pose.x - light_pose.x) ** 2 + (car_pose.y - light_pose.y) ** 2)
                    if distance < closest_dist:
                        closest_dist = distance
                        closest_wp = i

                light = self.lights[closest_wp]
                stop_light_pose = self.stop_line_positions[closest_wp]

                # Find waypoint closes to the traffic light
                closest_wp = -1
                closest_dist = 999999
                for i in range(len(self.final_waypoints)):
                    wp_car = self.final_waypoints[i].pose.pose.position
                    wp_light = light.pose.pose.position
                    distance = math.sqrt(
                        (wp_car.x - wp_light.x) ** 2 + (wp_car.y - wp_light.y) ** 2 + (wp_car.z - wp_light.z) ** 2)
                    if distance < closest_dist:
                        closest_dist = distance
                        closest_wp = i

                braking_tolerance = 10
                distance_car_to_light = self.distance(self.final_waypoints, 0, closest_wp)

                light_pose = light.pose.pose.position
                distance_stop_to_light = math.sqrt(
                    (light_pose.x - stop_light_pose[0]) ** 2 + (light_pose.y - stop_light_pose[1]) ** 2)

                # check if vehicle crossed the stop line
                crossed_light = False
                if distance_car_to_light < distance_stop_to_light:
                    crossed_light = True

                max_permissible_deceleration = 1.0
                min_safety_deceleration = 1.0
                min_braking_distance = (self.current_velocity ** 2) / (2 * max_permissible_deceleration)
                min_braking_distance += distance_stop_to_light

                braking_waypoints = 0
                for i in range(len(self.final_waypoints)):
                    if self.distance(self.final_waypoints, 0, i) > min_braking_distance:
                        braking_waypoints = i
                        break

                if min_braking_distance < 150 and crossed_light is False:
                    braking_clearance = 5
                    braking_start_wp = max(0, closest_wp - braking_waypoints - braking_clearance)
                    if braking_start_wp <= 2 * braking_clearance:
                        deceleration = 100.0
                    else:
                        deceleration = max(min_safety_deceleration, self.current_velocity / (braking_waypoints * 1))

                    for i in range(braking_start_wp, LOOKAHEAD_WPS):
                        dec_step = i - braking_start_wp + 1
                        self.final_waypoints[i].twist.twist.linear.x = self.current_velocity - (deceleration * dec_step)
                        self.final_waypoints[i].twist.twist.linear.x = max(0.00, self.final_waypoints[
                            i].twist.twist.linear.x - 1.0)

            else:
                rospy.logwarn('SAN: SAFE TO DRIVE')
            '''

            self.publish_final_waypoints()

    def publish_final_waypoints(self):
        lane = Lane()
        lane.header.frame_id = '/world'
        lane.header.stamp = rospy.Time.now()
        lane.waypoints = self.final_waypoints
        self.final_waypoints_pub.publish(lane)

    def velocity_cb(self, msg):
        self.current_velocity = msg.twist.linear.x
        pass

    def pose_cb(self, msg):
        self.current_pose = msg
        pass

    def waypoints_cb(self, waypoints):
        self.base_waypoints = waypoints
        pass

    def traffic_cb(self, msg):
        self.traffic_waypoint = msg.data

    def obstacle_cb(self, msg):
        self.obstacle_waypoint = msg.data
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def get_closest_waypoint_and_distance(self, current_position, waypoints):
        closest_waypoint = -1
        closest_distance = sys.maxint
        for i in range(len(waypoints)):
            wp = waypoints[i].pose.pose.position
            distance = math.sqrt((wp.x - current_position.x) ** 2 + (wp.y - current_position.y) ** 2 + (wp.z - current_position.z) ** 2)
            if distance < closest_distance:
                closest_distance = distance
                closest_waypoint = i
        return closest_waypoint, closest_distance

    def update_final_waypoints(self, closest_waypoint, waypoints):
        for i in range(closest_waypoint, closest_waypoint + LOOKAHEAD_WPS):
            wp_index = i % len(waypoints)
            wp = self.copy_waypoint(waypoints[wp_index])
            self.final_waypoints.append(wp)

    def tla_cb(self, msg):
        self.lights = msg.lights


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')