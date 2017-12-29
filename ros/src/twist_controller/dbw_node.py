#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
from twist_controller import Controller

'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''

FREQUENCY_IN_HERTZ = 50.0


class DBWNode(object):
    """
    Represents drive by wire controller.
    Receives current and requested steering/velocities, calculates throttle, brake and steering commands and
    publishes them to the vehicle.
    """

    def __init__(self):
        rospy.init_node('dbw_node')

        vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        deceleration_limit_in_mps = rospy.get_param('~decel_limit', -5)
        acceleration_limit_in_mps = rospy.get_param('~accel_limit', 1.)
        wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        min_speed = rospy.get_param('~min_speed', 1.)
        wheel_base = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        max_lat_acceleration = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)

        # Publishers
        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd', SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd', ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd', BrakeCmd, queue_size=1)

        # Subscribers
        rospy.Subscriber("/current_velocity", TwistStamped, self.current_velocity_cb)
        rospy.Subscriber("/twist_cmd", TwistStamped, self.twist_cmd_cb)
        rospy.Subscriber("/vehicle/dbw_enabled", Bool, self.dbw_enabled_cb)

        self.controller = Controller(vehicle_mass=vehicle_mass, fuel_capacity=fuel_capacity,
                                     acceleration_limit=acceleration_limit_in_mps,
                                     deceleration_limit=deceleration_limit_in_mps,
                                     wheel_base=wheel_base, wheel_radius=wheel_radius,
                                     steer_ratio=steer_ratio, max_lat_acceleration=max_lat_acceleration,
                                     max_steer_angle=max_steer_angle, min_speed=min_speed)

        self.current_velocity = None
        self.twist_cmd = None
        self.dbw_enabled = None
        self.last_time = rospy.Time().now()

        self.loop()

    def loop(self):
        rate = rospy.Rate(FREQUENCY_IN_HERTZ)
        while not rospy.is_shutdown():

            cur_time = rospy.Time().now()
            time_span = cur_time - self.last_time
            self.last_time = cur_time

            if self.can_control():
                throttle, brake, steering = self.controller.control(self.twist_cmd,
                                                                    self.current_velocity,
                                                                    time_span.to_sec())
                if self.dbw_enabled:
                    self.publish(throttle, brake, steering)

            rate.sleep()

    def can_control(self):
        """
        This node can operate only when it is turned on, all velocites are available and traffic light classifier
        is ready
        :return: True if publishing to the vehicle is allowed
        """
        return self.dbw_enabled is not None and self.dbw_enabled and \
               self.twist_cmd is not None and self.current_velocity is not None

    def publish(self, throttle, brake, steer):
        """
        Publishes commands to the vehicle
        :param throttle:
        :param brake:
        :param steer:
        """
        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
        self.brake_pub.publish(bcmd)

    def current_velocity_cb(self, velocity):
        """
        Updates current velocity (50Hz)
        :param velocity:
        """
        self.current_velocity = velocity

    def twist_cmd_cb(self, twist):
        """
        Updates requested velocity (50Hz)
        :param twist:
        """
        self.twist_cmd = twist

    def dbw_enabled_cb(self, dbw_enabled):
        """
        Updates knowledge whether this module is turned on (update on demand)
        :param dbw_enabled:
        :return:
        """
        # Reset controllers in case drive by wire was turned on
        if dbw_enabled:
            self.controller.reset()

        self.dbw_enabled = dbw_enabled.data

        rospy.loginfo("Drive by wire was turned {}".format("on" if dbw_enabled else "off"))


if __name__ == '__main__':
    DBWNode()
