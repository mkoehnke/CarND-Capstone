#!/usr/bin/env python

import pid
import yaw_controller
import lowpass
import numpy as np

GAS_DENSITY = 2.858

K_P = 1.5
K_I = 0.001
K_D = 0.

MIN_VELOCITY = 0.1

class Controller(object):
    """
    Acceleration and steering controller.
    Acceleration is controlled via PID controller.
    Steering is calculated using YawController which simply calculates needed angle to keep needed velocity,
    after that steering is smoothed using low pass filter.
    """
    def __init__(self, vehicle_mass, fuel_capacity, acceleration_limit, deceleration_limit,
                 wheel_base, wheel_radius, steer_ratio,
                 max_lat_acceleration, max_steer_angle, brake_deadband, min_speed):

        self.velocity_controller = pid.PID(kp=K_P, ki=K_I, kd=K_D, mn=deceleration_limit, mx=acceleration_limit)

        self.yaw_controller = yaw_controller.YawController(wheel_base=wheel_base,
                                                           steer_ratio=steer_ratio,
                                                           min_speed=min_speed,
                                                           max_lat_accel=max_lat_acceleration,
                                                           max_steer_angle=max_steer_angle)

        self.total_mass = vehicle_mass + (fuel_capacity * GAS_DENSITY)
        self.wheel_radius = wheel_radius
        self.brake_deadband = brake_deadband
        self.decel_limit = deceleration_limit

    def control(self, twist_cmd, current_velocity, time_span):
        """
        Calculates acceleration or deceleration and steering angle given current and requested velocity and time.
        In case acceleration is required it is returned as PID output, returned deceleration is zero.
        In case deceleration is required torque is calculated from PID output, returned acceleration is zero.
        :param twist_cmd: required velocity
        :param current_velocity: current velocity
        :param time_span: time passed from the last call
        :return: acceleration, deceleration, steering angle in wheel frame
        """

        # Calculate velocity error
        velocity_error = twist_cmd.twist.linear.x - current_velocity.twist.linear.x

        # Get velocity correction from PID controller
        acceleration = self.velocity_controller.step(velocity_error, time_span)

        # Calculate steering angle
        steer = self.yaw_controller.get_steering(twist_cmd.twist.linear.x,
                                                 twist_cmd.twist.angular.z,
                                                 current_velocity.twist.linear.x)

        # Try to stop the car completely in case requested velocity is zero and current velocity is small enough
        if np.isclose(twist_cmd.twist.linear.x, 0.) and current_velocity.twist.linear.x < MIN_VELOCITY:
            return 0., self.calc_torque(self.decel_limit), steer
        else:
            if acceleration > 0:
                return acceleration, 0., steer
            else:
                torque = self.calc_torque(-acceleration,)
                return 0., torque, steer

    def calc_torque(self, acceleration):
        return acceleration * self.total_mass * self.wheel_radius

    def reset(self):
        self.velocity_controller.reset()
        self.yaw_filter.reset()
