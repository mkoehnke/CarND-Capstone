#!/usr/bin/env python

import pid
import yaw_controller
import lowpass
import numpy as np

GAS_DENSITY = 2.858

K_P = 1.5
K_I = 0.001
K_D = 0.

YAW_TAU = 0.35
YAW_TS  = 0.02

MIN_VELOCITY = 0.1

class Controller(object):
    """
    Acceleration and steering controller.
    Acceleration is controlled via PID controller.
    Steering is calculated using YawController which simply calculates needed angle to keep needed velocity,
    after that steering is smoothed using low pass filter.
    """
    def __init__(self, vehicle_mass, fuel_capacity, accel_limit, decel_limit,
                 wheel_base, wheel_radius, steer_ratio,
                 max_lat_accel, max_steer_angle, brake_deadband):
        # TODO: Implement

        self.velocity_controller = pid.PID(kp=K_P, ki=K_I, kd=K_D, mn=decel_limit, mx=accel_limit)

        self.yaw_controller = yaw_controller.YawController(wheel_base=wheel_base,
                                                           steer_ratio=steer_ratio,
                                                           min_speed=1.,
                                                           max_lat_accel=max_lat_accel,
                                                           max_steer_angle=max_steer_angle)

        self.yaw_filter = lowpass.LowPassFilter(YAW_TAU, YAW_TS)

        # need to calculate torque
        self.total_mass = vehicle_mass + (fuel_capacity * GAS_DENSITY)
        self.wheel_radius = wheel_radius
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit

    def control(self, twist_cmd, cur_velocity, time_span):
        """
        Calculates acceleration or deceleration and steering angle given current and requested velocity and time.
        In case acceleration is required it is returned as PID output, returned deceleration is zero.
        In case deceleration is required torque is calculated from PID output, returned acceleration is zero.
        :param twist_cmd: required velocity
        :param cur_velocity: current velocity
        :param time_span: time passed from the last call
        :return: acceleration, deceleration, steering angle in wheel frame
        """
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer

        # calculate error
        velocity_error = twist_cmd.twist.linear.x - cur_velocity.twist.linear.x
        # get pid correction
        acceleration = self.velocity_controller.step(velocity_error, time_span)

        # calculate needed steering angle
        steer = self.yaw_controller.get_steering(twist_cmd.twist.linear.x,
                                                 twist_cmd.twist.angular.z,
                                                 cur_velocity.twist.linear.x)
        # smooth steering angle
        steer = self.yaw_filter.filt(steer)

        # in case requested velocity is zero and current velocity is small enough then immobilise the car
        if np.isclose(twist_cmd.twist.linear.x, 0.) and cur_velocity.twist.linear.x < MIN_VELOCITY:
            return 0., self.calc_torque(self.decel_limit), steer
        else:
            # i
            if acceleration > 0:
                return acceleration, 0., steer
            else:
                torque = self.calc_torque(-acceleration,)
                return 0., torque, steer

    def calc_torque(self, acceleration):
        """
        Calculates torque
        :param acceleration:
        :return: torque
        """
        return acceleration * self.total_mass * self.wheel_radius

    def reset(self):
        """
        Resets controllers. This is needed in case drive by wire is turned off.
        """
        self.velocity_controller.reset()
        self.yaw_filter.reset()
