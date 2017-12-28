import rospy
import math
from pid import PID
from yaw_controller import YawController

GAS_DENSITY = 2.858
# ONE_MPH = 0.44704

VEL_K_P = 11.2
VEL_K_I = 0.05
VEL_K_D = 0.3

STEER_K_P = 0.8
STEER_K_I = 0.05
STEER_K_D = 0.2

MAX_V_MPS = 10.7  # Maximum speed in meters_per_second
BRAKE_TORQUE_SCALE = 100000
YAW_SCALE = 8.2


class Controller(object):
    def __init__(self, **kwargs):
        # TODO: Implement
        max_lat_acc = kwargs['max_lat_acc']
        max_steer_angle = kwargs['max_steer_angle']
        steer_ratio = kwargs['steer_ratio']
        wheel_base = kwargs['wheel_base']

        self.accel_limit = kwargs['accel_limit']
        self.brake_deadband = kwargs['brake_deadband']
        self.decel_limit = kwargs['decel_limit']

        self.vehicle_mass = kwargs['vehicle_mass']
        self.fuel_capacity = kwargs['fuel_capacity']
        self.wheel_radius = kwargs['wheel_radius']
        self.total_mass = self.vehicle_mass + (self.fuel_capacity * GAS_DENSITY)

        # Create Variable for the last update time
        self.last_update_time = None

        self.velocity_pid = PID(VEL_K_P, VEL_K_I, VEL_K_D, -self.accel_limit, self.accel_limit)
        self.steering_pid = PID(STEER_K_P, STEER_K_I, STEER_K_D, -max_steer_angle / 2, max_steer_angle / 2)

        # Create a steering controller
        self.steering_yaw = YawController(wheel_base=wheel_base, steer_ratio=steer_ratio, min_speed=0.0,
                                          max_lat_accel=max_lat_acc, max_steer_angle=max_steer_angle)

        # scale of max acceleration to deceleration
        self.brake_scale = self.decel_limit / (-self.accel_limit)

        pass

    def control(self, **kwargs):
        # Get the variables from the arguments
        twist_linear = kwargs['twist_command'].twist.linear
        twist_angular = kwargs['twist_command'].twist.angular
        cv_linear = kwargs['current_velocity'].twist.linear
        steer_feedback = kwargs['steer_feedback']

        # Set the present and target values
        target_linear_velocity = twist_linear.x
        present_linear_velocity = cv_linear.x
        target_angular_velocity = twist_angular.z
        #rospy.logwarn('SAN: Present vel: %s', present_linear_velocity)
        #rospy.logwarn('SAN: Target vel: %s', target_linear_velocity)

        if math.fabs(target_linear_velocity) < 0.1:
            target_linear_velocity = 0.0
        if math.fabs(target_angular_velocity) < 0.001:
            target_angular_velocity = 0.0


        if self.last_update_time is not None:
            # Get current time
            time = rospy.get_time()

            # Compute timestep between updates and save
            # for next iteration
            dt = time - self.last_update_time
            self.last_update_time = time

            # if vehicle is trying to stop we want to reset the integral component
            # of the PID controllers so as to not oscillate around zero
            if present_linear_velocity < 1.0:
                self.velocity_pid.reset()
                self.steering_pid.reset()

            # PID class returns output for throttle and
            # brake axes as a joint forward_backward axis

            velocity_error = target_linear_velocity - present_linear_velocity
            # Convert to fraction of v_max so pid output is directly
            # usable as input to throttle_cmd
            # velocity_error = velocity_error/MAX_V_MPS

            forward_axis = self.velocity_pid.step(velocity_error, dt)
            reverse_axis = -forward_axis * (self.decel_limit / (-self.accel_limit))

            # if forward axis is positive only then give any throttle
            throttle = min(max(0.0, forward_axis), 1.0)
            # rospy.logwarn('SAN: forward_axis %s', forward_axis)
            # rospy.logwarn('SAN: reverse_axis %s', reverse_axis)
            # rospy.logwarn('SAN: throttle %s', throttle)
            # Only apply brakes if the reverse axis value is large enough to supersede the deadband
            brake = max(0.0, reverse_axis - self.brake_deadband)

            # Convert brake to Torque value since that is what the publisher expects
            #brake *= 100
            torque = self.calc_torque(brake)

            # get the steering value from the yaw controller
            steering = self.steering_yaw.get_steering(target_linear_velocity, target_angular_velocity, present_linear_velocity)
            steering_error = steering - steer_feedback

            # update using steering pid loop
            steering = self.steering_pid.step(steering_error, dt)

            return throttle, torque, steering
        else:
            # Update the last_update time and send Zeroes tuple
            self.last_update_time = rospy.get_time()
            return 0., 0., 0.


    def calc_torque(self, acceleration):
        """
        Calculates torque
        :param acceleration:
        :return: torque
        """
        return acceleration * self.total_mass * self.wheel_radius
