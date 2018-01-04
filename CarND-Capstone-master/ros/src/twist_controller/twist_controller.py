# inspiration from Dataspeed:
# https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/c3ac292ddc167e170f81030cd97b2a74bc5c4f84/dbw_mkz_twist_controller/src/TwistControllerNode.cpp?at=default&fileviewer=file-view-default#TwistControllerNode.cpp-137

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

from pid import PID # for acceleration
from lowpass import LowPassFilter # for acceleration
from yaw_controller import YawController # for steering

class Controller(object):
    def __init__(self, wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle,
                 vehicle_mass, fuel_capacity, brake_deadband, decel_limit, accel_limit, wheel_radius):
        # TODO: Implement

        # for velocity error estimation
        self.min_speed = min_speed
        
        # pid controller
        #values from:https://github.com/josemiguelpulido/udacity-carnd/blob/master/CarND-PID-Control-Project/src/main.cpp
        kp = 0.1
        ki = 0.00001
        kd = 0.8
        self.vel_pid = PID(kp, ki, kd, decel_limit, accel_limit)

        # low pass filter
        tau = 0.1
        ts = 0.1
        self.lowpass = LowPassFilter(tau, ts)

        # for throttle computation
        self.vehicle_mass = vehicle_mass + fuel_capacity * GAS_DENSITY # assume full tank, as we do not have info about tank levels
        self.wheel_radius = wheel_radius
        self.brake_deadband = brake_deadband

        # for steering computation
        self.yaw = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)


    def control(self, linear_velocity, angular_velocity, current_velocity, sample_time):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer

        steer = self.yaw.get_steering(linear_velocity.x, angular_velocity.z, current_velocity.x)
        
        vel_error = linear_velocity.x - current_velocity.x
        if linear_velocity.x < self.min_speed:
            self.vel_pid.reset()
        
        throttle = self.vel_pid.step(self.lowpass(vel_error), sample_time)

        if throttle < self.brake_deadband:
            brake = -throttle * self.vehicle_mass * self.wheel_radius;
        else:
            brake = 0

        return throttle, brake, steer
