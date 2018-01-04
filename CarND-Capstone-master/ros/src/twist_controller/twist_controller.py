
GAS_DENSITY = 2.858
ONE_MPH = 0.44704

from pid import PID # for acceleration
from lowpass import LowPassFilter # for acceleration
from yaw_controller import YawController # for steering

class Controller(object):
    def __init__(self, wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle,
                 vehicle_mass, fuel_capacity, brake_deadband, decel_limit, accel_limit, wheel_radius):
        # TODO: Implement

        # for brake computation
        self.min_speed = min_speed
        
        # pid controller for velocity
        #values from:https://github.com/josemiguelpulido/udacity-carnd/blob/master/CarND-PID-Control-Project/src/main.cpp
        kp = 0.1
        ki = 0.00001
        kd = 0.8
        
        self.vel_pid = PID(kp, ki, kd)


        # low pass filter
        tau =
        ts = 
        self.lowpass = LowPassFilter(tau, ts)

        # for throttle computation
        self.vehicle_mass = vehicle_mass + fuel_capacity * GAS_DENSITY # assume full tank, as we do not have info about tank levels
        self.wheel_radius = wheel_radius
        self.acc_pid = PID(kp, ki, kd)

        # for steering computation
        self.yaw = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)


    def control(self, linear_velocity, angular_velocity, current_velocity, sample_time, <dbw status>):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer

        steer = self.yaw.get_steering(linear_velocity.x, angular_velocity.z, current_velocity.x)
        
        vel_error = linear_velocity.x - current_velocity.x
        if linear_velocity.x < self.min_speed:
            vel_pid.reset()
        
        throttle  = self.pid.step(self.lowpass(error), sample_time)


        accel_cmd = min(throttle_cmd, -530 / self.vehicle_mass / self.wheel_radius)
        return 1., 0., steer
