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
        #tau = 0.1
        #ts = 0.1
        a = 0.25 # weight of last sample
        self.lowpass = LowPassFilter(a)
        self.lowpass_lin_vel = LowPassFilter(a)
        self.lowpass_ang_vel = LowPassFilter(a)
        self.lowpass_cur_vel = LowPassFilter(a)

        # for throttle computation

        # assume full tank, as we do not have info about tank levels
        # 1 gallon / 0.0038 cubic meter
        self.vehicle_mass = vehicle_mass + (fuel_capacity * GAS_DENSITY / 0.0038)
        
        self.wheel_radius = wheel_radius
        self.brake_deadband = brake_deadband

        # for steering computation
        self.yaw = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)


    def control(self, linear_velocity, angular_velocity, current_velocity, sample_time, dbw_enabled):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer

        if not dbw_enabled:
            return 1., 0., 0.

        # smooth out raw signals because they can be noisy. Needed?
        # (and student recommends using abs, to account for occasional errors)
        lin_velocity = self.lowpass_lin_vel.filt(abs(linear_velocity.x)) 
        ang_velocity = self.lowpass_ang_vel.filt(angular_velocity.z)
        cur_velocity = self.lowpass_cur_vel.filt(abs(current_velocity.x))

        # steering
        steer = self.yaw.get_steering(lin_velocity, ang_velocity, cur_velocity)

        # throttle
        vel_diff = lin_velocity - cur_velocity
        if lin_velocity < self.min_speed:
            self.vel_pid.reset()
        
        accel = self.vel_pid.step(self.lowpass.filt(vel_diff), sample_time)
        throttle = max(0,min(1,accel)) # throttle values in [0,1]

        # brake
        if accel < -self.brake_deadband:
            brake = -accel * self.vehicle_mass * self.wheel_radius;
        else:
            brake = 0

        return throttle, brake, steer
