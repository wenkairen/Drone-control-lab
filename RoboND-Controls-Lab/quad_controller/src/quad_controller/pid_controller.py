# Copyright (C) 2017 Udacity Inc.
# All Rights Reserved.

# Author: Brandon Kinman


class PIDController:
    def __init__(self, kp = 0.0, ki = 0.0, kd = 0.0, max_windup = 10):
        #TODO
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)

        self.last_timestamp = 0.0
        self.target = 0.
        self.max_windup = float(max_windup)
        self.error_sum = 0.
        self.last_error = 0.

        
    def reset(self):
        self.kp = 0.
        self.ki = 0.
        self.kd = 0.

    def setTarget(self, target):
        #TODO
        self.target = target

    def setKP(self, kp):
        #TODO
        self.kp = kp

    def setKI(self, ki):
        #TODO
        self.ki = ki

    def setKD(self, kd):
        #TODO
        self.kd = kd

    def setMaxWindup(self, max_windup):
        #TODO
        self.max_windup = int(max_windup)

    def update(self, measured_value, timestamp):
        #TODO
        delta_time = timestamp - self.last_timestamp

        if delta_time == 0:
            return 0

        error = self.target - measured_value

        self.last_timestamp = timestamp

        self.error_sum += error * delta_time

        delta_error = error - self.last_error

        self.last_error = error

        if self.error_sum > self.max_windup:
            self.error_sum = self.max_windup
        elif self.error_sum < -self.max_windup:
            self.error_sum = -self.max_windup

        p = self.kp * error

        i = self.ki * self.error_sum

        d = self.kd * delta_error

        u = p + i + d 
        
        return u        





