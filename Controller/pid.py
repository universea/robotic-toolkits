import numpy as np
import math

class PID():

    def __init__(self,kp=1,ki=1,kd=0.0):

        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.last_error = 0
        self.integral = 0
        self.output_limit = 5
        self.integral_limit = 5
    
    def set_parameters(self,kp,ki,kd):
        
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def calculate(self,set_value,feed_back):

        error = set_value - feed_back
        differential = error - self.last_error
        output = self.kp * error + self.ki * self.integral + self.kd * differential
        self.integral += error
        if self.integral > self.integral_limit:
            self.integral = self.integral_limit
        if self.integral < - self.integral_limit:
            self.integral = -self.integral_limit
        if output > self.output_limit:
            output = self.output_limit
        if output < - self.output_limit:
            output = -self.output_limit
        self.last_error = error

        return output