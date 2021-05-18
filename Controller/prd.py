import numpy as np
import math

class PRD():

    def __init__(self,ts,kp,kr,wi,kd,out_limit=150):

        self.ts=ts 
        self.kp=kp
        self.kr=kr
        self.wi=wi
        self.kd=kd

        self.output_of_backward_integrator=0
        self.output_of_feedback=0
        self.output_of_forward_integrator=0
        self.last_input_of_forward_integrator=0

        self.out_limit = out_limit
        self.error_last=0.0

    def set_prd(self,kp,kr,wi,kd):

        self.kp=kp
        self.kr=kr
        self.wi=wi
        self.kd=kd

    def calculate(self,ref,feedback,wg):

        error = ref-feedback
        input_of_forward_integrator=2 * self.wi * self.kr * error - self.output_of_feedback
        # Forward integrator
        self.output_of_forward_integrator += self.ts *  input_of_forward_integrator
        # Backward integrator
        self.output_of_backward_integrator += self.ts * self.output_of_forward_integrator * wg * wg
        self.output_of_feedback = self.output_of_backward_integrator + 2 * self.wi * self.output_of_forward_integrator
        # Differential
        differential = (error-self.error_last)*self.ts
        out = self.output_of_forward_integrator + self.kp * error + self.kd*differential
        self.error_last=error

        if out > self.out_limit:
            out=self.out_limit
        if out < -self.out_limit:
            out=-self.out_limit

        return out