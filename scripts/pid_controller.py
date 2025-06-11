#!/usr/bin/env python3

class PIDController:
    def __init__(self, kp, ki, kd, min_out=float('-inf'), max_out=float('inf')):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min_out = min_out
        self.max_out = max_out
        self.integral = 0.0
        self.prev_error = 0.0

    def calculate(self, setpoint, measurement, dt):
        error = setpoint - measurement
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        output = max(self.min_out, min(self.max_out, output))
        self.prev_error = error
        return output
