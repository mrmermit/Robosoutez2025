from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

# pid.py  (or just paste into your main file above Base)
class PID:
    def __init__(self, kp=0.6, ki=0.0, kd=0.1, integral_limit=10000, output_limit=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.last_error = 0.0
        self.integral_limit = integral_limit
        self.output_limit = output_limit

    def reset(self):
        self.integral = 0.0
        self.last_error = 0.0

    def compute(self, error):
        # P
        p = self.kp * error

        # I (clamped)
        self.integral += error
        if self.integral > self.integral_limit:
            self.integral = self.integral_limit
        elif self.integral < -self.integral_limit:
            self.integral = -self.integral_limit
        i = self.ki * self.integral

        # D
        d = self.kd * (error - self.last_error)
        self.last_error = error

        out = p + i + d
        if self.output_limit is not None:
            if out > self.output_limit:
                out = self.output_limit
            elif out < -self.output_limit:
                out = -self.output_limit
        return out
