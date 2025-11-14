from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

class Trolley:
    rotator_motor = None

    MAX_SPEED = 700

    def __init__(self, motor):
        self.rotator_motor = motor

    def reset_zero_trolley(self):
        self.rotator_motor.reset_angle(angle=0)
    
    def reset_to_zero(self):
        TARGET_MOTOR_ANGLE = 0
        return self.run_target_async(TARGET_MOTOR_ANGLE)
    
    def open_cart(self):
        TARGET_MOTOR_ANGLE = 36/12 * 110
        return self.run_target_async(TARGET_MOTOR_ANGLE)
    
    def empty_cart(self):
        TARGET_MOTOR_ANGLE = 36/12 * 200
        return self.run_target_async(TARGET_MOTOR_ANGLE)    
    def close_cart(self):
        TARGET_MOTOR_ANGLE = -36/12 * 70
        return self.run_target_async(TARGET_MOTOR_ANGLE)

    def run_target_async(self, target_angle):
        multiplier = 1 if self.rotator_motor.angle() < target_angle else -1
        
        # print("RUN target: " + str(abs(self.rotator_motor.angle() - target_angle)) + ", motor: " + str(self.rotator_motor.angle()))
        
        if abs(self.rotator_motor.angle() - target_angle) < 8:
            self.rotator_motor.hold()
            return True
        self.rotator_motor.run(self.MAX_SPEED * multiplier)
        return False
