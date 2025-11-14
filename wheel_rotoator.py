from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch


class WheelRotator:
    M_ROTATOR_DEG = 445 
    M_SPEED_DEG = 400
    M_DUTY_LIMIT = 50
    rotator_motor = None
    base = None

    def __init__(self, base_class, motor):
        self.base = base_class
        self.rotator_motor = motor

    def reset_rotator(self):
        M_ROTATOR_DEG = self.M_ROTATOR_DEG
        M_SPEED_DEG = self.M_SPEED_DEG
        M_DUTY_LIMIT = self.M_DUTY_LIMIT

        self.base.base_change_speed(-M_SPEED_DEG * 14 / 56)
        self.rotator_motor.run_until_stalled(-M_SPEED_DEG, then=Stop.HOLD, duty_limit=M_DUTY_LIMIT)
        self.base.base_change_speed(M_SPEED_DEG * 14 / 56)
        
        wait(500)
        self.rotator_motor.reset_angle(angle=0)

        self.base.base_change_speed(M_SPEED_DEG * 14 / 56)
        self.rotator_motor.run_until_stalled(M_SPEED_DEG, then=Stop.HOLD, duty_limit=M_DUTY_LIMIT)
        self.base.base_change_speed(-M_SPEED_DEG * 14 / 56)
        
        wait(500)
        self.base.base_change_speed(-M_SPEED_DEG * 14 / 56)
        self.rotator_motor.run_angle(M_SPEED_DEG, -(self.rotator_motor.angle()-420)/2 , then=Stop.HOLD)
        self.base.base_change_speed(M_SPEED_DEG * 14 / 56)
        self.rotator_motor.reset_angle(angle=0)
        wait(500)

        self.base.base_change_speed(-M_SPEED_DEG * 14 / 56)
        self.rotator_motor.run_angle(M_SPEED_DEG, -M_ROTATOR_DEG , then=Stop.HOLD)
        self.base.base_change_speed(M_SPEED_DEG * 14 / 56)
        
        print("Reset finished...")

    def get_rotator_angle(self):
        return - 3.1415/2 * (self.rotator_motor.angle()/self.M_ROTATOR_DEG)
    
    def rotate_to_angle(self, target_angle):
        M_SPEED_DEG = self.M_SPEED_DEG

        multiplier = 1 if target_angle-self.get_rotator_angle() > 0 else -1
        self.base.base_change_speed(-M_SPEED_DEG * 14 / 56 * multiplier)
        
        while not self.run_target_async(target_angle, M_SPEED_DEG):
            wait(5)
        
        self.base.base_change_speed(M_SPEED_DEG * 14 / 56 * multiplier)
        
    def run_target_async(self, target_angle, speed):
        multiplier = -1 if -self.rotator_motor.angle() < target_angle/3.1415*2 * self.M_ROTATOR_DEG else 1
        
        # print("RUN target: " + str(abs(self.rotator_motor.angle() - target_angle)) + ", motor: " + str(self.rotator_motor.angle()))
        
        if abs(-self.rotator_motor.angle() - target_angle/3.1415*2 * self.M_ROTATOR_DEG) < 7:
            self.rotator_motor.hold()
            return True
        self.rotator_motor.run(speed * multiplier)
        return False


