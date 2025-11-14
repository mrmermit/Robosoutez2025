from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch
from wheel_rotoator import WheelRotator
from gyro import SpikeGyro
from pid import PID

class Base:
    hub = None
    front_motor = None
    back_motor = None
    wheel_rotator = None
    gyro = None

    WHEEL_DIEAMETER = 56

    MAX_SPEED = 520
    front_motor_speed = 0
    back_motor_speed = 0

    pid_drive_traveled = 0
    pid_drive_loop_rotation_front = -1
    pid_drive_loop_rotation_back = -1
    pid_drive_front_speed = 0
    pid_drive_back_speed = 0

    drive_pid = None

    def __init__(self, hub, front_motor, back_motor, rotator_motor):
        self.back_motor = back_motor
        self.front_motor = front_motor
        self.hub = hub

        self.wheel_rotator = WheelRotator(self, rotator_motor)
        self.drive_pid = PID(kp=90, ki=0.9, kd=0, output_limit=400)
        self.gyro = SpikeGyro(hub=hub)
        ok = False
        try:
            ok = gyro.calibrate(timeout_ms=10000)
        except Exception as e:
            print("calibrate() raised:", e)
    
    def get_wheel_rotator_class(self):
        return self.wheel_rotator
    
    def front_change_speed(self, adjustment):
        self.front_motor_speed += adjustment
        if(abs(self.front_motor_speed) < 5):
            self.front_motor.hold()
            return
        self.front_motor.run(self.front_motor_speed)
    
    def back_change_speed(self, adjustment):
        self.back_motor_speed += adjustment
        if(abs(self.back_motor_speed) < 5):
            self.back_motor.hold()
            return
        self.back_motor.run(self.back_motor_speed)

    def base_change_speed(self, adjustment):
        self.front_change_speed(adjustment)
        self.back_change_speed(adjustment)

    def drive_distance_PID(self, distance_mm, target_speed, gyro_target_heading):
        current_heading = self.gyro.get_rotation()
        
        correction = self.drive_pid.compute(current_heading - gyro_target_heading)
        print("Corection: " + str(correction) + " ," + str(current_heading))   

        self.pid_drive_front_speed = max(min(self.MAX_SPEED, target_speed - correction), -self.MAX_SPEED) 
        self.pid_drive_back_speed = max(min(self.MAX_SPEED, target_speed + correction), -self.MAX_SPEED)
        
        self.front_motor_speed += self.pid_drive_front_speed
        self.back_motor_speed += self.pid_drive_back_speed

        if self.pid_drive_loop_rotation_back == -1 or self.pid_drive_loop_rotation_front == -1:
            self.front_motor.reset_angle(0)
            self.back_motor.reset_angle(0)
            
            self.pid_drive_loop_rotation_front = self.front_motor.angle()
            self.pid_drive_loop_rotation_back = self.back_motor.angle()
            self.pid_drive_traveled = 0
        else:
            front_speed_percentage = 0 if self.front_motor_speed == 0 else self.pid_drive_front_speed/self.front_motor_speed
            back_speed_percentage = 0 if self.back_motor_speed == 0 else self.pid_drive_back_speed/self.back_motor_speed

            self.pid_drive_traveled += (abs(self.front_motor.angle()-self.pid_drive_loop_rotation_front)*front_speed_percentage+abs(self.back_motor.angle()-self.pid_drive_loop_rotation_back)*back_speed_percentage)/720 * self.WHEEL_DIEAMETER * 3.141592
            
            print(str((self.front_motor.angle()-self.pid_drive_loop_rotation_front)) + ", " + str(self.pid_drive_back_speed/self.back_motor_speed)+ ", " + str(((self.front_motor.angle()-self.pid_drive_loop_rotation_front)*self.pid_drive_back_speed/self.back_motor_speed-(self.back_motor.angle()-self.pid_drive_loop_rotation_back)*self.pid_drive_front_speed/self.front_motor_speed)/2))
            
            self.pid_drive_loop_rotation_front = self.front_motor.angle()
            self.pid_drive_loop_rotation_back = self.back_motor.angle()
            print("traveled: " + str(self.pid_drive_traveled) + " " + str(self.front_motor.angle()) + " " + str(self.back_motor.angle()))
            
        
        if abs(self.pid_drive_traveled) >= abs(distance_mm-10):
            self.front_motor.hold()
            self.back_motor.hold()
            self.pid_drive_loop_rotation_front = -1
            self.pid_drive_loop_rotation_back = -1
            return {"reached": True, "distance": self.pid_drive_traveled}
        
        
        self.front_motor.run(-self.front_motor_speed)
        self.back_motor.run(-self.back_motor_speed)

        self.front_motor_speed -= self.pid_drive_front_speed
        self.back_motor_speed -= self.pid_drive_back_speed
        return {"reached": False, "distance": self.pid_drive_traveled}

        

        