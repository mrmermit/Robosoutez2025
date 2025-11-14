from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch
from wheel_rotoator import WheelRotator
from base import Base
from trolley import Trolley
import umath
hub = PrimeHub()


base = Base(hub, Motor(Port.C), Motor(Port.B), Motor(Port.F))


wheel_rotoator = base.get_wheel_rotator_class()
trolley = Trolley(Motor(Port.A))

wheel_rotoator.reset_rotator()

wait(1000)

# while not base.drive_distance_PID(600, 400, 0)["reached"]:
#     wait(100)

print("reached...")
print(wheel_rotoator.get_rotator_angle())
wait(500)

# trolley.reset_zero_trolley()
# wait(2000)
# while not trolley.open_cart():
#     wait(10)

# wait(2000)
# while not trolley.close_cart():
#     wait(10)

# wait(2000)
# while not trolley.empty_cart():
#     wait(10)
# wait(2000)
# while not trolley.reset_to_zero():
#     wait(50)

wheel_rotoator.rotate_to_angle(umath.pi/8)
print(wheel_rotoator.get_rotator_angle()-wheel_rotoator.get_rotator_angle())
print(umath.pi/8)
wait(1000)
wheel_rotoator.rotate_to_angle(umath.pi/2)
print(wheel_rotoator.get_rotator_angle())
print(umath.pi/2-wheel_rotoator.get_rotator_angle())
