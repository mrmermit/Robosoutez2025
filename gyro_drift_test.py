from gyro import SpikeGyro
from pybricks.hubs import PrimeHub
from pybricks.tools import wait
hub = PrimeHub()
gyro = SpikeGyro(hub=hub)


print("SpikeGyro test — keep the hub still for calibration")


ok = False
try:
    ok = gyro.calibrate(timeout_ms=10000)
except Exception as e:
    print("calibrate() raised:", e)

while len(hub.buttons.pressed()) == 0:
    print(gyro.get_rotation())
    wait(100) 
print(ok)