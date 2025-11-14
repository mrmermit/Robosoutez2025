from pybricks.hubs import PrimeHub
from pybricks.parameters import Axis
from pybricks.tools import wait, StopWatch


class SpikeGyro:
    def __init__(self, hub, axis: Axis = Axis.Z):
        self.hub = hub
        self.axis = axis

    def calibrate(self, timeout_ms: int = 5000) -> bool:
        watch = StopWatch()
        while not self.hub.imu.ready():
            if watch.time() > timeout_ms:
                print("IMU not ready within timeout.")
                return False
            wait(20)

        stable = StopWatch()
        while stable.time() < 1000:
            if not self.hub.imu.stationary():
                stable.reset()
            if watch.time() > timeout_ms + 2000:
                print("IMU never stationary enough.")
                return False
            wait(20)

        self.hub.imu.reset_heading(0)
        print("Calibration complete.")
        return True


    def set_zero(self) -> None:
        self.hub.imu.reset_heading(0)

    def get_rotation(self, use_heading: bool = True):
        if use_heading:
            return self.hub.imu.heading()
        return self.hub.imu.rotation(self.axis, calibrated=True)


    def ready(self):
        return self.hub.imu.ready()

    def stationary(self):
        return self.hub.imu.stationary()

    