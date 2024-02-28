from pymycobot import Mercury
from arm_control import pump_on, pump_off
import time

arm = Mercury("/dev/ttyTHS0")

# print("start")
# pump_on(arm)
# time.sleep(10)
# print("end")

pump_off(arm)
time.sleep(3)
