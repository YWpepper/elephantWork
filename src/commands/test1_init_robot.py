from pymycobot import Mercury
import time

left_arm = Mercury("/dev/ttyTHS0", debug=True)
right_arm = Mercury("/dev/right_arm", debug=True)

left_arm.power_on()
time.sleep(0.3)
#right_arm.power_on()
time.sleep(0.3)

print(left_arm.go_zero())
time.sleep(0.3)
print(right_arm.go_zero())
time.sleep(0.3)
