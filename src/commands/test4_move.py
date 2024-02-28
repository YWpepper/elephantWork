from pymycobot import Mercury
from arm_control import *

left_arm = Mercury("/dev/ttyTHS0")
right_arm = Mercury("/dev/ttyACM1")

# left_arm.send_angles([0, 0, 0, 0, 0, 90, 90], 50)
# left_arm.send_angle(7, 0, 10)
# right_arm.send_angle(13, -20, 10)
# print(right_arm.get_angles())

# left_arm.release_servo(7)
# left_arm.focus_servo(7)
# left_arm.set_servo_calibration(7)
# print(left_arm.get_error_information())

# left_arm.send_base_coords([384.1602264670991, 73.20234865138842, 183.32765615672707, 176.93695329017524, -0.19136428069888178, -89.91218571010839], 80)

left_arm.send_base_coord(1, 180, 80)