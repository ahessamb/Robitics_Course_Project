# important points:
# use <robot_position> to get current position of robot in <x,y,theta> format.
# use <robot_omega> to get current values for the wheels in <w1,w2,w3> format.

# import numpy as np
from initialization import *
from general_functions import *
from Bug0 import Bug0
from Bug1 import Bug1
from Bug2 import Bug2
import math
import follow_boundary as fb
import line as ln
import reachable
from general_functions import set_motor_speed

target = 1.3, 6.15  # todo

prev_pos = None
first_step = True



if __name__ == "__main__":

    TIME_STEP = 32
    robot = init_robot(time_step=TIME_STEP)
    init_robot_state(in_pos=[0, 0, 0], in_omega=[0, 0, 0])
    update_motor_speed(input_omega=[0, 0, 0])

    init_things = True
    planner = None

    while robot.step(TIME_STEP) != -1:


        gps_values, compass_val, sonar_value, encoder_value, ir_value = read_sensors_values()
        update_robot_state()
        print('Sonar: ', sonar_value,'  ir: ',ir_value)

        if init_things:
            init_things = False
            planner = Bug2(target)

        sensor_values = read_sensors_values()
        planner.next(sensor_values)
