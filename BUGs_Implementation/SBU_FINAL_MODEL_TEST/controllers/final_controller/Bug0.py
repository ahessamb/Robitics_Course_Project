from general_functions import *
from const import *
import math
import follow_boundary as fb
import reachable


class Bug0:
    def __init__(self, target):
        self.target = target
        self.next_state = 0
        self.leaving_position = None
        self.insurance_check = 0
        self.follower = None
        self.angle = 0
        self.angle_temp = 0

    def next(self, sensor_values):

        if check_collision(sensor_values):
            self.next_state = -99

        current_state = self.next_state
        # starting state ( onto the self.target)
        if current_state == 0:
            moving = go_to_target(sensor_values, self.target)
            arrived = is_arrived(sensor_values, self.target, 0.3)
            if arrived:
                self.next_state = -2
            else:
                obstacle_distances = check_surroundings(sensor_values)
                no_obs = check_obstacle(obstacle_distances, distance_range)
                if no_obs or not moving:
                    self.next_state = 0
                else:
                    self.leaving_position = current_position(sensor_values)
                    self.next_state = 4
        # obstacle assurance
        if current_state == 4:
            go_to_target(sensor_values, self.target)
            arrived = is_arrived(sensor_values, self.target, 0.5)
            if arrived:
                self.next_state = -1
            obstacle_distances = check_surroundings(sensor_values)
            no_obs = check_obstacle(obstacle_distances, distance_range)
            if no_obs:
                self.next_state = 0
                self.insurance_check = 0
            else:
                if self.insurance_check < 5:
                    self.next_state = 4
                    self.insurance_check += 1
                else:
                    self.leaving_position = current_position(sensor_values)
                    self.insurance_check = 0
                    self.next_state = 1
                    self.follower = fb.Follow_Wall()
        # follow boundary assurance for loop state
        elif current_state == 1:
            self.follower.follow_boundary(sensor_values)
            distance_from_leaving_pos, _ = get_distance(sensor_values, self.leaving_position)
            if distance_from_leaving_pos < distance_from_leaving_pos_threshold:
                self.next_state = 1
            else:
                self.next_state = 11
        # follow boundary state
        elif current_state == 11:
            self.follower.follow_boundary(sensor_values)
            if reachable.is_reachable(sensor_values, self.target):
                self.next_state = 0
            else:
                self.next_state = 11

        # adjust head angle
        elif current_state == -2:
            set_motor_speed(rotate_right)
            if abs(get_bearing_in_degrees(sensor_values[1])) < 1:
                self.next_state = -1
            else:
                self.next_state = -2
        # stop state
        elif current_state == -1:
            self.next_state = -1
            set_motor_speed(stop)

        # self adaptation
        elif current_state == -99:
            print('self adaptation')
            self.angle_temp = self.angle
            self.angle = self.self_adaptation(sensor_values)
            if self.angle is None:
                self.next_state = -98
                self.angle = self.angle_temp
            else:
                self.next_state = -99

        elif current_state == -98:
            print('self adaptation degree')
            set_motor_speed(rotate_right)
            if abs(get_bearing_in_degrees(sensor_values[1]) - self.angle) < 3:
                self.next_state = 1
                self.follower = fb.Follow_Wall()
            else:
                self.next_state = -98

    def self_adaptation(self, sensor_values, max_threshold=1000):

        ir_value = sensor_values[4]
        current_head = get_bearing_in_degrees(sensor_values[1])

        if (ir_value[0] < max_threshold) and (ir_value[5] < max_threshold):
            print('Second left')
            set_motor_speed(move_left)
            return current_head - 60 if current_head - 60 > -179 else current_head - 60 + 360

        elif (ir_value[2] < max_threshold) and (ir_value[4] < max_threshold):
            print('Second forward')
            set_motor_speed(move_forward)
            return current_head - 180 if current_head - 180 > -179 else current_head + 180

        elif (ir_value[1] < max_threshold) and (ir_value[3] < max_threshold):
            print('Second right', move_right)

            set_motor_speed(move_right)
            return current_head + 60 if current_head + 60 <= 180 else current_head + 60 - 360

        elif (ir_value[0] < max_threshold) or (ir_value[3] < max_threshold):
            yr, yl = 1000 - ir_value[0], 1000 - ir_value[3]
            yr, yl = yr / (yr + yl), yl / (yr + yl)
            yr, yl = yr * 1.5, yl * 1.5

            vec = [yr, -yl, 0]
            print('First backward', vec)

            set_motor_speed(vec)
            return current_head

        elif (ir_value[2] < max_threshold) or (ir_value[5] < max_threshold):
            print('Second right reverse')
            set_motor_speed(move_right, -1)
            return current_head - 120 if current_head - 120 > -179 else current_head - 120 + 360

        elif (ir_value[1] < max_threshold) or (ir_value[4] < max_threshold):
            print('Second left reverse')
            set_motor_speed(move_left, -1)
            return current_head + 120 if current_head + 120 <= 180 else current_head + 120 - 360

        elif not ir_is_valid(ir_value):
            return None

