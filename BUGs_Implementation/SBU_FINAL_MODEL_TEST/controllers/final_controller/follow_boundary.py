from general_functions import *
from const import *


class Follow_Wall:
    def __init__(self):
        self.starting_degree = 0
        self.fb_nxstate = 0
        self.wall_dist = None
        self.distance_range = (100, 250)


        self.minimum_dist = 100000
        self.minimum_dist_alpha = 0
        self.minimum_angle_dist_threshold = 1
        self.minimum_dist_threshold = 8
        self.minimum_dist_turn = 'right'
        self.left_move_times = 0
        self.no_move_stack = []

    def follow_boundary(self, sensor_values):

        fb_state = self.fb_nxstate
        obstacle_distances = check_surroundings(sensor_values)

        # tuning
        if fb_state == 0:
            if self.minimum_dist_turn == 'right':
                set_motor_speed(rotate_right, 2)
                self.set_minimum_dist(obstacle_distances[0], get_bearing_in_degrees(sensor_values[1]))
                if obstacle_distances[0] > 1000:
                    self.minimum_dist_turn = 'left'
                    set_motor_speed(rotate_left, 5)
            elif self.minimum_dist_turn == 'left':
                set_motor_speed(rotate_left, 2)
                self.set_minimum_dist(obstacle_distances[0], get_bearing_in_degrees(sensor_values[1]))

                if obstacle_distances[0] > 1000:
                    self.minimum_dist_turn = 'tune_angle'
                    set_motor_speed(rotate_right, 5)

            elif self.minimum_dist_turn == 'tune_angle':
                set_motor_speed(rotate_right, 2)
                if self.arrive_to_angle(get_bearing_in_degrees(sensor_values[1]), (self.minimum_dist_alpha)):
                    set_motor_speed(stop)
                    self.minimum_dist_turn = 'tune_distance'
            elif self.minimum_dist_turn == 'tune_distance':
                if abs(obstacle_distances[0] - self.distance_range[1]) < self.minimum_dist_threshold:
                    set_motor_speed(stop)
                    self.set_minimum_dist(obstacle_distances[0], get_bearing_in_degrees(sensor_values[1]), force=True)
                    self.minimum_dist_turn = '15_degree'
                elif obstacle_distances[0] - self.distance_range[1] < 0:
                    set_motor_speed(move_backward, 1)
                else:
                    set_motor_speed(move_forward, 1)

            elif self.minimum_dist_turn == '15_degree':
                set_motor_speed(rotate_right, 2)
                if self.arrive_to_angle(get_bearing_in_degrees(sensor_values[1]), self.minimum_dist_alpha, -15):
                    set_motor_speed(stop)
                    self.reset_state1()
                    self.fb_nxstate = 1

        # move left side
        elif fb_state == 1:
            set_motor_speed(move_left, 5)
            self.left_move_times += 1
            if 10 < obstacle_distances[0] - self.minimum_dist < 400:
                set_motor_speed(move_left_complement, 5)
            elif obstacle_distances[0] - self.minimum_dist > 900:
                self.starting_degree = get_bearing_in_degrees(sensor_values[1])
                self.fb_nxstate = 103
            elif self.distance_range[0] < obstacle_distances[2] < self.distance_range[1]:
                self.starting_degree = get_bearing_in_degrees(sensor_values[1])
                self.fb_nxstate = 2

            if self.left_move_times % 8 == 0:
                if self.left_move_times == 400:
                    self.fb_nxstate = 0
                    self.reset_tuner()
                    self.left_move_times = 0

                if self.arrive_to_angle(get_bearing_in_degrees(sensor_values[1]), self.minimum_dist_alpha, -15):
                    set_motor_speed(rotate_left, 1)

        # turn left side
        elif fb_state == 2:
            if self.arrive_to_angle(get_bearing_in_degrees(sensor_values[1]), self.starting_degree, 120):
                self.fb_nxstate = 0
                self.reset_tuner()
            else:
                set_motor_speed(rotate_left, 2)

        # turn right side (curve)
        elif fb_state == 3:
            if self.arrive_to_angle(get_bearing_in_degrees(sensor_values[1]), self.starting_degree, -75):
                self.fb_nxstate = 0
                self.reset_tuner()
            else:
                set_motor_speed(turn_right_wall_following, 2)
        # end of wall insurance
        elif fb_state == 103:
            set_motor_speed(rotate_left, 6)
            self.fb_nxstate = 113
        # end of wall insurance
        elif fb_state == 113:
            if obstacle_distances[0] - self.minimum_dist < 900:
                self.fb_nxstate = 0
                print("---------------returned to normal-------------")
            else:
                set_motor_speed(rotate_right, 6)
                self.fb_nxstate = 3
                print("------------------rotate --------------")

        return fb_state

    def reset_tuner(self, goto0=False):
        if goto0:
            self.fb_nxstate = 0
        self.minimum_dist_turn = 'right'
        self.minimum_dist = 100000
        self.minimum_dist_alpha = 0

    def reset_state1(self):
        self.left_move_times = 0

    def set_minimum_dist(self, dist, angle, force=False):
        if force or dist < self.minimum_dist:
            self.minimum_dist = dist
            self.minimum_dist_alpha = angle

    def arrive_to_angle(self, alpha, beta, gama=0):
        #  result = alpha - (beta + gama)
        betagama = beta + gama
        if betagama > 180:
            betagama = betagama - 360
        elif betagama <= -180:
            betagama = betagama + 360

        if abs(alpha - betagama) < self.minimum_angle_dist_threshold:
            return True
        else:
            return False
