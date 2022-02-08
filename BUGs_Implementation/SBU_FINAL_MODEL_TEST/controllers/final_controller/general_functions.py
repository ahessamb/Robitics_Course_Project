from initialization import *
from const import *


def set_motor_speed(move_vector, gain=1.0):
    speeds = move_vector[0] * gain, move_vector[1] * gain, move_vector[2] * gain
    update_motor_speed(speeds)


def is_arrived(sensor_values, target, arrive_threshold=0.001):
    distance, _ = get_distance(sensor_values, target)
    return True if distance < arrive_threshold else False


def current_position(sensor_values):
    return sensor_values[0]


def get_distance(sensor_values, target):
    source = current_position(sensor_values)
    x = (target[0] - source[0])
    y = (target[1] - source[1])
    alpha = math.atan2(y, x) * 180 / math.pi
    return (x ** 2 + y ** 2) ** 0.5, (x, y, alpha)


def complete_loop(sensor_values, leaving_position,threshold=1):
    return is_arrived(sensor_values, leaving_position, threshold)


def get_bearing_in_degrees(north):
    rad = math.atan2(north[0], north[1])
    bearing = (rad) / math.pi * 180.0
    if bearing < 0.0:
        bearing = bearing + 360
    bearing -= 180

    return bearing


def go_to_target(sensor_values, target, move_speed=10):
    alpha_threshold = 0.1
    alpha_precise_threshold = 30
    alpha_precise2_threshold = 5
    moving_notRotating = False

    _, (x, y, alpha) = get_distance(sensor_values, target)
    head_degree = get_bearing_in_degrees(sensor_values[1])
    differential_degree = alpha - head_degree
    if abs(differential_degree) > alpha_precise_threshold:
        if differential_degree < 0:
            set_motor_speed(rotate_right, 10)
        else:
            set_motor_speed(rotate_left, 10)
    elif abs(differential_degree) > alpha_precise2_threshold:
        if differential_degree < 0:
            set_motor_speed(rotate_right, 2)
        else:
            set_motor_speed(rotate_left, 2)
    elif abs(differential_degree) > alpha_threshold:
        if differential_degree < 0:
            set_motor_speed(rotate_right, 0.5)
        else:
            set_motor_speed(rotate_left)
    else:
        moving_notRotating = True
        set_motor_speed(move_forward, move_speed)

    return moving_notRotating


def check_surroundings(sensor_values):
    sonar_threshold = 900
    ir_threshold = 900
    _, compass_val, sonar_value, _, ir_value = sensor_values
    obstacles = [10000, 10000, 10000]
    if sonar_value[1] < sonar_threshold or ir_value[0] < ir_threshold or ir_value[3] < ir_threshold:
        obstacles[0] = sonar_value[1] if (sonar_value[1] < sonar_threshold) else 0
    if sonar_value[0] < sonar_threshold or ir_value[2] < ir_threshold or ir_value[5] < ir_threshold:
        obstacles[1] = sonar_value[2] if sonar_value[0] < sonar_threshold else 0
    if sonar_value[2] < sonar_threshold or ir_value[1] < ir_threshold or ir_value[4] < ir_threshold:
        obstacles[2] = sonar_value[2] if sonar_value[2] < sonar_threshold else 0
    return obstacles


def check_obstacle(obstacle_distances, distance_range):
    if (obstacle_distances[0] > distance_range[0] and obstacle_distances[0] < distance_range[1]) or (
            obstacle_distances[1] > distance_range[0] and obstacle_distances[1] < distance_range[1]) or (
            obstacle_distances[2] > distance_range[0] and obstacle_distances[2] < distance_range[1]):
        return False
    else:
        return True


def check_collision(sensor_values):

    collision= ir_is_valid(sensor_values[4])
    return collision


def ir_is_valid(irs, max_threshold=1000):
    if (irs[0] >= max_threshold) and (irs[1] >= max_threshold) and (irs[2] >= max_threshold) and (
            irs[3] >= max_threshold) and (irs[4] >= max_threshold) and (
            irs[5] >= max_threshold):
        return False
    return True
