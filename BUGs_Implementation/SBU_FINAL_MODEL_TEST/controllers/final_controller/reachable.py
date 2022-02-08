from general_functions import *
from const import *


def is_reachable(sensor_values, target):
    _, (x, y, alpha) = get_distance(sensor_values, target)
    head_degree = get_bearing_in_degrees(sensor_values[1])
    differential_degree = alpha - head_degree if alpha > head_degree else head_degree - alpha

    if 90 <= differential_degree <= 270:
        return True
    else:
        return False
