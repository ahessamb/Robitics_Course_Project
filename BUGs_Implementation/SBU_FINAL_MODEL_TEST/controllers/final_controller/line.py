import math


class Line:
    def __init__(self, target, source):
        self.source = source
        self.target = target

        self.b, self.a, self.c = self.get_line()

    def get_line(self):
        dx = self.target[0] - self.source[0]
        dy = self.target[1] - self.source[1]
        slope = dy / dx
        return -1, slope, -slope * self.source[0] + self.target[0]

    def get_distance_from_line(self, current_position):
        s = abs(self.a * current_position[0] + self.b * current_position[1] + self.c)
        m = math.sqrt(self.a ** 2 + self.b ** 2)

        return s / m

    def is_on_line(self, current_position, threshold=0.1):
        d = self.get_distance_from_line(current_position)
        if d < threshold:
            if ((self.source[0] <= current_position[0] <= self.target[0]) or (self.source[0] >= current_position[0] >= self.target[0]))and ((self.source[1] <= current_position[1] <= self.target[1]) or (self.source[1] >= current_position[1] >= self.target[1])):
                    return True
        return False

    def complete_line(self, sensor_values, leaving_position):
        h,_ = get_distance_between_two_points(leaving_position, self.target)
        current_distance_from_target,_ = get_distance_between_two_points(sensor_values[0], self.target)
        print(current_distance_from_target , h ,self.is_on_line(sensor_values[0]),'<<<<<<<<<<<<<<' )
        if current_distance_from_target < h and self.is_on_line(sensor_values[0]):
            return True
        return False


def get_distance_between_two_points(source, target):
    x_threshold = 0.001

    # print(source,target)
    x = (target[0] - source[0])
    y = (target[1] - source[1])
    alpha = math.atan2(y, x) * 180 / math.pi

    return (x ** 2 + y ** 2) ** 0.5, (x, y, alpha)
