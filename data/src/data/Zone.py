#!/usr/bin/env python
from geometry_msgs.msg import Pose
class Zone:
    def __init__(self, zone_corners, id):
        self.corners = zone_corners  # list of [top left, top right, bottom right, bottom left]
        self.id = id

    #   x ->
    # y 1  2  3  4
    # |    map
    # v 9 10 11 12
    def is_out_of_zone(self, position, map):
        point_corners = []
        for index in self.corners:
            point_corners.append(self.index_to_point(index, map))
        if point_corners[1][0] > point_corners[2][0]:
            x_max = point_corners[1][0]
        else:
            x_max = point_corners[2][0]

        if point_corners[0][0] > point_corners[3][0]:
            x_min = point_corners[3][0]
        else:
            x_min = point_corners[0][0]

        if point_corners[2][1] > point_corners[3][1]:
            y_max = point_corners[2][1]
        else:
            y_max = point_corners[3][1]

        if point_corners[0][1] > point_corners[1][1]:
            y_min = point_corners[1][1]
        else:
            y_min = point_corners[0][1]

        return x_max > position[0] > x_min or y_max > position[1] > y_min

    def index_to_point(self, index, map):
        x = index / map.width
        y = index % map.width
        z = map(index)
        return [x, y, z]
