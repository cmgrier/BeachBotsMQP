#!/usr/bin/env python
# for testing
import math
import rospy
from navigation.AStar import AStar, Map

if __name__ == '__main__':
    map = Map()
    map.map = [0, 0, 0, 0,
               -1, 0, 0, 0,
               0, 0, -1, 0,
               0, 0, 0, 0]  # length = 16
    map.width = int(math.sqrt(map.map.__len__()))
    a_star = AStar(map)
    assert a_star.find_path(0, 0) == [0]

    path = a_star.find_path(0, 14)
    assert path == [0, 1, 5, 9, 13, 14]

