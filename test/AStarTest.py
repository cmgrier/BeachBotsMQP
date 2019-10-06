# for testing
import math

from navigation.AStar import AStar, Map

if __name__ == '__main__':
    map = Map()
    map.map = [0, 0, 0, 0,
               -1, 0, 0, 0,
               0, 0, -1, 0,
               0, 0, 0, 0]  # length = 16
    map.width = int(math.sqrt(map.map.__len__()))
    a_star = AStar(map)
    assert a_star.find_path(0,0) == [0]

    path = a_star.find_path(0, 1)
    print(path)
    print("all tests passed")
