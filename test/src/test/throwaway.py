# for testing
import math

from baseBot.MapManager import MapManager

if __name__ == '__main__':
    mapManager = MapManager()
    map = [0] * 10000  # 100 by 100 height map of 0s
    map_width = int(math.sqrt(len(map)))
    mapManager.mapMaker.map = map
    mapManager.update_map()
