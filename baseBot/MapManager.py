from baseBot.MapMaker import MapMaker
import math

# This class holds the current map and passes it between the other managers
from data.Zone import Zone
from support import Constants


class MapManager:

    def __init__(self):
        self.zones = list()
        self.map = []   # should be a Mesh type
        self.mapMaker = MapMaker()
        self.landing_strip = []     # [top_left, top_right, bottom_right, bottom_left]
        self.update_map()

    # run on startup
    # sets the current map to be the avg map
    def update_map(self):
        self.map = self.mapMaker.create_avg_map()

    # divides the map into different zones
    def create_zones(self):
        self.zones = list()
        map_width = int(math.sqrt(len(self.map)))
        number_of_zones = int(map_width / Constants.zone_width)

        for i in range(0, number_of_zones - 1):
            top_left = i * Constants.zone_width
            top_right = top_left + Constants.zone_width
            bottom_left = top_left + map_width * (map_width - Constants.landing_strip_width)
            bottom_right = bottom_left + Constants.zone_width
            zone_corners = [top_left, top_right, bottom_right, bottom_left]
            new_zone = Zone(zone_corners, i)
            self.zones.append(new_zone)

        top_left = self.zones[len(self.zones) - 1].corners[1]
        top_right = map_width - 1
        bottom_left = self.zones[len(self.zones) - 1].corners[3]
        bottom_right = top_right + map_width * (map_width - Constants.landing_strip_width)
        new_zone = Zone([top_left, top_right, bottom_right, bottom_left], len(self.zones))
        self.zones.append(new_zone)

    def __create_landing_strip(self):
        map_width = int(math.sqrt(len(self.map)))
        top_left = map_width * (map_width - Constants.landing_strip_width)
        top_right = top_left + map_width
        bottom_right = len(self.map) - 1
        bottom_left = bottom_right - map_width
        self.landing_strip = [top_left, top_right, bottom_right, bottom_left]

    def percent_of_indexes_safe(self, index_list):
        indexes_too_difficult = 0.0
        for index in index_list:
            if self.map[index] > Constants.terrain_too_difficult or self.map[index] == -1:
                indexes_too_difficult += 1.0

        return indexes_too_difficult / float(len(index_list))

    def point_to_index(self, position):
        return 1