#!/usr/bin/env python
from support import Constants


# creates maps for the Map Manager class
class MapMaker:
    def __init__(self):
        self.maps = list()
        self.map = []

    def get_map(self):
        return self.map

    # Following code shouldn't be necessary because ZED should avg maps for us

    def add_map_from_mesh(self, mesh):
        self.add_map(mesh.vertices)

    def add_map(self, map):
        if self.maps.__len__() < Constants.stored_map_size:
            self.maps.append(map)
        else:
            self.maps.pop(0)
            self.maps.append(map)
        return map

    def create_avg_map(self):
        avg_map = [sum(i) for i in zip(self.maps)]
        self.map = avg_map
        return avg_map

