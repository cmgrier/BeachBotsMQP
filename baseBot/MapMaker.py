from support import Constants


# creates maps for the Map Manager class
class MapMaker:
    def __init__(self):
        self.maps = list()

    def create_map(self, photo):
        map = []

        # create map using photo todo

        if self.maps.__len__() < Constants.storedMapSize:
            self.maps.append(map)
        else:
            self.maps.pop(0)
            self.maps.append(map)
        return map

    def create_avg_map(self):
        return [sum(i) for i in zip(self.maps)]
