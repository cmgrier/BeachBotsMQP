from baseBot.MapMaker import MapMaker


# This class holds the current map and passes it between the other managers
class MapManager:

    def __init__(self):
        self.zones = list()
        self.map = []
        self.mapMaker = MapMaker()

        self.update_map()

    # run on startup
    # sets the current map to be the avg map
    def update_map(self):
        self.map = self.mapMaker.create_avg_map()

    # divides the map into different zones
    def create_zones(self):
        self.zones = list()  # todo
