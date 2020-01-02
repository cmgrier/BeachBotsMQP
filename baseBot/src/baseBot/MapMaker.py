#!/usr/bin/env python
from support import Constants
import sl

# creates maps for the Map Manager class
class MapMaker:
    def __init__(self):
        self.maps = list()
        self.map = []
        self.mesh = None
        self.init_params = None
        self.mapping_parameters = None
        self.ros_node()

    def get_map(self):
        return self.map

    def ros_node(self):
        rospy.init_node('map_maker', anonymous=True)
        # rostopic, msgtype, callback method
        # rospy.Subscriber('/orientation', Float64MultiArray, server.orientation_callback)
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.RESOLUTION_HD720  # Use HD720 video mode (default fps: 60)
        init_params.coordinate_system = sl.COORDINATE_SYSTEM.COORDINATE_SYSTEM_RIGHT_HANDED_Y_UP  # Use a right-handed Y-up coordinate system
        init_params.coordinate_units = sl.UNIT.UNIT_METER  # Set units in meters

        mapping_parameters = sl.SpatialMappingParameters()
        mapping_parameters.resolution_meter = 0.03  # Set resolution to 3cm
        mapping_parameters.resolution_meter = mapping_parameters.get_resolution_preset(
            sl.MAPPING_RESOLUTION.MAPPING_RESOLUTION_LOW)  # Or use preset

        mapping_parameters.range_meter = 5  # Set maximum depth mapping range to 5m
        mapping_parameters.range_meter = mapping_parameters.get_range_preset(
            sl.MAPPING_RANGE.MAPPING_RANGE_MEDIUM)  # Or use preset

        mapping_parameters.map_type = sl.MAP_TYPE.MAP_TYPE_MESH

        self.init_params = init_params
        self.mapping_parameters = mapping_parameters

        print("map maker node started")

    def make_mesh(self):
        self.mesh = sl.Mesh()



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