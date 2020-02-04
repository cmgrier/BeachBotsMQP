#!/usr/bin/env python3
import pyzed.sl as sl
import time

import rospy

from support.Constants import *

# creates maps for the Map Manager class
class MapMaker:
    def __init__(self):
        self.maps = list()
        self.map = []
        self.mesh = sl.Mesh()
        self.zed = sl.Camera()
        self.translation = [0, 0, 0]
        self.orientation = [0, 0, 0, 0]
        self.init_params = sl.InitParameters()
        py_transform = sl.Transform()
        self.tracking_params = sl.TrackingParameters(init_pos=py_transform)
        self.mapping_params = sl.SpatialMappingParameters(
            sl.MAPPING_RESOLUTION.MAPPING_RESOLUTION_LOW,
            sl.MAPPING_RANGE.MAPPING_RANGE_FAR)
        self.filter_params = sl.MeshFilterParameters()
        self.runtime_params = sl.RuntimeParameters()
        self.start_camera()
        self.update_pose()
        self.has_requested_map = False
        self.last_update_time = time.time()

    def start_camera(self):
        self.init_params.camera_resolution = sl.RESOLUTION.RESOLUTION_HD720  # Use HD720 video mode (default fps: 60)
        self.init_params.coordinate_system = sl.COORDINATE_SYSTEM.COORDINATE_SYSTEM_RIGHT_HANDED_Z_UP  # Use a right-handed Y-up coordinate system
        self.init_params.coordinate_units = sl.UNIT.UNIT_METER  # Set units in meters

        self.zed.open(self.init_params)

        self.mapping_params.map_type = sl.SPATIAL_MAP_TYPE.SPATIAL_MAP_TYPE_MESH
        self.mapping_params.save_texture = True     # may change this to reduce load on system
        self.filter_params.set(sl.MESH_FILTER.MESH_FILTER_HIGH)  # not available for fused point cloud

        self.zed.enable_tracking(self.tracking_params)
        self.zed.enable_spatial_mapping(self.mapping_params)


    def close_camera(self):
        # Disable tracking and mapping and close the camera
        self.zed.disable_spatial_mapping()
        self.zed.disable_tracking()
        self.zed.close()

    def get_map(self):
        return self.map

    def make_mesh(self):
        if self.zed.grab(sl.RuntimeParameters()) == sl.ERROR_CODE.SUCCESS:
            self.zed.request_spatial_map_async()

    def update_map_blocking(self):
        mesh = sl.Mesh()
        # Retrieve the spatial map, this is blocking
        self.zed.extract_whole_spatial_map(mesh)
        # Filter the mesh
        mesh.filter(self.filter_params)  # not available for fused point cloud
        self.mesh = mesh

    def update_map_async(self):
        if not self.has_requested_map:
            self.zed.request_spatial_map_async()
            self.has_requested_map = True
        # if mesh is ready, update stored mesh
        status = self.zed.get_spatial_map_request_status_async()
        time_from_last_update = time.time() - self.last_update_time
        if status == sl.ERROR_CODE.SUCCESS and time_from_last_update > 1 / MESH_REFRESH_RATE:
            mesh = sl.Mesh()
            # retrieve generated mesh
            self.zed.retrieve_spatial_map_async(mesh)
            # Filter the mesh
            mesh.filter(self.filter_params)  # not available for fused point cloud
            self.mesh = mesh
            self.has_requested_map = False
            self.last_update_time = time.time()
        return status

    def get_new_frame(self):
        timer = 0
        while self.zed.grab(self.runtime_params) != sl.ERROR_CODE.SUCCESS:
            # When grab() = SUCCESS, a new image, depth and pose is available.
            # Spatial mapping automatically ingests the new data to build the mesh.
            timer += 1

    # this will be called in main loop to update mesh and grab a camera frame
    def get_frame(self):
        return self.zed.grab(self.runtime_params)

    # if there is a visible ground plane, returns a Plane and a transform
    def get_ground_plane(self):
        ground = sl.Plane()
        transform = sl.Transform()
        result = self.zed.find_floor_plane(ground, transform)
        return [result, ground, transform]

    # returns mesh of the ground plane
    def get_ground_mesh(self):
        plane_result = self.get_ground_plane()
        ground_mesh = plane_result[1].extract_mesh()
        return ground_mesh

    def update_pose(self):
        pose = sl.Pose()
        self.zed.get_position(pose)
        t = pose.get_translation()
        self.translation = [t.get()[0], t.get()[1], t.get()[2]]
        o = pose.get_orientation()
        self.orientation = [o.get()[0], o.get()[1], o.get()[2], o.get()[3]]