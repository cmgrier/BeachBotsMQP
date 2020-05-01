#!/usr/bin/env python3
import pyzed.sl as sl
import time
import math
import numpy as np

import rospy

from support.Constants import *

# creates maps for the Map Manager class
# manages the ZED camera
class MapMaker:
    def __init__(self):
        self.maps = list()
        #self.map = []
        self.mesh = sl.Mesh()
        print("Finding ZED...")
        self.zed = sl.Camera()
        self.translation = [0, 0, 0]
        self.orientation = [0, 0, 0, 0]
        self.init_params = sl.InitParameters()
        py_transform = sl.Transform()
        trackparms = sl.TrackingParameters(init_pos=py_transform)
        trackparms.enable_pose_smoothing = True
        self.tracking_params = trackparms
        self.mapping_params = sl.SpatialMappingParameters(
            resolution=sl.MAPPING_RESOLUTION.MAPPING_RESOLUTION_LOW,
            mapping_range=sl.MAPPING_RANGE.MAPPING_RANGE_FAR,
            save_texture=True)
        self.filter_params = sl.MeshFilterParameters()
        self.runtime_params = sl.RuntimeParameters()
        print("Starting ZED...")
        self.start_camera()
        self.update_pose()
        self.has_requested_map = False
        self.last_update_time = time.time()

    # starts up the ZED camera with specific parameters, only needs to be run on class startup
    def start_camera(self):
        self.init_params.camera_resolution = sl.RESOLUTION.RESOLUTION_HD720  # Use HD720 video mode (default fps: 60)
        self.init_params.coordinate_system = sl.COORDINATE_SYSTEM.COORDINATE_SYSTEM_RIGHT_HANDED_Z_UP  # Use a right-handed Y-up coordinate system
        self.init_params.coordinate_units = sl.UNIT.UNIT_METER  # Set units in meters
        print("Opening...")
        result = self.zed.open(self.init_params)
        print(result)

        self.mapping_params.map_type = sl.SPATIAL_MAP_TYPE.SPATIAL_MAP_TYPE_MESH
        self.mapping_params.save_texture = True     # may change this to reduce load on system
        self.filter_params.set(sl.MESH_FILTER.MESH_FILTER_HIGH)  # not available for fused point cloud
        print("Enabling Tracking...")
        self.zed.enable_tracking(self.tracking_params)
        print("Enabling Spatial Mapping...")
        self.zed.enable_spatial_mapping(self.mapping_params)

    # shuts down the camera, should be called when the basebot shuts down to ensure safe shutdown
    def close_camera(self):
        # Disable tracking and mapping and close the camera
        self.zed.disable_spatial_mapping()
        self.zed.disable_tracking()
        self.zed.close()

    # method to return the map TODO check to see if it can be removed
    def get_map(self):
        return None
        #return self.map

    # starts an asynchronous process that generates a mesh
    def make_mesh(self):
        if self.zed.grab(sl.RuntimeParameters()) == sl.ERROR_CODE.SUCCESS:
            self.zed.request_spatial_map_async()

    # starts a blocking process that will generate a mesh and updates the self.mesh parameter with the new mesh
    def update_map_blocking(self):
        mesh = sl.Mesh()
        # Retrieve the spatial map, this is blocking
        self.zed.extract_whole_spatial_map(mesh)
        # Filter the mesh
        mesh.filter(self.filter_params)  # not available for fused point cloud
        self.mesh = mesh

    # updates the self.mesh with an asynchronous process. Will only update as frequently as the MESH_REFRESH_RATE
    def update_map_async(self, apply_texture = False):
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
            if apply_texture:
                if DEBUG:
                    print("applying texture...")
                self.mesh.apply_texture()
                self.mesh.save("MYMAP")
        return status

    # requests a new frame to be captured from the zed camera, will block until the frame is grabbed
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

    # updates the position and orientation of the zed camera, should be run in main loop. Orientation is a quaternion
    def update_pose(self):
        pose = sl.Pose()
        if self.zed.get_position(pose, sl.REFERENCE_FRAME.REFERENCE_FRAME_WORLD) == sl.TRACKING_STATE.TRACKING_STATE_OK:
            t = pose.get_translation()
            self.translation = [t.get()[0], t.get()[1], t.get()[2]]
            o = pose.get_orientation()
            self.orientation = [o.get()[0], o.get()[1], o.get()[2], o.get()[3]]

    # updates position and orientation, but only the z orientation. x and y are assumed to be 0
    def update_pose_z(self):
        pose = sl.Pose()
        if self.zed.get_position(pose, sl.REFERENCE_FRAME.REFERENCE_FRAME_WORLD) == sl.TRACKING_STATE.TRACKING_STATE_OK:
            t = pose.get_translation()
            self.translation = [t.get()[0], t.get()[1], t.get()[2]]
            o = pose.get_orientation()
            euler = self.quaternion_to_euler(o.get()[0], o.get()[1], o.get()[2], o.get()[3])
            new_quat = self.euler_to_quaternion(euler[2], 0, 0)
            self.orientation = new_quat

    # converts the given quaternion to euler angles in radians
    def quaternion_to_euler(self, x, y, z, w):

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        X = math.degrees(math.atan2(t0, t1))

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        Y = math.degrees(math.asin(t2))

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        Z = math.degrees(math.atan2(t3, t4))

        return X, Y, Z

    # converts the given euler angles (radian) to quaternion values
    def euler_to_quaternion(self, yaw, pitch, roll):

        qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(
            yaw / 2)
        qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(
            yaw / 2)
        qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(
            yaw / 2)
        qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(
            yaw / 2)

        return [qx, qy, qz, qw]