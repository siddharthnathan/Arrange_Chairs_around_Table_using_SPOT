# Import Necessary Libraries
from scipy.spatial.transform import Rotation as R
import numpy as np
import utils
import time
import cv2
import os

# Import Necessary Bosdyn Libraries
from bosdyn.client.robot_command import RobotCommandBuilder, RobotCommandClient, blocking_stand
from bosdyn.api import geometry_pb2, image_pb2, manipulation_api_pb2, world_object_pb2
from bosdyn.client.manipulation_api_client import ManipulationApiClient
from bosdyn.client.frame_helpers import (BODY_FRAME_NAME, get_a_tform_b)
from bosdyn.client import RpcError, create_standard_sdk
from bosdyn.client.world_object import WorldObjectClient
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.robot_id import RobotIdClient
from bosdyn.client.image import ImageClient
from bosdyn.client.lease import LeaseClient
from bosdyn.client.power import PowerClient
import bosdyn.client.lease
import bosdyn.client.util
import bosdyn.client


# Define a Function to Power on SPOT and make it Stand
def make_SPOT_stand(robot):
        
    # Verify the robot is not estopped and that an external application has registered and holds an estop endpoint.
    assert not robot.is_estopped(), 'Robot is estopped. Please use an external E-Stop client such as the estop SDK example, to configure E-Stop.'

    # Define Clients to Claim Lease
    lease_client = robot.ensure_client(bosdyn.client.lease.LeaseClient.default_service_name)

    # Until Lease is kept alive
    with bosdyn.client.lease.LeaseKeepAlive(lease_client, must_acquire = True, return_at_exit = False):
        
        # Power On SPOT
        robot.logger.info('Powering on SPOT...')
        robot.power_on(timeout_sec = 20)
        assert robot.is_powered_on(), 'SPOT Power on Failed.'
        robot.logger.info('SPOT Powered on')

        # Command SPOT to Stand
        robot.logger.info('Commanding SPOT to Stand...')
        command_client = robot.ensure_client(RobotCommandClient.default_service_name)
        blocking_stand(command_client, timeout_sec = 10)
        robot.logger.info('SPOT Standing')


# Define a Class to Detect Fiducials with SPOT robot
class DetectFiducial(object):

    # Define the Initialisation function
    def __init__(self, robot):

        # Define the Robot instance variables
        self._robot = robot
        self._robot_id = robot.ensure_client(RobotIdClient.default_service_name).get_id(timeout = 0.4)
        self._power_client = robot.ensure_client(PowerClient.default_service_name)
        self._image_client = robot.ensure_client(ImageClient.default_service_name)
        self._robot_state_client = robot.ensure_client(RobotStateClient.default_service_name)
        self._robot_command_client = robot.ensure_client(RobotCommandClient.default_service_name)
        self._world_object_client = robot.ensure_client(WorldObjectClient.default_service_name)
        self._use_world_object_service = True

        # Indicators for movement and image displays
        self._standup = True

        # Indicator for if motor power is on
        self._powered_on = False

        # Camera intrinsics for the current camera source being analyzed
        self._intrinsics = None

        # Transform from the robot's camera frame to the baselink frame
        self._camera_tform_body = None

        # Transform from the robot's baselink to the world frame
        self._body_tform_world = None

        # Latest detected fiducial's position in the world
        self._current_tag_world_pose = np.array([])

        # Dictionary mapping camera source to it's latest image taken
        self._image = dict()

        # List of all possible camera sources
        self._source_names = [
            src.name for src in self._image_client.list_image_sources() if
            (src.image_type == image_pb2.ImageSource.IMAGE_TYPE_VISUAL and 'depth' not in src.name)
        ]

        # Dictionary mapping camera source to previously computed extrinsics
        self._camera_to_extrinsics_guess = self.populate_source_dict()

        # Camera source which a bounding box was last detected in
        self._previous_source = None

    # Define a Function to get the Latest Robot state proto
    @property
    def robot_state(self):
        return self._robot_state_client.get_robot_state()

    # Define a Function to Get Current Image from their source
    @property
    def image(self):
        return self._image

    # Define a Function to  get List of camera sources
    @property
    def image_sources_list(self):
        return self._source_names

    # Define a Function to Populate Source dictionary
    def populate_source_dict(self):
        camera_to_extrinsics_guess = dict()
        for src in self._source_names:
            camera_to_extrinsics_guess[src] = (False, (None, None))
        return camera_to_extrinsics_guess


    # Define a Function to Start Fiducial Detection wrt SPOT Body frame
    def detect_aruco_tags_wrt_spot_body_frame(self):
        
        # Sync Robot time
        self._robot.logger.info('Detecting Fiducials in Environment wrt SPOT body frame...')
        self._robot.time_sync.wait_for_sync()

        # Get the fiducial objects Spot detects with the world object service
        fiducials = self.get_fiducial_objects()
        aruco_tags_wrt_spot_body_frame = []

        # If Fiducials are Detected
        if fiducials is not None:

            # For every Fiducial Detected
            for fiducial in fiducials:

                # Initialise AruCo tag wrt SPOT Body frame
                aruco_tag_wrt_spot_body_frame = {}

                # Get the Name of the AruCo tag in Environment
                aruco_tag_wrt_spot_body_frame['Name'] = utils.get_object_with_aruco_tag(int(fiducial.apriltag_properties.frame_name_fiducial[-1]))
                
                # Get its Transformation wrt SPOT body frame
                fiducial_wrt_spot_body = get_a_tform_b(
                    fiducial.transforms_snapshot, BODY_FRAME_NAME,
                    fiducial.apriltag_properties.frame_name_fiducial).to_proto()
                
                # Get the Translation vector & Rotation quartenion of AruCo tag wrt SPOT body frame
                translation = fiducial_wrt_spot_body.position
                translation = [translation.x, translation.y, translation.z]
                quartenion = fiducial_wrt_spot_body.rotation
                quartenion = [quartenion.x, quartenion.y, quartenion.z, quartenion.w]

                # Store the Pose of AruCo tag wrt SPOT Body frame
                aruco_tag_wrt_spot_body_frame['Pose'] = utils.compute_pose_from_quartenion(translation, quartenion)

                # Append into List of Fiducials detected
                aruco_tags_wrt_spot_body_frame.append(aruco_tag_wrt_spot_body_frame)
        
        # Return the List fo Fiducials detected
        return aruco_tags_wrt_spot_body_frame
    

    # Define a Function to Get all fiducials that Spot detects with its perception system
    def get_fiducial_objects(self):

        # Get all fiducial objects (an object of a specific type).
        request_fiducials = [world_object_pb2.WORLD_OBJECT_APRILTAG]
        fiducial_objects = self._world_object_client.list_world_objects(
            object_type = request_fiducials).world_objects
        
        # Return the all detected fiducials
        if len(fiducial_objects) > 0:
            return fiducial_objects
        
        # Return none if no fiducials are found.
        return None
    

    # Define a Function to convert Quartenion into Euler angles
    def convert_quartenion_to_angles(self, quartenion):

        # Convert Quartenion into Euler angles
        rotation = R.from_quat(quartenion)
        euler_angles = rotation.as_euler('zyx', degrees = True)
        euler_angles[2] += 90

        # Return the rounded off angles
        return utils.round_float_list(euler_angles, 3)[::-1]