# Import Necessary Libraries
import utils
import time

# Import Necessary Bosdyn Libraries
from bosdyn.client.robot_command import RobotCommandBuilder, RobotCommandClient, blocking_stand, block_until_arm_arrives
from bosdyn.api import geometry_pb2, image_pb2, world_object_pb2
from bosdyn.client.frame_helpers import (GRAV_ALIGNED_BODY_FRAME_NAME, ODOM_FRAME_NAME, get_a_tform_b)
from bosdyn.client import RpcError, create_standard_sdk
from bosdyn.client.world_object import WorldObjectClient
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.robot_id import RobotIdClient
from bosdyn.client.image import ImageClient
from bosdyn.client.power import PowerClient
from bosdyn.client import math_helpers
import bosdyn.client.lease
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
        time.sleep(5)


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
                    fiducial.transforms_snapshot, GRAV_ALIGNED_BODY_FRAME_NAME,
                    fiducial.apriltag_properties.frame_name_fiducial).to_proto()

                # Store the Pose of AruCo tag wrt SPOT Body frame
                aruco_tag_wrt_spot_body_frame['Pose'] = utils.compute_pose_from_spot_data(fiducial_wrt_spot_body)

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
    

# Define a Function to Move Robot gripper to given Location
def move_arm_to_location(robot, pose):

    # Verify the robot is not estopped and that an external application has registered and holds
    # an estop endpoint.
    assert not robot.is_estopped(), 'Robot is estopped. Please use an external E-Stop client, ' \
                                    'such as the estop SDK example, to configure E-Stop.'

    # Create required Robot clients
    robot_state_client = robot.ensure_client(RobotStateClient.default_service_name)
    lease_client = robot.ensure_client(bosdyn.client.lease.LeaseClient.default_service_name)
    command_client = robot.ensure_client(RobotCommandClient.default_service_name)

    # Until Lease exists
    with bosdyn.client.lease.LeaseKeepAlive(lease_client, must_acquire = True, return_at_exit = False):
       
        # Get Translation and Rotation quartenion from Pose
        translation, quartenion = utils.get_translation_and_quartenion_from_pose(pose)

        # Build a position to move the arm to (in meters, relative to and expressed in the gravity aligned body frame).
        x, y, z = translation
        hand_ewrt_flat_body = geometry_pb2.Vec3(x = x, y = y, z = z)

        # Get Rotation as a quaternion
        qx, qy, qz, qw = quartenion
        flat_body_Q_hand = geometry_pb2.Quaternion(w = qw, x = qx, y = qy, z = qz)
        
        # Frame the Pose
        flat_body_T_hand = geometry_pb2.SE3Pose(position = hand_ewrt_flat_body, rotation = flat_body_Q_hand)

        # Get Transformation from ODOM frame to Gravity aligneed Body frame
        robot_state = robot_state_client.get_robot_state()
        odom_T_flat_body = get_a_tform_b(robot_state.kinematic_state.transforms_snapshot, ODOM_FRAME_NAME, GRAV_ALIGNED_BODY_FRAME_NAME)

        # Frame the Final pose
        odom_T_hand = odom_T_flat_body * math_helpers.SE3Pose.from_proto(flat_body_T_hand)

        # Duration in seconds
        seconds = 2

        # Define the Arm Command
        arm_command = RobotCommandBuilder.arm_pose_command(
            odom_T_hand.x, odom_T_hand.y, odom_T_hand.z, odom_T_hand.rot.w, odom_T_hand.rot.x,
            odom_T_hand.rot.y, odom_T_hand.rot.z, ODOM_FRAME_NAME, seconds)

        # Make the open gripper RobotCommand
        gripper_command = RobotCommandBuilder.claw_gripper_open_fraction_command(1.0)

        # Combine the arm and gripper commands into one RobotCommand
        command = RobotCommandBuilder.build_synchro_command(gripper_command, arm_command)

        # Send the request
        cmd_id = command_client.robot_command(command)
        robot.logger.info('Moving arm to Grasp Chair')
        
        # Wait until the arm arrives at the goal
        block_until_arm_arrives(command_client, cmd_id)
        robot.logger.info('Grasped Chair')