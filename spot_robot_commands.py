# Import Necessary Bosdyn Libraries
from bosdyn.client.frame_helpers import (BODY_FRAME_NAME, GRAV_ALIGNED_BODY_FRAME_NAME, ODOM_FRAME_NAME, get_a_tform_b)
from bosdyn.client.robot_command import RobotCommandBuilder, RobotCommandClient, blocking_stand, block_until_arm_arrives
from bosdyn.client.world_object import WorldObjectClient
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.api import geometry_pb2, world_object_pb2
from bosdyn.client.image import ImageClient
from bosdyn.client.power import PowerClient
from bosdyn.client import math_helpers
import bosdyn.client.lease
import bosdyn.client.util
import bosdyn.client

# Import Necessary Libraries
import numpy as np
import utils
import time
import os


# Define a Function to Setup and Configure SPOT Robot
def setup_and_configure_robot():

    # Define the SPOT Robot Credentials
    os.environ['BOSDYN_CLIENT_USERNAME'] = "admin"
    os.environ['BOSDYN_CLIENT_PASSWORD'] = "pvwmr4j08osj"
    robot_ip = "192.168.80.3"

    # Setup Configurations for SPOT
    bosdyn.client.util.setup_logging(False)
    sdk = bosdyn.client.create_standard_sdk('ArrangeChairs')
    robot = sdk.create_robot(robot_ip)
    bosdyn.client.util.authenticate(robot)
    robot.time_sync.wait_for_sync()

    # Return the Robot object
    return robot


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

        # Command SPOT to Stand
        robot.logger.info('Commanding SPOT to Stand...')
        command_client = robot.ensure_client(RobotCommandClient.default_service_name)
        blocking_stand(command_client, timeout_sec = 10)
        time.sleep(1)


# Define a Class to Detect Fiducials with SPOT robot
class DetectFiducial(object):

    # Define the Initialisation function
    def __init__(self, robot):

        # Define the Robot instance variables
        self._robot = robot
        self._power_client = robot.ensure_client(PowerClient.default_service_name)
        self._image_client = robot.ensure_client(ImageClient.default_service_name)
        self._robot_state_client = robot.ensure_client(RobotStateClient.default_service_name)
        self._robot_command_client = robot.ensure_client(RobotCommandClient.default_service_name)
        self._world_object_client = robot.ensure_client(WorldObjectClient.default_service_name)

    # Define a Function to get the Latest Robot state proto
    @property
    def robot_state(self):
        return self._robot_state_client.get_robot_state()

    # Define a Function to Detect Fiducials wrt SPOT Body frame
    def detect_aruco_tags_wrt_spot_body_frame(self):
        
        # Sync Robot time
        self._robot.logger.info('Detecting Fiducials in Environment wrt SPOT body frame...')
        self._robot.time_sync.wait_for_sync()

        # Get the fiducial objects Spot detects with the world object service
        for _ in range(3):
            fiducials = self.get_fiducial_objects()
            time.sleep(1)
        aruco_tags_wrt_spot_body_frame = []

        # If Fiducials are Detected
        if fiducials is not None:

            # For every Fiducial Detected
            for fiducial in fiducials:

                # Initialise AruCo tag wrt SPOT Body frame
                aruco_tag_wrt_spot_body_frame = {}

                # Get the Name of the AruCo tag in Environment
                aruco_tag_wrt_spot_body_frame['ID'] = int(fiducial.apriltag_properties.frame_name_fiducial[-1])
                       
                # Get its Transformation wrt SPOT body frame
                fiducial_wrt_spot_body = get_a_tform_b(
                                                        fiducial.transforms_snapshot, BODY_FRAME_NAME,
                                                        fiducial.apriltag_properties.frame_name_fiducial
                                                      ).to_proto()

                # Store the Pose of AruCo tag wrt SPOT Body frame
                aruco_tag_wrt_spot_body_frame['Pose'] = utils.compute_pose_from_spot_data(fiducial_wrt_spot_body)

                # Update Pose of AruCo tag by Rotating -90 deg along Z-axis
                aruco_tag_wrt_spot_body_frame['Pose'] = aruco_tag_wrt_spot_body_frame['Pose'] @ np.array([
                                                                                                            [ 0, -1,  0,  0],
                                                                                                            [ 1,  0,  0,  0],
                                                                                                            [ 0,  0,  1,  0],
                                                                                                            [ 0,  0,  0,  1]
                                                                                                         ])

                # Append into List of Fiducials detected
                aruco_tags_wrt_spot_body_frame.append(aruco_tag_wrt_spot_body_frame)
        
        # Return the List fo Fiducials detected
        return aruco_tags_wrt_spot_body_frame
    

    # Define a Function to Get all fiducials that Spot detects with its perception system
    def get_fiducial_objects(self):

        # Get all fiducial objects (an object of a specific type)
        fiducial_objects = self._world_object_client.list_world_objects(object_type = [world_object_pb2.WORLD_OBJECT_APRILTAG]).world_objects
        
        # Return the all detected fiducials
        if len(fiducial_objects) > 0:
            return fiducial_objects
        
        # Return none if no fiducials are found
        return None


# Define a Function to Open/Close Gripper of SPOT arm
def open_or_close_gripper(robot, action):

    # Verify the robot is not estopped and that an external application has registered and holds an estop endpoint.
    assert not robot.is_estopped(), 'Robot is estopped. Please use an external E-Stop client, such as the estop SDK example, to configure E-Stop.'

    # Create required Robot clients
    lease_client = robot.ensure_client(bosdyn.client.lease.LeaseClient.default_service_name)
    command_client = robot.ensure_client(RobotCommandClient.default_service_name)

    # Until Lease exists
    with bosdyn.client.lease.LeaseKeepAlive(lease_client, must_acquire = True, return_at_exit = False):

        # Make the open/close gripper RobotCommand
        if action == 'open':
            gripper_command = RobotCommandBuilder.claw_gripper_open_fraction_command(1.0)
            robot.logger.info('Opening Gripper')
        elif action == 'close':
            gripper_command = RobotCommandBuilder.claw_gripper_open_fraction_command(0.0)
            robot.logger.info('Closing Gripper')
        
        # Send the Request
        command = RobotCommandBuilder.build_synchro_command(gripper_command)
        cmd_id = command_client.robot_command(command)
        time.sleep(1)
    

# Define a Function to Move SPOT arm to given Pose
def move_arm_to_pose(robot, pose):

    # Verify the robot is not estopped and that an external application has registered and holds an estop endpoint.
    assert not robot.is_estopped(), 'Robot is estopped. Please use an external E-Stop client, such as the estop SDK example, to configure E-Stop.'

    # Create required Robot clients
    robot_state_client = robot.ensure_client(RobotStateClient.default_service_name)
    lease_client = robot.ensure_client(bosdyn.client.lease.LeaseClient.default_service_name)
    command_client = robot.ensure_client(RobotCommandClient.default_service_name)

    # Until Lease exists
    with bosdyn.client.lease.LeaseKeepAlive(lease_client, must_acquire = True, return_at_exit = False):
       
        # Get Translation and Rotation quartenion from Pose
        translation, quartenion = utils.get_translation_and_rotation_from_pose(pose, angle = False)

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
        odom_T_hand = odom_T_flat_body * math_helpers.SE3Pose.from_proto(flat_body_T_hand)

        # Define the Arm Command
        arm_command = RobotCommandBuilder.arm_pose_command(
            odom_T_hand.x, odom_T_hand.y, odom_T_hand.z, odom_T_hand.rot.w, odom_T_hand.rot.x,
            odom_T_hand.rot.y, odom_T_hand.rot.z, ODOM_FRAME_NAME, 10)
        
        # Tell the robot's body to follow the arm
        follow_arm_command = RobotCommandBuilder.follow_arm_command()

        # Combine the arm and mobility commands into one synchronized command
        command = RobotCommandBuilder.build_synchro_command(follow_arm_command, arm_command)

        # Send the request
        move_command_id = command_client.robot_command(command)
        robot.logger.info('Moving arm to given pose')
        block_until_arm_arrives(command_client, move_command_id)


# Define a Function to Move arm to Default Pose
def move_arm_to_default_pose(robot):

    # Verify the robot is not estopped and that an external application has registered and holds an estop endpoint.
    assert not robot.is_estopped(), 'Robot is estopped. Please use an external E-Stop client, such as the estop SDK example, to configure E-Stop.'

    # Create required Robot clients
    lease_client = robot.ensure_client(bosdyn.client.lease.LeaseClient.default_service_name)
    command_client = robot.ensure_client(RobotCommandClient.default_service_name)

    # Until Lease exists
    with bosdyn.client.lease.LeaseKeepAlive(lease_client, must_acquire = True, return_at_exit = False):

        # Bring the Arm back to Initial Configuration
        gripper_initial_config = RobotCommandBuilder.arm_stow_command()
        command = RobotCommandBuilder.build_synchro_command(gripper_initial_config)

        # Send the Request
        move_command_id = command_client.robot_command(command)
        robot.logger.info('Moving arm to default pose')
        block_until_arm_arrives(command_client, move_command_id)


# Define a Function to Move SPOT to a given Pose
def move_robot_to_location(robot, pose):

    # Verify the robot is not estopped and that an external application has registered and holds an estop endpoint.
    assert not robot.is_estopped(), 'Robot is estopped. Please use an external E-Stop client such as the estop SDK example, to configure E-Stop.'

    # Define Clients to Read Images
    lease_client = robot.ensure_client(bosdyn.client.lease.LeaseClient.default_service_name)

    # Get the Translation and Rotation from Pose
    translation, rotation = utils.get_translation_and_rotation_from_pose(pose, angle = True)
    
    # Until Lease is kept alive
    with bosdyn.client.lease.LeaseKeepAlive(lease_client, must_acquire = True, return_at_exit = False):
        
        # Create a Command Client
        command_client = robot.ensure_client(RobotCommandClient.default_service_name)

        # Define the Command and send the Request
        cmd = RobotCommandBuilder.synchro_velocity_command(v_x = translation[0], v_y = translation[1], v_rot = np.deg2rad(rotation[2]))
        command_client.robot_command(cmd, end_time_secs = time.time() + 1)


# Define a Function to Move Robot gripper to Grasp Chair
def move_arm_to_grasp_chair(robot, pose):

    # Open arm gripper first
    open_or_close_gripper(robot, action = 'open')

    # Move arm to Grasp pose
    move_arm_to_pose(robot, pose)
    
    # Close arm gripper to grasp chair
    open_or_close_gripper(robot, action = 'close')
    return True