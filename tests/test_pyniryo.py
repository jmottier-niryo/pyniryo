#!/usr/bin/env python

import numpy as np
import sys
import unittest
import time

from pyniryo import (NiryoRobot,
                     RobotAxis,
                     ConveyorID,
                     ToolID,
                     CalibrateMode,
                     TcpCommandException,
                     PinID,
                     PoseObject,
                     DigitalPinObject,
                     PinState,
                     PinMode,
                     AnalogPinObject,
                     ObjectShape,
                     ObjectColor,
                     NiryoRobotException,
                     PoseMetadata,
                     JointsPosition,
                     Command)

simulation = "-rpi" not in sys.argv
tool_used = ToolID.GRIPPER_1

robot_ip_address_rpi = "10.10.10.10"
robot_ip_address_gazebo = "127.0.0.1"
robot_ip_address = robot_ip_address_gazebo if simulation else robot_ip_address_rpi

#TODO: remove this hard-coded ip address
robot_ip_address = '192.168.1.140'


class BaseTestTcpApi(unittest.TestCase):

    def setUp(self):
        self.niryo_robot = NiryoRobot(robot_ip_address, verbose=False, deprecation_msg=False)
        self.niryo_robot.clear_collision_detected()

    def tearDown(self):
        self.niryo_robot.close_connection()

    @staticmethod
    def assertAlmostEqualVector(a, b, decimal=1):
        np.testing.assert_almost_equal(a, b, decimal)

    def assertAlmostEqualPose(self, a, b, decimal=1):
        if isinstance(a, list):
            a = PoseObject(*a, metadata=PoseMetadata.v1())
        if isinstance(b, list):
            b = PoseObject(*b, metadata=PoseMetadata.v1())
        self.assertEqual(a.metadata.tcp_version,
                         b.metadata.tcp_version,
                         "Can't compare two poses with different TCP versions")

        a_quaternion = [a.x, a.y, a.z] + a.quaternion()
        b_quaternion = [b.x, b.y, b.z] + b.quaternion()

        self.assertAlmostEqualVector(a_quaternion, b_quaternion, decimal)


# noinspection PyTypeChecker
class TestMainPurpose(BaseTestTcpApi):

    def test_calibration(self):
        self.assertIsNone(self.niryo_robot.calibrate(CalibrateMode.AUTO))
        self.assertIsNone(self.niryo_robot.calibrate_auto())
        self.assertIsNone(self.niryo_robot.calibrate(CalibrateMode.MANUAL))
        self.assertFalse(self.niryo_robot.need_calibration())
        with self.assertRaises(TcpCommandException):
            self.niryo_robot.calibrate(PinID)
        with self.assertRaises(TcpCommandException):
            self.niryo_robot.calibrate(1)
        with self.assertRaises(TcpCommandException):
            self.niryo_robot.calibrate(ConveyorID.ID_1)

    def test_learning_mode(self):

        def setter_learning_mode(state):
            self.niryo_robot.learning_mode = state

        base_state = self.niryo_robot.learning_mode

        self.assertIsNone(self.niryo_robot.set_learning_mode(False))
        self.assertFalse(self.niryo_robot.learning_mode)
        with self.assertRaises(TcpCommandException):
            self.niryo_robot.set_learning_mode(PinID)
        with self.assertRaises(TcpCommandException):
            self.niryo_robot.learning_mode = 1
        with self.assertRaises(TcpCommandException):
            self.niryo_robot.set_learning_mode(ConveyorID.ID_1)
        self.assertIsNone(setter_learning_mode(True))
        self.assertTrue(self.niryo_robot.learning_mode)

        self.niryo_robot.learning_mode = base_state

    def test_others(self):
        self.assertIsNone(self.niryo_robot.set_arm_max_velocity(95))
        self.assertIsNone(self.niryo_robot.set_jog_control(False))
        with self.assertRaises(TcpCommandException):
            self.assertIsNone(self.niryo_robot.set_arm_max_velocity(-95))
        with self.assertRaises(TcpCommandException):
            self.assertIsNone(self.niryo_robot.set_arm_max_velocity(0))
        with self.assertRaises(TcpCommandException):
            self.niryo_robot.set_jog_control(ConveyorID.ID_1)


# noinspection PyTypeChecker
class TestJointsPoseFunctions(BaseTestTcpApi):
    # TODO : Add limits tests for joints && poses
    def test_joints(self):

        def setter_joints(joints):
            self.niryo_robot.joints = joints

        # Classic Move Joints & Get Joints
        self.assertIsNone(self.niryo_robot.move_joints(0.1, -0.1, 0.0, 0.0, 0.0, 0.0))
        self.assertAlmostEqualVector(self.niryo_robot.joints, [0.1, -0.1, 0.0, 0.0, 0.0, 0.0])
        self.assertIsNone(setter_joints([0, 0, 0, 0, 0, 0]))
        self.assertAlmostEqualVector(self.niryo_robot.get_joints(), 6 * [0.0])
        # Jog
        self.assertIsNone(self.niryo_robot.jog_joints(0.1, -0.1, 0, 0, 0, 0))
        self.niryo_robot.wait(0.75)
        self.niryo_robot.set_jog_control(False)
        self.assertAlmostEqualVector(self.niryo_robot.get_joints(), [0.1, -0.1, 0.0, 0.0, 0.0, 0.0])
        # Check Exception
        with self.assertRaises(TcpCommandException):
            self.niryo_robot.move_joints(0.54, 0.964, 0.34, "a", "m", ConveyorID.ID_1)

    def test_pose(self):

        def setter_pose(pose):
            self.niryo_robot.pose = pose

        # Classic Move Pose & Get Pose
        self.assertIsNone(self.niryo_robot.move_pose(0.15, 0.0, 0.25, 0.0, 0.0, 0.0))
        self.assertIsNone(self.niryo_robot.move_pose([0.2, 0, 0.25, 0.0, 0.0, 0.0]))
        self.assertIsNone(setter_pose(PoseObject(0.156, 0.09, 0.34, 0.307, -0.148, 0.755, metadata=PoseMetadata.v1())))
        expected_pose = PoseObject(0.156, 0.09, 0.34, -1.11, -1.23, -1.25)
        self.assertAlmostEqualPose(self.niryo_robot.get_pose(), expected_pose)
        self.assertAlmostEqualPose(self.niryo_robot.pose, expected_pose)
        # Linear Move Pose
        self.assertIsNone(self.niryo_robot.move_linear_pose(0.15, 0.09, 0.22, 0.31, -0.15, 0.75))
        target_pose = PoseObject(0.2, 0.09, 0.22, -1.11, -1.23, -1.25)
        self.assertIsNone(self.niryo_robot.move(target_pose, move_cmd=Command.MOVE_LINEAR_POSE))
        self.assertAlmostEqualPose(self.niryo_robot.get_pose(), target_pose)
        # Shift axis & Jog
        self.assertIsNone(self.niryo_robot.shift_pose(RobotAxis.Y, 0.05))
        self.assertIsNone(self.niryo_robot.jog_pose(-0.02, 0.0, 0.02, 0.1, 0, 0))
        self.niryo_robot.set_jog_control(False)
        # Shift axis linear
        self.assertIsNone(self.niryo_robot.shift_linear_pose(RobotAxis.Y, 0.05))
        # Check Exceptions
        with self.assertRaises(TcpCommandException):
            self.niryo_robot.shift_pose(ToolID.ELECTROMAGNET_1, 0.05)
        with self.assertRaises(TcpCommandException):
            self.niryo_robot.move_pose(0.54, 0.964, 0.34, "a", "m", ConveyorID.ID_1)
        with self.assertRaises(TcpCommandException):
            self.niryo_robot.move_linear_pose(0.54, 0.964, 0.7, "a", "m", 1)
        with self.assertRaises(TcpCommandException):
            self.niryo_robot.shift_linear_pose(ConveyorID.ID_1, 0.9)

    def test_move(self):
        test_positions = [
            PoseObject(0.2, 0, 0.3, 0, 0, 0, metadata=PoseMetadata.v1()),
            PoseObject(0.14, 0, 0.204, -0.017, 0.745, -0.001, metadata=PoseMetadata.v1()),
            PoseObject(0.135, 0.067, 0.514, -1.748, -0.746, 1.305, metadata=PoseMetadata.v1()),
            PoseObject(0.052, -0.22, 0.525, -0.378, 0.068, -1.394, metadata=PoseMetadata.v1()),
            PoseObject(0.14, 0, 0.203, 0.005, 0.747, 0.001, metadata=PoseMetadata.v1()),
            PoseObject(0.14006130314993392,
                       6.048173041991534e-06,
                       0.20401713144974865,
                       3.110362076103925,
                       -0.8266489340816215,
                       0.037467558750953124),
            PoseObject(0.1432125589005656,
                       0.06831283301679453,
                       0.5002248793857345,
                       0.8520537601093433,
                       0.07059014583057896,
                       2.7549475859358963),
            PoseObject(0.06091218004601295,
                       -0.21754547980629296,
                       0.5256674137051471,
                       1.70125249680832,
                       -1.1864220377733763,
                       0.09318154272294449),
            PoseObject(0.14006130314993392,
                       6.048173041991534e-06,
                       0.20401713144974865,
                       3.110362076103925,
                       -0.8266489340816215,
                       0.037467558750953124),
            JointsPosition(0, 0, 0, 0, 0, 0),
            JointsPosition(0.1, -0.1, 0.0, 0.0, 0.0, 0.0),
            JointsPosition(0, 0.5, -1.25, 0, 0, 0),
            JointsPosition(-1, 0.5, -0.6, 0.8, 1.2, -1.4),
            JointsPosition(0, 0.5, -1.25, 0, 0, 0),
        ]

        for test_position in test_positions:
            self.niryo_robot.move(test_position)
            if isinstance(test_position, JointsPosition):
                self.assertAlmostEqualVector(self.niryo_robot.joints.to_list(), test_position.to_list())
            elif test_position.metadata.tcp_version == self.niryo_robot.pose.metadata.tcp_version:
                # can't compare poses with different conventions
                self.assertAlmostEqualPose(self.niryo_robot.pose, test_position)

            self.assertFalse(self.niryo_robot.collision_detected)

    def test_kinematics(self):
        initial_pose = self.niryo_robot.get_pose()
        # Forward Kinematics
        joints_target = 0.2, 0.0, -0.4, 0.0, 0.0, 0.0
        pose_target = self.niryo_robot.forward_kinematics(joints_target)
        pose_target_2 = self.niryo_robot.forward_kinematics(JointsPosition(*joints_target))
        self.assertAlmostEqualPose(pose_target, pose_target_2)
        self.assertIsNone(self.niryo_robot.move(pose_target))
        joints_reached = self.niryo_robot.get_joints()
        self.assertAlmostEqualVector(joints_target, joints_reached)
        # Inverse Kinematics
        joints_target_to_initial_pose = self.niryo_robot.inverse_kinematics(initial_pose)
        self.assertIsNone(self.niryo_robot.move_joints(joints_target_to_initial_pose))
        pose_reached = self.niryo_robot.get_pose()
        self.assertAlmostEqualPose(initial_pose, pose_reached)


class TestSavedPose(BaseTestTcpApi):
    # TODO : Check for non-reachable saved pose ?

    def test_creation_delete_pos(self):
        # Get saved pose list & copy it
        base_list = self.niryo_robot.get_saved_pose_list()
        new_list = [v for v in base_list]

        # Create new poses
        list_names_saved = []
        for i in range(3):
            new_name = 'test_{:03d}'.format(i)
            if i == 0:
                self.assertIsNone(self.niryo_robot.save_pose(new_name, [0.2, 0.0, 0.3, 0.0, 0.0, 0.0]))
            elif i == 1:
                self.assertIsNone(self.niryo_robot.save_pose(new_name, 0.2, 0.0, 0.3, 0.0, 0.0, 0.0))
            else:
                self.assertIsNone(
                    self.niryo_robot.save_pose(new_name,
                                               PoseObject(0.2, 0.0, 0.3, 0.0, 0.0, 0.0, metadata=PoseMetadata.v1())))
            if new_name not in new_list:
                new_list.append(new_name)
                list_names_saved.append(new_name)
            self.assertEqual(self.niryo_robot.get_saved_pose_list(), new_list)

        # Delete created poses
        for name in list_names_saved:
            saved_pose = self.niryo_robot.get_pose_saved(name)
            self.assertListEqual(saved_pose.to_list(), [0.2, 0.0, 0.3, 0.0, 0.0, 0.0])
            self.assertIsNone(self.niryo_robot.delete_pose(name))
            new_list.pop(new_list.index(name))
            self.assertEqual(self.niryo_robot.get_saved_pose_list(), new_list)

    def test_execute_pose_saved(self):
        pose_name = "test_pose_save_and_execute"
        pose_target = PoseObject(0.2, 0, 0.3, 0, -1.56, 3.14)
        # Saving pose
        self.assertIsNone(self.niryo_robot.save_pose(pose_name, pose_target))

        # Recovering pose
        pose_recup = self.niryo_robot.get_pose_saved(pose_name)
        self.assertAlmostEqualPose(pose_target, pose_recup)

        # Moving to the pose
        self.assertIsNone(self.niryo_robot.move(pose_recup))

        # Deleting the pose
        self.assertIsNone(self.niryo_robot.delete_pose(pose_name))


# noinspection PyTypeChecker
class TestPickPlaceFunction(BaseTestTcpApi):
    pose_1 = [0.2, 0.1, 0.1, 0., 1.57, 0.]
    pose_2 = [0.2, -0.1, 0.1, 0., 1.57, 0.]

    def setUp(self):
        super(TestPickPlaceFunction, self).setUp()
        self.niryo_robot.update_tool()

    def test_pick_n_place_individually(self):
        self.assertIsNone(self.niryo_robot.pick_from_pose(PoseObject(*self.pose_1, metadata=PoseMetadata.v1())))
        self.assertIsNone(self.niryo_robot.place_from_pose(*self.pose_2))

        self.assertIsNone(self.niryo_robot.pick(PoseObject(*self.pose_1, metadata=PoseMetadata.v1())))
        self.assertIsNone(self.niryo_robot.place(PoseObject(*self.pose_2, metadata=PoseMetadata.v1())))
        # Testing random values
        with self.assertRaises(TcpCommandException):
            self.niryo_robot.pick_from_pose(0.54, 0.964, 0.34, "a", "m", ConveyorID.ID_1)

    def test_pick_n_place_in_one(self):
        self.assertIsNone(
            self.niryo_robot.pick_and_place(PoseObject(*self.pose_1, metadata=PoseMetadata.v1()), self.pose_2))


class TestTrajectoryMethods(BaseTestTcpApi):
    joints_list = [[-0.493, -0.32, -0.505, -0.814, -0.282, 0], [0.834, -0.319, -0.466, 0.822, -0.275, 0],
                   [1.037, -0.081, 0.248, 1.259, -0.276, 0]]

    robot_poses = [[0.25, 0.1, 0.25, 0., 0., 0., 1.], [0.25, -0.1, 0.25, 0., 0., 0., 1.],
                   [0.25, -0.1, 0.3, 0., 0., 0., 1.], [0.25, 0.1, 0.3, 0., 0., 0., 1.]]

    def test_last_learned_trajectory(self):
        for i in range(3):
            new_name = 'test_{:03d}'.format(i)
            new_description = 'test_description_{:03d}'.format(i)
            self.assertIsNone(self.niryo_robot.save_trajectory(self.joints_list, new_name, new_description))
        self.assertIsNone(self.niryo_robot.clean_trajectory_memory())
        result = self.niryo_robot.get_saved_trajectory_list()
        self.assertEqual(result, [])

        self.assertIsNone(self.niryo_robot.save_trajectory(self.joints_list, "last_executed_trajectory", ""))
        self.assertIsNone(self.niryo_robot.save_last_learned_trajectory("unittest_name", "unittest_description"))
        result = self.niryo_robot.get_saved_trajectory_list()
        self.assertEqual(result, ["unittest_name"])

    def test_creation_delete_trajectory(self):
        # Get saved trajectory list & copy it

        self.assertIsNone(self.niryo_robot.clean_trajectory_memory())
        result = self.niryo_robot.get_saved_trajectory_list()
        self.assertEqual(result, [])

        trajectories = [self.joints_list, [JointsPosition(*joints) for joints in self.joints_list]]
        for trajectory in trajectories:
            # Create new trajectory
            name = 'test_trajectory'
            description = 'test_description'
            self.assertIsNone(self.niryo_robot.save_trajectory(trajectory, name, description))
            result = self.niryo_robot.get_saved_trajectory_list()
            self.assertEqual(result, [name])

            # Update Trajectory
            new_name = 'test_update_pose'
            new_description = 'test_update_description'
            self.assertIsNone(self.niryo_robot.update_trajectory_infos(name, new_name, new_description))
            result = self.niryo_robot.get_saved_trajectory_list()
            self.assertEqual(result, [new_name])

            # Delete created trajectory
            self.assertIsNone(self.niryo_robot.delete_trajectory(new_name))
            result = self.niryo_robot.get_saved_trajectory_list()
            self.assertEqual(result, [])

    def test_execute_trajectory(self):
        robot_positions = []
        robot_positions += [JointsPosition(*joints) for joints in self.joints_list]
        robot_positions += [PoseObject(*pose[:6], metadata=PoseMetadata.v1()) for pose in self.robot_poses]
        self.assertIsNone(self.niryo_robot.execute_trajectory(robot_positions))

    def test_execute_trajectory_from_poses(self):
        # Testing trajectory from poses
        self.assertIsNone(self.niryo_robot.execute_trajectory_from_poses(self.robot_poses))

    def test_save_and_execute_trajectory(self):
        # Create & save a trajectory, then execute it & eventually delete it
        traj_name = "test_trajectory_save_and_execute"
        traj_description = "test_description_trajectory_save_and_execute"
        self.assertIsNone(self.niryo_robot.save_trajectory(self.joints_list, traj_name, traj_description))
        self.assertIsNone(self.niryo_robot.execute_registered_trajectory(traj_name))
        self.assertIsNone(self.niryo_robot.delete_trajectory(traj_name))


class TestDynamicFrame(BaseTestTcpApi):
    robot_poses = [
        [[0.2, 0.2, 0.1, 0, 0, 0], [0.4, 0.3, 0.1, 0, 0, 0], [0.3, 0.4, 0.1, 0, 0, 0]],
        [[-0.2, -0.2, 0.1, 0, 0, 0], [-0.4, -0.3, 0.1, 0, 0, 0], [-0.3, -0.4, 0.1, 0, 0, 0]],
    ]

    robot_point = [
        [[-0.2, 0.2, 0.1], [0.4, 0.3, 0], [0.3, 0.4, 0]],
        [[0.2, -0.2, 0.1], [-0.4, -0.3, 0], [-0.3, -0.4, 0]],
    ]

    def test_main_frame(self):
        self.__test_creation_edition_frame()
        self.__test_move_in_frame()
        self.__test_deletion()

    def __test_creation_edition_frame(self):
        base_list_name, base_list_desc = self.niryo_robot.get_saved_dynamic_frame_list()
        new_list_name = [frame for frame in base_list_name]

        # Create frame by pose
        list_saved = []
        for i in range(4):
            if i < 2:
                # Test creation by poses
                new_name = 'unitTestFramePose_{:03d}'.format(i)
                new_desc = 'descTestFramePose_{:03d}'.format(i)
                pose_o = self.robot_poses[i][0]
                pose_x = self.robot_poses[i][1]
                pose_y = self.robot_poses[i][2]
                self.assertIsNone(
                    self.niryo_robot.save_dynamic_frame_from_poses(new_name,
                                                                   new_desc,
                                                                   pose_o,
                                                                   PoseObject(*pose_x, metadata=PoseMetadata.v1()),
                                                                   PoseObject(*pose_y, metadata=PoseMetadata.v1())))

                # Test edition
                new_edit_name = 'unitEditTestFramePose_{:03d}'.format(i)
                new_edit_desc = 'descEditTestFramePose_{:03d}'.format(i)
                self.assertIsNone(self.niryo_robot.edit_dynamic_frame(new_name, new_edit_name, new_edit_desc))
                self.assertEqual(self.niryo_robot.get_saved_dynamic_frame(new_edit_name)[0], new_edit_name)

                with self.assertRaises(TcpCommandException):
                    self.niryo_robot.get_saved_dynamic_frame(0)

                if new_edit_name not in new_list_name:
                    new_list_name.append(new_edit_name)
                    list_saved.append(new_edit_name)

                new_list_name.sort()

                self.assertEqual(self.niryo_robot.get_saved_dynamic_frame_list()[0], new_list_name)

                with self.assertRaises(TcpCommandException):
                    self.niryo_robot.save_dynamic_frame_from_poses(0, "unittest", pose_o, pose_x, pose_y)

                with self.assertRaises(TcpCommandException):
                    self.niryo_robot.save_dynamic_frame_from_points(0, "unittest", pose_o, pose_x, pose_y)

                with self.assertRaises(TcpCommandException):
                    self.niryo_robot.edit_dynamic_frame("unitTestFramePose_000", 0, 0)

            else:
                # Test creation by points
                new_name = 'unitTestFramePose_{:03d}'.format(i)
                new_desc = 'descTestFramePose_{:03d}'.format(i)
                point_o = self.robot_point[2 - i][0]
                point_x = self.robot_point[2 - i][1]
                point_y = self.robot_point[2 - i][2]
                self.assertIsNone(
                    self.niryo_robot.save_dynamic_frame_from_points(new_name, new_desc, point_o, point_x, point_y))

                # Test edition
                new_edit_name = 'unitEditTestFramePose_{:03d}'.format(i)
                new_edit_desc = 'descEditTestFramePose_{:03d}'.format(i)
                self.assertIsNone(self.niryo_robot.edit_dynamic_frame(new_name, new_edit_name, new_edit_desc))
                self.assertEqual(self.niryo_robot.get_saved_dynamic_frame(new_edit_name)[0], new_edit_name)

                if new_edit_name not in new_list_name:
                    new_list_name.append(new_edit_name)
                    list_saved.append(new_edit_name)

                new_list_name.sort()

                self.assertEqual(self.niryo_robot.get_saved_dynamic_frame_list()[0], new_list_name)

    def __test_move_in_frame(self):
        # Move frame 000
        pose0 = (0, 0, 0, 0, 1.57, 0)
        self.assertIsNone(self.niryo_robot.move_pose(pose0, "unitEditTestFramePose_000"))
        self.assertIsNone(self.niryo_robot.move_linear_pose((0.05, 0.05, 0.05, 0, 1.57, 0),
                                                            "unitEditTestFramePose_000"))

        # Move frame 001
        pose1 = PoseObject(0, 0, 0, 0, 1.57, 0, metadata=PoseMetadata.v1())
        self.assertIsNone(self.niryo_robot.move_pose(pose1, "unitEditTestFramePose_001"))
        self.assertIsNone(self.niryo_robot.move_linear_pose((0.05, 0.05, 0.05, 0, 1.57, 0),
                                                            "unitEditTestFramePose_001"))

        # Move frame 002
        pose2 = (0, 0, 0, 0, 1.57, 0)
        self.assertIsNone(self.niryo_robot.move_pose(pose2, "unitEditTestFramePose_002"))
        self.assertIsNone(self.niryo_robot.move_relative([0.05, 0.05, 0.05, 0.1, 0.1, 0.1],
                                                         "unitEditTestFramePose_002"))
        self.assertIsNone(
            self.niryo_robot.move_linear_relative([-0.05, -0.05, -0.05, 0, 0, 0], "unitEditTestFramePose_002"))

        # Move frame 003
        pose3 = PoseObject(0, 0, 0, 0, 1.57, 0, metadata=PoseMetadata.v1())
        self.assertIsNone(self.niryo_robot.move_pose(pose3, "unitEditTestFramePose_003"))
        self.assertIsNone(self.niryo_robot.move_relative([0.05, 0.05, 0.05, 0.1, 0.1, 0.1],
                                                         "unitEditTestFramePose_003"))
        self.assertIsNone(
            self.niryo_robot.move_linear_relative([-0.05, -0.05, -0.05, 0, 0, 0], "unitEditTestFramePose_003"))

        # Test default world frame
        self.assertIsNone(self.niryo_robot.move_relative([0.1, 0.1, 0.1, 0, 0, 0]))
        self.assertIsNone(self.niryo_robot.move_linear_relative([0, 0, -0.1, 0, 0, 0]))

        with self.assertRaises(TcpCommandException):
            self.niryo_robot.move_relative([0.05, 0.05, 0.05, 0.1, 0.1, 0.1], 0)

        with self.assertRaises(TcpCommandException):
            self.niryo_robot.move_linear_relative([0.05, 0.05, 0.05, 0.1, 0.1, 0.1], 0)

    def __test_deletion(self):
        base_list = self.niryo_robot.get_saved_dynamic_frame_list()[0]
        new_list = [frame for frame in base_list]

        for i in range(4):
            name_delete = 'unitEditTestFramePose_{:03d}'.format(i)
            self.assertIsNone(self.niryo_robot.delete_dynamic_frame(name_delete))

            new_list.remove(name_delete)

            self.assertEqual(self.niryo_robot.get_saved_dynamic_frame_list()[0], new_list)


# noinspection PyTypeChecker
class TestTools(BaseTestTcpApi):

    @classmethod
    def setUpClass(cls):
        cls.niryo_robot = NiryoRobot(robot_ip_address)

    @classmethod
    def tearDownClass(cls):
        cls.niryo_robot.close_connection()

    def setUp(self):
        pass

    def tearDown(self):
        pass

    # noinspection PyTypeChecker
    def test_select(self):
        # Set tool used and check
        self.assertIsNone(self.niryo_robot.update_tool())
        self.assertEqual(tool_used, self.niryo_robot.tool)
        self.assertEqual(tool_used, self.niryo_robot.get_current_tool_id())
        self.assertNotEqual(self.niryo_robot.get_current_tool_id(), tool_used.value)
        self.assertNotEqual(self.niryo_robot.get_current_tool_id(), ToolID.NONE)

    def test_use_tool(self):
        # Equip tool
        self.assertIsNone(self.niryo_robot.update_tool())

        # Grasp/Release with ID
        self.assertIsNone(self.niryo_robot.grasp_with_tool())
        self.assertIsNone(self.niryo_robot.release_with_tool())

        # Grasp/Release without ID
        self.assertIsNone(self.niryo_robot.grasp_with_tool())
        self.assertIsNone(self.niryo_robot.release_with_tool())

        # Grippers specific functions
        if tool_used in [ToolID.GRIPPER_1, ToolID.GRIPPER_2, ToolID.GRIPPER_3]:
            self.assertIsNone(self.niryo_robot.close_gripper())
            self.assertIsNone(self.niryo_robot.open_gripper())
            self.assertIsNone(self.niryo_robot.open_gripper(speed=500))
            self.assertIsNone(self.niryo_robot.close_gripper(speed=500))
            self.assertIsNone(self.niryo_robot.open_gripper(max_torque_percentage=100, hold_torque_percentage=50))
            self.assertIsNone(self.niryo_robot.close_gripper(max_torque_percentage=100, hold_torque_percentage=50))

    def test_electromagnet(self):
        # Equip tool
        self.assertIsNone(self.niryo_robot.setup_electromagnet(PinID.GPIO_1A))
        self.assertIsNone(self.niryo_robot.setup_electromagnet("1B"))

        # Grasp/Release without ID
        self.assertIsNone(self.niryo_robot.grasp_with_tool())
        self.assertIsNone(self.niryo_robot.release_with_tool())

        # Grasp/Release with ID
        self.assertIsNone(self.niryo_robot.activate_electromagnet(PinID.GPIO_1B))
        self.assertIsNone(self.niryo_robot.deactivate_electromagnet("1B"))


# noinspection PyTypeChecker
class TestIOs(BaseTestTcpApi):

    @classmethod
    def setUpClass(cls):
        cls.niryo_robot = NiryoRobot(robot_ip_address)

    @classmethod
    def tearDownClass(cls):
        cls.niryo_robot.close_connection()

    def setUp(self):
        pass

    def tearDown(self):
        pass

    def test_digital_ios(self):
        self.assertIsInstance(self.niryo_robot.get_digital_io_state(), list)
        self.assertIsInstance(self.niryo_robot.get_digital_io_state()[0], DigitalPinObject)
        self.assertIsInstance(self.niryo_robot.get_digital_io_state()[0].pin_id, PinID)
        self.assertIsInstance(self.niryo_robot.get_digital_io_state()[0].name, str)
        self.assertIsInstance(self.niryo_robot.get_digital_io_state()[0].mode, PinMode)
        self.assertIsInstance(self.niryo_robot.get_digital_io_state()[0].state, PinState)

        for index, pin_object in enumerate(self.niryo_robot.get_digital_io_state()):
            if pin_object.name.startswith('DI'):
                continue

            if not (pin_object.name.startswith('SW') or pin_object.name.startswith('DO')):
                self.assertIsNone(self.niryo_robot.set_pin_mode(pin_object.name, PinMode.OUTPUT))
                self.niryo_robot.wait(1)

            self.assertEqual(self.niryo_robot.get_digital_io_state()[index].mode, PinMode.OUTPUT)

            self.assertIsNone(self.niryo_robot.digital_write(pin_object.pin_id, PinState.HIGH))
            self.assertEqual(self.niryo_robot.digital_read(pin_object.pin_id), PinState.HIGH)
            self.assertEqual(self.niryo_robot.get_digital_io_state()[index].state, PinState.HIGH)
            self.assertIsNone(self.niryo_robot.digital_write(pin_object.name, PinState.LOW))
            self.assertEqual(self.niryo_robot.digital_read(pin_object.name), PinState.LOW)
            self.assertEqual(self.niryo_robot.get_digital_io_state()[index].state, PinState.LOW)

            if not (pin_object.name.startswith('SW') or pin_object.name.startswith('DO')):
                self.assertIsNone(self.niryo_robot.set_pin_mode(pin_object.name, PinMode.INPUT))
                self.assertEqual(self.niryo_robot.get_digital_io_state()[index].mode, PinMode.INPUT)

                # with self.assertRaises(NiryoRobotException):
                #    self.niryo_robot.digital_write(pin, PinState.LOW)

    def test_analog_ios(self):
        self.assertIsInstance(self.niryo_robot.get_analog_io_state(), list)
        self.assertIsInstance(self.niryo_robot.get_analog_io_state()[0], AnalogPinObject)
        self.assertIsInstance(self.niryo_robot.get_analog_io_state()[0].pin_id, PinID)
        self.assertIsInstance(self.niryo_robot.get_analog_io_state()[0].name, str)
        self.assertIsInstance(self.niryo_robot.get_analog_io_state()[0].mode, PinMode)
        self.assertIsInstance(self.niryo_robot.get_analog_io_state()[0].value, (float, int))

        for index, pin_object in enumerate(self.niryo_robot.get_analog_io_state()):
            if pin_object.name.startswith('AI'):
                self.assertEqual(self.niryo_robot.get_analog_io_state()[index].mode, PinMode.INPUT)
            else:
                self.assertEqual(self.niryo_robot.get_analog_io_state()[index].mode, PinMode.OUTPUT)

                self.assertIsNone(self.niryo_robot.analog_write(pin_object.pin_id, 5.0))
                self.assertEqual(self.niryo_robot.analog_read(pin_object.pin_id), 5.0)
                self.assertEqual(self.niryo_robot.get_analog_io_state()[index].value, 5.0)

                self.assertIsNone(self.niryo_robot.analog_write(pin_object.pin_id, 2.5))
                self.assertEqual(self.niryo_robot.analog_read(pin_object.pin_id), 2.5)
                self.assertEqual(self.niryo_robot.get_analog_io_state()[index].value, 2.5)

                self.assertIsNone(self.niryo_robot.analog_write(pin_object.pin_id, 0))
                self.assertEqual(self.niryo_robot.analog_read(pin_object.pin_id), 0)
                self.assertEqual(self.niryo_robot.get_analog_io_state()[index].value, 0)

    def test_button(self):
        self.assertIsInstance(self.niryo_robot.get_custom_button_state(), str)


@unittest.skipUnless(simulation, "Vision test is only coded for Gazebo")
class TestVision(BaseTestTcpApi):
    workspace_name = "gazebo_1"
    workspace_h = 0.001
    point_1 = [0.3369, 0.087, workspace_h]
    point_2 = [point_1[0], -point_1[1], workspace_h]
    point_3 = [0.163, -point_1[1], workspace_h]
    point_4 = [point_3[0], point_1[1], workspace_h]

    def setUp(self):
        super(TestVision, self).setUp()
        self.assertIsNone(self.niryo_robot.move_joints(0.0, 0.0, 0.0, 0.0, -1.57, 0.0))
        self.assertIsNone(self.niryo_robot.update_tool())
        self.assertIsNone(
            self.niryo_robot.save_workspace_from_points(self.workspace_name,
                                                        self.point_1,
                                                        self.point_2,
                                                        self.point_3,
                                                        self.point_4))

    def tearDown(self):
        self.assertIsNone(self.niryo_robot.delete_workspace(self.workspace_name))
        super(TestVision, self).tearDown()

    def test_vision_detect(self):
        # Getting img compressed & calibration object
        self.assertIsNotNone(self.niryo_robot.get_img_compressed())
        self.assertIsNotNone(self.niryo_robot.get_camera_intrinsics())

        # Getting target pose's from multiple ways
        self.assertIsNotNone(self.niryo_robot.get_target_pose_from_rel(self.workspace_name, 0.1, 0.5, 0.5, 0.0))

        self.assertIsNotNone(
            self.niryo_robot.get_target_pose_from_cam(self.workspace_name, 0.1, ObjectShape.ANY, ObjectColor.ANY))

        self.assertIsNotNone(self.niryo_robot.detect_object(self.workspace_name, ObjectShape.ANY, ObjectColor.RED))

    def test_vision_move(self):
        # Test to move to the object
        self.assertIsNotNone(
            self.niryo_robot.move_to_object(self.workspace_name, 0.1, ObjectShape.ANY, ObjectColor.GREEN))
        # Going back to observation pose
        self.assertIsNone(self.niryo_robot.move_joints(0.0, 0.0, 0.0, 0.0, -1.57, 0.0))
        # Vision Pick
        self.assertIsNotNone(self.niryo_robot.vision_pick(self.workspace_name, 0.1, ObjectShape.ANY, ObjectColor.BLUE))


class TestWorkspaceMethods(BaseTestTcpApi):
    robot_poses = [[0.3, 0.1, 0.1, 0., 1.57, 0.], [0.3, -0.1, 0.1, 0., 1.57, 0.], [0.1, -0.1, 0.1, 0., 1.57, 0.],
                   [0.1, 0.1, 0.1, 0., 1.57, 0.]]
    points = [[p[0], p[1], p[2] + 0.05] for p in robot_poses]
    robot_poses_obj = [PoseObject(*pose, metadata=PoseMetadata.v1()) for pose in robot_poses]

    def test_creation_delete_workspace(self):
        # Get saved workspace list & copy it
        base_list = self.niryo_robot.get_workspace_list()
        new_list = [v for v in base_list]

        # Create new trajectories
        list_names_saved = []
        for i in range(6):
            new_name = 'test_{:03d}'.format(i)
            if i < 2:
                self.assertIsNone(self.niryo_robot.save_workspace_from_robot_poses(new_name, *self.robot_poses_obj))
            elif i < 4:
                self.assertIsNone(self.niryo_robot.save_workspace_from_robot_poses(new_name, *self.robot_poses))
            else:
                self.assertIsNone(self.niryo_robot.save_workspace_from_points(new_name, *self.points))

            # Checking ratio
            self.assertAlmostEquals(self.niryo_robot.get_workspace_ratio(new_name), 1.0, places=2)

            if new_name not in new_list:
                new_list.append(new_name)
                list_names_saved.append(new_name)
            # Checking workspace has been appended to the list
            self.assertEqual(self.niryo_robot.get_workspace_list(), new_list)

        # Delete created workspaces
        for name in list_names_saved:
            self.assertIsNone(self.niryo_robot.delete_workspace(name))
            new_list.pop(new_list.index(name))
            self.assertEqual(self.niryo_robot.get_workspace_list(), new_list)


class TestSound(BaseTestTcpApi):

    def test_sons(self):
        self.assertIsNotNone(self.niryo_robot.get_sounds())
        self.assertIsInstance(self.niryo_robot.get_sounds(), list)

        sound_name = self.niryo_robot.get_sounds()[0]

        self.assertGreater(self.niryo_robot.get_sound_duration(sound_name), 0)
        sound_duration = self.niryo_robot.get_sound_duration(sound_name)

        self.assertIsNone(self.niryo_robot.set_volume(200))
        self.assertIsNone(self.niryo_robot.set_volume(100))

        self.assertIsNone(self.niryo_robot.play_sound(sound_name, True, 0.1, sound_duration - 0.1))

        self.assertIsNone(self.niryo_robot.play_sound(sound_name, False))
        self.niryo_robot.wait(0.1)
        self.assertIsNone(self.niryo_robot.stop_sound())
        self.assertIsNone(self.niryo_robot.stop_sound())

        self.assertIsNone(self.niryo_robot.say("Test", 0))

        sound_list = self.niryo_robot.get_sounds()
        sound_name_test = "unittest.mp3"
        while sound_name_test + ".mp3" in sound_list:
            sound_name_test += "0"
        sound_name_test += ".mp3"

        with self.assertRaises(TcpCommandException):
            self.niryo_robot.get_sound_duration(sound_name_test)

        with self.assertRaises(TcpCommandException):
            self.niryo_robot.play_sound(sound_name_test, True)

        with self.assertRaises(NiryoRobotException):
            self.niryo_robot.say("Test", -1)

        with self.assertRaises(TcpCommandException):
            self.niryo_robot.set_volume(-100)

        with self.assertRaises(TcpCommandException):
            self.niryo_robot.set_volume(201)


class TestLedRing(BaseTestTcpApi):

    def test_ledring(self):

        def check_delay(function, **kwargs):
            start_time = time.time()
            self.assertIsNotNone(function(**kwargs))
            self.assertGreaterEqual(time.time(), start_time + kwargs['period'] * kwargs['iterations'])

        self.assertIsNotNone(self.niryo_robot.set_led_color(1, [0, 255, 255]))
        self.assertIsNotNone(self.niryo_robot.led_ring_solid([255, 0, 255]))
        self.assertIsNotNone(
            self.niryo_robot.led_ring_custom(led_colors=[[i / 30. * 255, 0, 255 - i / 30.] for i in range(30)]))

        check_delay(self.niryo_robot.led_ring_flashing, color=[255, 255, 0], period=0.5, iterations=5, wait=True)

        check_delay(self.niryo_robot.led_ring_alternate,
                    color_list=[[0, 255, 255], [255, 0, 255]],
                    period=0.5,
                    iterations=4,
                    wait=True)

        check_delay(self.niryo_robot.led_ring_chase, color=[255, 255, 0], period=1, iterations=2, wait=True)
        check_delay(self.niryo_robot.led_ring_go_up, color=[0, 255, 255], period=0.5, iterations=4, wait=True)
        check_delay(self.niryo_robot.led_ring_go_up_down, color=[255, 0, 255], period=0.5, iterations=4, wait=True)
        check_delay(self.niryo_robot.led_ring_breath, color=[255, 255, 0], period=2, iterations=2, wait=True)
        check_delay(self.niryo_robot.led_ring_snake, color=[0, 255, 255], period=0.5, iterations=4, wait=True)

        check_delay(self.niryo_robot.led_ring_rainbow, period=3, iterations=2, wait=True)
        check_delay(self.niryo_robot.led_ring_rainbow_cycle, period=3, iterations=2, wait=True)
        check_delay(self.niryo_robot.led_ring_rainbow_chase, period=5, iterations=1, wait=True)

        self.assertIsNotNone(self.niryo_robot.led_ring_turn_off())


if __name__ == '__main__':
    unittest.main()
