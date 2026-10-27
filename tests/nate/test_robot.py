import unittest
import xml.etree.ElementTree as ET

from unittest.mock import MagicMock, patch
from uuid import uuid4

from pyniryo.nate.models import robot, geometry, motion
from pyniryo.nate.models.robot import TrajectoryExecutorStatus
from pyniryo.nate._internal import transport_models, paths_gen, topics_gen
from pyniryo.nate.components.robot import Robot, MoveCommand
from pyniryo.nate.exceptions import ClientError, GenerateTrajectoryError, TransportError

from .base import BaseTestComponent


class TestRobot(BaseTestComponent):

    def setUp(self):
        super().setUp()
        self.robot = Robot(http_client=self.http_client,
                           mqtt_client=self.mqtt_client,
                           correlation_id=self.correlation_id)

    def tearDown(self):
        del self.robot

    def test_get_joints(self):
        """Test getting current joint positions."""
        joint_values = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
        self.http_client.get.return_value = transport_models.s.Joints(root=joint_values)

        joints = self.robot.get_joints()

        self.http_client.get.assert_called_once_with(paths_gen.Robot.GET_ROBOT_JOINTS, transport_models.s.Joints)
        self.assertIsInstance(joints, geometry.Joints)
        self.assertEqual(joints.data, joint_values)

    def test_on_joints(self):
        """Test subscribing to joint updates."""
        callback = MagicMock()

        self.robot.on_joints(callback)

        self.mqtt_client.subscribe.assert_called_once()
        topic, internal_callback, model = self.mqtt_client.subscribe.call_args[0]
        self.assertEqual(topic, topics_gen.Robot.JOINTS)
        self.assertEqual(model, transport_models.a.Joints)

        # Simulate receiving joints
        joint_values = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
        internal_callback(topic, transport_models.a.Joints(positions=joint_values, velocities=[0.0] * 6, timestamp=0))

        callback.assert_called_once()
        called_joints = callback.call_args[0][0]
        self.assertIsInstance(called_joints, geometry.Joints)
        self.assertEqual(called_joints.data, joint_values)

    def test_get_all_frames(self):
        """Test getting all frame IDs."""
        frame_ids = ['base_link', 'tcp', 'world']
        self.http_client.get.return_value = transport_models.FrameIdList(root=frame_ids)

        frames = self.robot.get_all_frames()

        self.http_client.get.assert_called_once_with(paths_gen.Robot.GET_ALL_FRAMES, transport_models.FrameIdList)
        self.assertEqual(frames, frame_ids)

    def test_get_frame_pose(self):
        """Test getting a specific frame's pose."""
        frame_id = 'tcp'
        mock_pose = transport_models.s.Pose(position=transport_models.s.Point(x=0.1, y=0.2, z=0.3),
                                            orientation=transport_models.s.Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
        self.http_client.get.return_value = mock_pose

        pose = self.robot.get_frame_pose(frame_id)

        self.http_client.get.assert_called_once_with(paths_gen.Robot.GET_FRAME_POSE.format(frame_id=frame_id),
                                                     transport_models.s.Pose)
        self.assertIsInstance(pose, geometry.Pose)
        self.assertEqual(pose.position.x, 0.1)

    def test_on_frame_pose(self):
        """Test subscribing to frame pose updates."""
        frame_id = 'tcp'
        callback = MagicMock()

        self.robot.on_frame_pose(frame_id, callback)

        self.mqtt_client.subscribe.assert_called_once()
        topic, internal_callback, model = self.mqtt_client.subscribe.call_args[0]
        self.assertEqual(topic, self.mqtt_client.format(topics_gen.Robot.FRAME_POSE, frame_id=frame_id))
        self.assertEqual(model, transport_models.a.Pose)

        # Simulate receiving pose
        mock_pose = transport_models.s.Pose(position=transport_models.s.Point(x=0.1, y=0.2, z=0.3),
                                            orientation=transport_models.s.Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
        internal_callback(topic, mock_pose)

        callback.assert_called_once()
        called_pose = callback.call_args[0][0]
        self.assertIsInstance(called_pose, geometry.Pose)

    def test_set_tcp_frame(self):
        """Test setting a TCP frame."""
        frame_id = 'my_tcp'
        pose = geometry.Pose(position=geometry.Point(x=0.1, y=0.0, z=0.0),
                             orientation=geometry.Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
        self.http_client.post.return_value = transport_models.EmptyPayload()

        self.robot.set_tcp_frame(frame_id, pose)

        self.http_client.post.assert_called_once()
        call_args = self.http_client.post.call_args[0]
        self.assertEqual(call_args[0], paths_gen.Robot.CREATE_FRAME.format(frame_id=frame_id))

    def test_move_with_joints(self):
        """Test moving with joint target."""
        command_id = str(uuid4())
        joints = geometry.Joints(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self.http_client.post.return_value = transport_models.s.FeedbackResponse(feedback_id=command_id)

        move_cmd = self.robot.move(joints)

        self.http_client.post.assert_called_once()
        call_args = self.http_client.post.call_args[0]
        self.assertEqual(call_args[0], paths_gen.Robot.MOVE_ALONG_WAYPOINTS)

        request = call_args[2]
        self.assertEqual(str(request.command_id), move_cmd.command_id)
        self.assertFalse(request.add_start)
        self.assertEqual(len(request.waypoints), 1)

        self.assertIsInstance(move_cmd, MoveCommand)

    def test_move_with_joints_add_start(self):
        """Test moving with joint target and add_start=True."""
        command_id = str(uuid4())
        joints = geometry.Joints(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self.http_client.post.return_value = transport_models.s.FeedbackResponse(feedback_id=command_id)

        _ = self.robot.move(joints, add_start=True)

        request = self.http_client.post.call_args[0][2]
        self.assertTrue(request.add_start)

    def test_move_with_pose(self):
        """Test moving with pose target."""
        command_id = str(uuid4())
        pose = geometry.Pose(position=geometry.Point(x=0.1, y=0.2, z=0.3),
                             orientation=geometry.Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
        self.http_client.post.return_value = transport_models.s.FeedbackResponse(feedback_id=command_id)

        move_cmd = self.robot.move(pose)

        request = self.http_client.post.call_args[0][2]
        self.assertEqual(len(request.waypoints), 1)
        self.assertIsNotNone(request.waypoints[0].pose)

    def test_move_with_waypoint(self):
        """Test moving with single waypoint target."""
        command_id = str(uuid4())
        waypoint = motion.Waypoint(joints=geometry.Joints(0.1, 0.2, 0.3, 0.4, 0.5, 0.6), velocity_factor=0.5)
        self.http_client.post.return_value = transport_models.s.FeedbackResponse(feedback_id=command_id)

        move_cmd = self.robot.move(waypoint)

        request = self.http_client.post.call_args[0][2]
        self.assertEqual(len(request.waypoints), 1)
        self.assertEqual(request.waypoints[0].velocity_factor, 0.5)

    def test_move_with_waypoints(self):
        """Test moving with multiple waypoints."""
        command_id = str(uuid4())
        waypoints = [
            motion.Waypoint(joints=geometry.Joints(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)),
            motion.Waypoint(joints=geometry.Joints(0.5, 0.5, 0.5, 0.5, 0.5, 0.5)),
        ]
        self.http_client.post.return_value = transport_models.s.FeedbackResponse(feedback_id=command_id)

        _ = self.robot.move(waypoints)

        request = self.http_client.post.call_args[0][2]
        self.assertEqual(len(request.waypoints), 2)

    def test_move_post_failure_leaves_no_command_behind(self):
        """A move whose request never got through must not stay queued nor keep listening for feedback."""
        unsubscribe = MagicMock()
        self.mqtt_client.subscribe.return_value = unsubscribe
        self.http_client.post.side_effect = TransportError('POST', paths_gen.Robot.MOVE_ALONG_WAYPOINTS,
                                                           'Read timed out')

        with self.assertRaises(TransportError):
            self.robot.move(geometry.Joints(0.0, 0.0, 0.0, 0.0, 0.0, 0.0))

        self.assertEqual([], self.robot.get_commands())
        unsubscribe.assert_called_once()

    def test_move_with_invalid_type(self):
        """Test moving with invalid target type."""
        with self.assertRaises(TypeError):
            self.robot.move("invalid")

    def test_move_joints_high_level(self):
        """Test high-level move_joints method."""
        command_id = str(uuid4())
        joints = geometry.Joints(0.1, 0.2, 0.3, 0.4, 0.5, 0.6)
        self.http_client.post.return_value = transport_models.s.FeedbackResponse(feedback_id=command_id)

        move_cmd = self.robot.move_joints(joints)

        self.http_client.post.assert_called_once()
        request = self.http_client.post.call_args[0][2]
        # move_joints always uses add_start=True
        self.assertTrue(request.add_start)
        self.assertIsInstance(move_cmd, MoveCommand)

    def test_move_joints_high_level_with_floats(self):
        """Test high-level move_joints method with individual floats."""
        command_id = str(uuid4())
        self.http_client.post.return_value = transport_models.s.FeedbackResponse(feedback_id=command_id)

        move_cmd = self.robot.move_joints(0.1, 0.2, 0.3, 0.4, 0.5, 0.6)

        request = self.http_client.post.call_args[0][2]
        self.assertEqual(len(request.waypoints), 1)

    def test_move_pose_high_level(self):
        """Test high-level move_pose method with a Pose object."""
        command_id = str(uuid4())
        pose = geometry.Pose(position=geometry.Point(x=0.1, y=0.2, z=0.3),
                             orientation=geometry.Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
        self.http_client.post.return_value = transport_models.s.FeedbackResponse(feedback_id=command_id)

        move_cmd = self.robot.move_pose(pose)

        self.http_client.post.assert_called_once()
        request = self.http_client.post.call_args[0][2]
        # move_pose always uses add_start=True
        self.assertTrue(request.add_start)
        self.assertIsInstance(move_cmd, MoveCommand)

    def test_execute_trajectory(self):
        """Test executing a trajectory."""
        command_id = str(uuid4())
        joints_stamped = motion.JointsStamped(joints=geometry.Joints(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                                              timestamp=0.0,
                                              velocities=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                                              accelerations=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        trajectory = [joints_stamped]
        self.http_client.post.return_value = transport_models.s.FeedbackResponse(feedback_id=command_id)

        move_cmd = self.robot.execute_trajectory(trajectory)

        self.http_client.post.assert_called_once()
        call_args = self.http_client.post.call_args[0]
        self.assertEqual(call_args[0], paths_gen.Robot.EXECUTE_TRAJECTORY)

        request = call_args[2]
        self.assertEqual(str(request.command_id), move_cmd.command_id)
        self.assertIsInstance(move_cmd, MoveCommand)

    def test_move_refused_in_automatic_mode(self):
        """Test that the refusal of the robot reaches the caller of a move."""
        self.http_client.post.side_effect = ClientError(
            409, '{"error": "direct control requires the robot to be in manual mode"}')

        with self.assertRaises(ClientError) as refusal:
            self.robot.move(geometry.Joints(0.0, 0.0, 0.0, 0.0, 0.0, 0.0))

        self.assertIn('manual mode', str(refusal.exception))

    def test_get_commands(self):
        """Test getting the command queue."""
        command_id = str(uuid4())
        joints = geometry.Joints(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self.http_client.post.return_value = transport_models.s.FeedbackResponse(feedback_id=command_id)

        cmd = self.robot.move(joints)
        commands = self.robot.get_commands()

        self.assertIn(cmd, commands)

    def test_get_urdf(self):
        """Test getting URDF."""
        urdf_content = b'<?xml version="1.0"?><robot name="test"><link name="base"/></robot>'

        def mock_download(path, buffer):
            buffer.write(urdf_content)

        self.http_client.download.side_effect = mock_download

        urdf = self.robot.get_urdf()

        self.http_client.download.assert_called_once()
        self.assertIsInstance(urdf, ET.Element)
        self.assertEqual(urdf.tag, 'robot')
        self.assertEqual(urdf.get('name'), 'test')

    def test_get_configuration(self):
        """Test getting robot configuration."""
        mock_config = transport_models.s.RobotConfig(
            name='test_robot',
            model='test_model',
            tool_model='test_tool',
            hardware_mode='mock',
            num_joints=6,
            joints=[
                transport_models.s.JointConfig(name=f'joint_{i + 1}',
                                               type='revolute',
                                               position_limit_min=-3.14,
                                               position_limit_max=3.14,
                                               velocity_limit=1.0,
                                               acceleration_limit=1.0,
                                               effort_limit=1.0) for i in range(6)
            ],
        )
        self.http_client.get.return_value = mock_config

        config = self.robot.get_configuration()

        self.http_client.get.assert_called_once_with(paths_gen.Robot.GET_ROBOT_CONFIG, transport_models.s.RobotConfig)
        self.assertIsInstance(config, robot.RobotConfiguration)
        self.assertEqual(config.name, mock_config.name)
        self.assertEqual(config.model, mock_config.model)
        self.assertEqual(config.tool_model, mock_config.tool_model)
        self.assertEqual(config.hardware_mode, mock_config.hardware_mode)
        self.assertEqual(config.n_joint, mock_config.num_joints)
        self.assertEqual(len(config.joints), mock_config.num_joints)

        for joint_config, mock_joint in zip(config.joints, mock_config.joints):
            self.assertEqual(joint_config.name, mock_joint.name)
            self.assertEqual(joint_config.type, mock_joint.type)
            self.assertEqual(joint_config.position_limit_min, mock_joint.position_limit_min)
            self.assertEqual(joint_config.position_limit_max, mock_joint.position_limit_max)
            self.assertEqual(joint_config.velocity_limit, mock_joint.velocity_limit)
            self.assertEqual(joint_config.acceleration_limit, mock_joint.acceleration_limit)
            self.assertEqual(joint_config.effort_limit, mock_joint.effort_limit)

    def test_get_control_mode(self):
        """Test getting control mode."""
        mock_mode = transport_models.s.ControlMode(mode_name=transport_models.s.ModeName.TRAJECTORY,
                                                   mode=transport_models.s.Mode(1))
        self.http_client.get.return_value = mock_mode

        mode = self.robot.get_control_mode()

        self.http_client.get.assert_called_once_with(paths_gen.Robot.GET_ROBOT_CONTROL_MODE,
                                                     transport_models.s.ControlMode)
        self.assertEqual(mode, robot.ControlMode.TRAJECTORY)

    def test_set_control_mode(self):
        """Test setting control mode successfully."""
        target_mode = robot.ControlMode.JOG
        mock_response = transport_models.s.ControlMode(mode_name=transport_models.s.ModeName.JOG,
                                                       mode=transport_models.s.Mode(2))
        self.http_client.put.return_value = mock_response

        self.robot.set_control_mode(target_mode)

        self.http_client.put.assert_called_once()
        call_args = self.http_client.put.call_args[0]
        self.assertEqual(call_args[0], paths_gen.Robot.SET_ROBOT_CONTROL_MODE)
        self.assertEqual(self.http_client.put.call_args.kwargs['exclude'], {'mode_name'})

    def test_set_control_mode_failure(self):
        """Test setting control mode when it fails to set."""
        target_mode = robot.ControlMode.JOG
        # Return a different mode than requested → should raise RuntimeError
        mock_response = transport_models.s.ControlMode(mode_name=transport_models.s.ModeName.TRAJECTORY,
                                                       mode=transport_models.s.Mode(1))
        self.http_client.put.return_value = mock_response

        with self.assertRaises(RuntimeError):
            self.robot.set_control_mode(target_mode)

    def test_jog_mode_context_manager(self):
        """Test jog mode context manager."""
        initial_mode = robot.ControlMode.TRAJECTORY.to_transport_model()
        jog_mode = robot.ControlMode.JOG.to_transport_model()

        self.http_client.get.return_value = initial_mode
        self.http_client.put.return_value = jog_mode

        with self.robot.jog_mode():
            self.http_client.put.return_value = initial_mode

        # Should call put twice: once to set jog, once to restore
        self.assertEqual(self.http_client.put.call_count, 2)

        # First call should set to jog (mode value 2)
        first_call = self.http_client.put.call_args_list[0][0][2]
        self.assertEqual(first_call.mode.value, 2)

    def test_jog_mode_context_manager_with_exception(self):
        """Test jog mode context manager restores mode even after exception."""
        initial_mode = robot.ControlMode.TRAJECTORY.to_transport_model()
        jog_mode = robot.ControlMode.JOG.to_transport_model()

        self.http_client.get.return_value = initial_mode
        self.http_client.put.return_value = jog_mode

        try:
            with self.robot.jog_mode():
                self.http_client.put.return_value = initial_mode
                raise ValueError("Test exception")
        except ValueError:
            pass

        # Should still restore mode even after exception
        self.assertEqual(self.http_client.put.call_count, 2)

    def test_stop_jog(self):
        """Test stopping jog command."""
        self.robot.stop_jog()

        self.mqtt_client.publish.assert_called_once()
        topic, payload = self.mqtt_client.publish.call_args[0]
        self.assertEqual(topic, topics_gen.Cmd.ROBOT_JOG_STOP)
        self.assertIsInstance(payload, transport_models.EmptyPayload)

    def test_jog_joints(self):
        """Test jogging with joint velocities."""
        target_joints = geometry.Joints(0.1, 0.2, 0.3, 0.4, 0.5, 0.6)

        self.robot.jog_joints(velocity=target_joints)

        self.mqtt_client.publish.assert_called_once()
        topic, payload = self.mqtt_client.publish.call_args[0]
        self.assertEqual(topic, topics_gen.Cmd.ROBOT_JOG_JOINT)
        self.assertIsInstance(payload, transport_models.a.JogJoint)
        self.assertEqual(payload.velocities, target_joints.data)
        self.assertIsNone(payload.displacements)

    def test_jog_joints_increment(self):
        increment = geometry.Joints(0.0, 0.0, 0.1, 0.0, 0.0, 0.0)

        self.robot.jog_joints_increment(increment)

        self.mqtt_client.publish.assert_called_once()
        topic, payload = self.mqtt_client.publish.call_args[0]
        self.assertEqual(topic, topics_gen.Cmd.ROBOT_JOG_JOINT)
        self.assertIsInstance(payload, transport_models.a.JogJoint)
        self.assertEqual(payload.displacements, increment.data)
        self.assertIsNone(payload.velocities)

    def test_jog_cartesian_without_frame(self):
        """Test jogging in Cartesian space without frame."""
        linear = [0.1, 0.0, 0.0]
        angular = [0.0, 0.0, 0.1]

        self.robot.jog_cartesian(linear, angular)

        self.mqtt_client.publish.assert_called_once()
        topic, payload = self.mqtt_client.publish.call_args[0]
        self.assertEqual(topic, topics_gen.Cmd.ROBOT_JOG_CARTESIAN)
        self.assertIsInstance(payload, transport_models.a.JogCartesian)
        self.assertEqual(payload.linear, linear)
        self.assertEqual(payload.angular, angular)
        self.assertIsNone(payload.frame_id)
        self.assertFalse(payload.is_increment)

    def test_jog_cartesian_with_frame(self):
        """Test jogging in Cartesian space with frame."""
        linear = [0.1, 0.0, 0.0]
        angular = [0.0, 0.0, 0.1]
        frame_id = 'tcp'

        self.robot.jog_cartesian(linear, angular, frame_id)

        payload = self.mqtt_client.publish.call_args[0][1]
        self.assertEqual(payload.frame_id, frame_id)

    def test_jog_cartesian_increment_without_frame(self):
        linear = [0.01, 0.0, 0.0]
        angular = [0.0, 0.0, 0.05]

        self.robot.jog_cartesian_increment(linear_increment=linear, angular_increment=angular)

        self.mqtt_client.publish.assert_called_once()
        topic, payload = self.mqtt_client.publish.call_args[0]
        self.assertEqual(topic, topics_gen.Cmd.ROBOT_JOG_CARTESIAN)
        self.assertIsInstance(payload, transport_models.a.JogCartesian)
        self.assertEqual(payload.linear, linear)
        self.assertEqual(payload.angular, angular)
        self.assertIsNone(payload.frame_id)
        self.assertTrue(payload.is_increment)

    def test_jog_cartesian_increment_with_frame(self):
        self.robot.jog_cartesian_increment(linear_increment=[0.01, 0.0, 0.0],
                                           angular_increment=[0.0, 0.0, 0.0],
                                           frame_id='base_link')

        payload = self.mqtt_client.publish.call_args[0][1]
        self.assertEqual(payload.frame_id, 'base_link')
        self.assertTrue(payload.is_increment)

    def test_executor_status(self):
        """Test getting executor status."""
        self.http_client.get.return_value = transport_models.s.TrajectoryExecutorStatus(
            command_id=None,
            status=transport_models.s.ExecutorStatus.RUNNING,
        )

        status = self.robot.executor_status()

        self.http_client.get.assert_called_once_with(paths_gen.Robot.GET_TRAJECTORY_EXECUTOR_STATUS,
                                                     transport_models.s.TrajectoryExecutorStatus)
        self.assertIsInstance(status, TrajectoryExecutorStatus)
        self.assertEqual(status.status, robot.ExecutorStatus.RUNNING)

    def test_executor_status_at_rest(self):
        """Test getting the executor status of a robot that has moved nothing since it started."""
        # Built through model_validate, like the HTTP client does: the raw body is where pydantic
        # rejects a status the generated enum does not carry.
        self.http_client.get.return_value = transport_models.s.TrajectoryExecutorStatus.model_validate(
            {'status': 'idle'})

        status = self.robot.executor_status()

        self.assertEqual(status.status, robot.ExecutorStatus.IDLE)
        self.assertFalse(status.status.is_final())
        self.assertFalse(status.status.is_error())

    def test_pause(self):
        """Test pausing trajectory execution."""
        self.http_client.patch.return_value = None

        self.robot.pause()

        self.http_client.patch.assert_called_once()
        call_args = self.http_client.patch.call_args[0]
        self.assertEqual(call_args[0], paths_gen.Robot.UPDATE_TRAJECTORY_EXECUTOR_STATUS)
        request = call_args[2]
        self.assertEqual(request.status, transport_models.s.ExecutorStatus.PAUSED)

    def test_stop(self):
        """Test stopping trajectory execution."""
        self.http_client.patch.return_value = None

        self.robot.stop()

        request = self.http_client.patch.call_args[0][2]
        self.assertEqual(request.status, transport_models.s.ExecutorStatus.STOPPED)

    def test_resume(self):
        """Test resuming trajectory execution."""
        self.http_client.patch.return_value = None

        self.robot.resume()

        request = self.http_client.patch.call_args[0][2]
        self.assertEqual(request.status, transport_models.s.ExecutorStatus.RUNNING)

    def test_get_target_speed(self):
        """Test getting the target speed factor."""
        self.http_client.get.return_value = transport_models.s.SpeedFactor(speed_factor=0.8)

        factor = self.robot.get_target_speed()

        self.http_client.get.assert_called_once_with(paths_gen.Robot.GET_TARGET_SPEED_FACTOR,
                                                     transport_models.s.SpeedFactor)
        self.assertEqual(factor, 0.8)

    def test_set_target_speed(self):
        """Test setting the target speed factor."""
        self.http_client.patch.return_value = transport_models.s.SpeedFactor(speed_factor=0.5)

        applied = self.robot.set_target_speed(0.5)

        self.assertEqual(applied, 0.5)
        self.http_client.patch.assert_called_once()
        call_args = self.http_client.patch.call_args[0]
        self.assertEqual(call_args[0], paths_gen.Robot.UPDATE_TARGET_SPEED_FACTOR)
        request = call_args[2]
        self.assertEqual(request.speed_factor, 0.5)

    def test_on_speed_factor(self):
        """Test subscribing to speed factor changes."""
        callback = MagicMock()

        unsubscribe = self.robot.on_speed_factor(callback)

        self.assertIs(unsubscribe, self.mqtt_client.subscribe.return_value)
        self.mqtt_client.subscribe.assert_called_once()
        topic, internal_callback, model = self.mqtt_client.subscribe.call_args[0]
        self.assertEqual(topic, topics_gen.Robot.SPEED_FACTOR)
        self.assertEqual(model, transport_models.a.SpeedFactor)

        # Simulate receiving a speed factor update
        internal_callback(topic, transport_models.a.SpeedFactor(speed_factor=0.75))
        callback.assert_called_once_with(0.75)

    def test_on_move_feedback(self):
        """Test subscribing to move feedback."""
        command_id = str(uuid4())
        callback = MagicMock()

        self.robot.on_move_feedback(command_id, callback)

        self.mqtt_client.subscribe.assert_called_once()
        topic, internal_callback, model = self.mqtt_client.subscribe.call_args[0]
        expected_topic = self.mqtt_client.format(topics_gen.Robot.ROBOT_MOVE_FEEDBACK, cmd_id=command_id)
        self.assertEqual(topic, expected_topic)
        self.assertEqual(model, transport_models.a.MoveFeedback)

        # Simulate receiving feedback
        internal_callback(
            topic, transport_models.a.MoveFeedback(state=transport_models.a.MoveFeedbackState.DONE, message='Move completed'))
        callback.assert_called_once()
        called_feedback = callback.call_args[0][0]
        self.assertIsInstance(called_feedback, motion.MoveFeedback)
        self.assertEqual(called_feedback.state, motion.MoveState.DONE)


class TestRobotSpecAlignment(BaseTestComponent):

    def setUp(self):
        super().setUp()
        self.robot = Robot(http_client=self.http_client,
                           mqtt_client=self.mqtt_client,
                           correlation_id=self.correlation_id)

    def test_frame_id_is_url_encoded(self):
        self.http_client.get.return_value = geometry.Pose.from_sequence(0, 0, 0, 0, 0, 0, 1).to_transport_model()

        self.robot.get_frame_pose('my tcp/1')

        self.assertEqual(self.http_client.get.call_args[0][0], '/api/robot/frames/my%20tcp%2F1/pose')

    def test_delete_tcp_frame(self):
        self.http_client.delete.return_value = transport_models.EmptyPayload()

        self.robot.delete_tcp_frame('my_tcp')

        self.http_client.delete.assert_called_once_with(paths_gen.Robot.DELETE_FRAME.format(frame_id='my_tcp'),
                                                        transport_models.EmptyPayload)

    def test_move_pose_reference_frame(self):
        self.http_client.post.return_value = transport_models.s.FeedbackResponse(feedback_id=uuid4())

        self.robot.move_pose(geometry.Pose.from_sequence(0.1, 0, 0.3, 0, 0, 0, 1), frame_id='eef_tcp',
                             reference_frame='table')

        waypoint = self.http_client.post.call_args[0][2].waypoints[0]
        self.assertEqual(waypoint.frame_id, 'eef_tcp')
        self.assertEqual(waypoint.reference_frame, 'table')

    def test_error_preparing_raises_a_pyniryo_error(self):
        from pyniryo.nate.exceptions import PrepareTrajectoryError
        self.assertIs(motion.MoveState.ERR_PREPARING.get_exception(), PrepareTrajectoryError)

    @patch('pyniryo.nate.components.robot.time')
    def test_timeout_counts_from_the_command_creation(self, mock_time):
        mock_time.monotonic.return_value = 100.0
        cmd = MoveCommand(self.robot, str(uuid4()), timeout=10.0, callbacks=[])
        cmd._started_at = 0.0

        with self.assertRaises(TimeoutError):
            cmd.wait()

        mock_time.sleep.assert_not_called()

    @patch('pyniryo.nate.components.robot.time')
    def test_zero_timeout_waits_indefinitely(self, mock_time):
        mock_time.monotonic.side_effect = [0.0, 100.0, 200.0]
        cmd = MoveCommand(self.robot, str(uuid4()), timeout=0.0, callbacks=[])
        internal_callback = self.mqtt_client.subscribe.call_args[0][1]
        mock_time.sleep.side_effect = lambda _: internal_callback(
            'topic', transport_models.a.MoveFeedback(state=transport_models.a.MoveFeedbackState.DONE, message=''))

        cmd.wait()

        self.assertEqual(cmd.state, motion.MoveState.DONE)


class TestRobotSubscriptions(BaseTestComponent):

    def setUp(self):
        super().setUp()
        self.robot = Robot(http_client=self.http_client,
                           mqtt_client=self.mqtt_client,
                           correlation_id=self.correlation_id)

    def test_on_led_ring(self):
        callback = MagicMock()

        unsubscribe = self.robot.on_led_ring(callback)

        topic, internal_callback, model = self.mqtt_client.subscribe.call_args[0]
        self.assertEqual(topic, topics_gen.Robot.LED_RING)
        self.assertEqual(model, transport_models.a.LedRing)
        self.assertIs(unsubscribe, self.mqtt_client.subscribe.return_value)
        internal_callback(topic, transport_models.a.LedRing(animation='breath',
                                                            color={'red': 0, 'green': 0, 'blue': 255},
                                                            brightness=200,
                                                            period_ms=1500))
        callback.assert_called_once_with(
            robot.LedRing(animation='breath', color=robot.Color(red=0, green=0, blue=255), brightness=200,
                          period_ms=1500))

    def test_on_led_ring_solid_has_no_period(self):
        callback = MagicMock()
        self.robot.on_led_ring(callback)
        topic, internal_callback, _ = self.mqtt_client.subscribe.call_args[0]

        internal_callback(topic, transport_models.a.LedRing(animation='solid',
                                                            color={'red': 0, 'green': 255, 'blue': 0},
                                                            brightness=255))

        self.assertIsNone(callback.call_args[0][0].period_ms)

    def test_on_control_mode(self):
        callback = MagicMock()

        self.robot.on_control_mode(callback)

        topic, internal_callback, model = self.mqtt_client.subscribe.call_args[0]
        self.assertEqual(topic, topics_gen.Robot.CONTROL_MODE)
        self.assertEqual(model, transport_models.a.ControlMode)
        internal_callback(topic, transport_models.a.ControlMode(mode=transport_models.a.ControlModeMode._2,
                                                                mode_name=transport_models.a.ControlModeModeName.JOG))
        callback.assert_called_once_with(robot.ControlMode.JOG)

    def test_on_jog_limit_violation(self):
        callback = MagicMock()

        self.robot.on_jog_limit_violation(callback)

        topic, internal_callback, model = self.mqtt_client.subscribe.call_args[0]
        self.assertEqual(topic, topics_gen.Robot.JOG_LIMIT_VIOLATION)
        self.assertEqual(model, transport_models.a.JogLimitViolation)
        internal_callback(topic, transport_models.a.JogLimitViolation(joint=4,
                                                                      position_limit_min=-3.14,
                                                                      position_limit_max=0.086))
        callback.assert_called_once_with(motion.JogLimitViolation(joint=4,
                                                                  position_limit_min=-3.14,
                                                                  position_limit_max=0.086))


class TestMoveCommand(BaseTestComponent):

    def setUp(self):
        super().setUp()
        self.robot = Robot(http_client=self.http_client,
                           mqtt_client=self.mqtt_client,
                           correlation_id=self.correlation_id)

    def tearDown(self):
        del self.robot

    def test_initial_state(self):
        """Test initial state of MoveCommand."""
        command_id = str(uuid4())

        cmd = MoveCommand(self.robot, command_id, timeout=0.0, callbacks=[])

        self.assertEqual(cmd.command_id, command_id)
        self.assertEqual(cmd.state, motion.MoveState.UNKNOWN)

        # Should subscribe to feedback topic via robot.on_move_feedback
        self.mqtt_client.subscribe.assert_called_once()
        topic = self.mqtt_client.subscribe.call_args[0][0]
        self.assertEqual(topic, self.mqtt_client.format(topics_gen.Robot.ROBOT_MOVE_FEEDBACK, cmd_id=command_id))

    def test_feedback_callback_updates_state(self):
        """Test that feedback callback updates state."""
        command_id = str(uuid4())
        cmd = MoveCommand(self.robot, command_id, timeout=0.0, callbacks=[])

        # Get the internal callback
        internal_callback = self.mqtt_client.subscribe.call_args[0][1]
        topic = self.mqtt_client.subscribe.call_args[0][0]

        # Simulate receiving feedback
        feedback = transport_models.a.MoveFeedback(state=transport_models.a.MoveFeedbackState.PREPARING, message='Preparing move')
        internal_callback(topic, feedback)

        self.assertEqual(cmd.state, motion.MoveState.PREPARING)

    def test_feedback_callback_unsubscribes_on_final_state(self):
        """Test that feedback callback triggers unsubscribe when reaching final state."""
        command_id = str(uuid4())
        _ = MoveCommand(self.robot, command_id, timeout=0.0, callbacks=[])

        internal_callback = self.mqtt_client.subscribe.call_args[0][1]
        topic = self.mqtt_client.subscribe.call_args[0][0]

        # Simulate completing the move — unsubscribe should be called
        feedback = transport_models.a.MoveFeedback(state=transport_models.a.MoveFeedbackState.DONE, message='Move completed')
        unsubscribe_mock = MagicMock()
        self.mqtt_client.subscribe.return_value = unsubscribe_mock
        internal_callback(topic, feedback)

    def test_user_callback_invoked_on_feedback(self):
        """Test that user-provided callback is invoked on move feedback."""
        command_id = str(uuid4())
        user_callback = MagicMock()
        _ = MoveCommand(self.robot, command_id, timeout=0.0, callbacks=[user_callback])

        internal_callback = self.mqtt_client.subscribe.call_args[0][1]
        topic = self.mqtt_client.subscribe.call_args[0][0]

        feedback = transport_models.a.MoveFeedback(state=transport_models.a.MoveFeedbackState.PREPARING, message='Preparing')
        internal_callback(topic, feedback)

        user_callback.assert_called_once()
        called_feedback = user_callback.call_args[0][0]
        self.assertIsInstance(called_feedback, motion.MoveFeedback)
        self.assertEqual(called_feedback.state, motion.MoveState.PREPARING)

    @patch('time.sleep')
    @patch('time.monotonic')
    def test_wait_success(self, mock_monotonic, mock_sleep):
        """Test waiting for move to complete successfully."""
        command_id = str(uuid4())
        mock_monotonic.return_value = 0.0

        cmd = MoveCommand(self.robot, command_id, timeout=0.0, callbacks=[])

        # Simulate move completion
        internal_callback = self.mqtt_client.subscribe.call_args[0][1]
        topic = self.mqtt_client.subscribe.call_args[0][0]
        feedback = transport_models.a.MoveFeedback(state=transport_models.a.MoveFeedbackState.DONE, message='Move completed')
        internal_callback(topic, feedback)

        # Wait should return without error
        cmd.wait()

    @patch('time.sleep')
    @patch('time.monotonic')
    def test_wait_timeout(self, mock_monotonic, mock_sleep):
        """Test waiting for move with timeout."""
        command_id = str(uuid4())
        mock_monotonic.side_effect = [0.0, 0.5, 1.0, 1.5]

        cmd = MoveCommand(self.robot, command_id, timeout=1.0, callbacks=[])

        # Don't change state, so it stays in UNKNOWN
        with self.assertRaises(TimeoutError):
            cmd.wait()

    @patch('time.sleep')
    @patch('time.monotonic')
    def test_wait_error_generate_trajectory(self, mock_monotonic, mock_sleep):
        """Test waiting for move that fails with trajectory generation error."""
        command_id = str(uuid4())
        mock_monotonic.return_value = 0.0

        cmd = MoveCommand(self.robot, command_id, timeout=0.0, callbacks=[])

        # Simulate trajectory generation error
        internal_callback = self.mqtt_client.subscribe.call_args[0][1]
        topic = self.mqtt_client.subscribe.call_args[0][0]
        feedback = transport_models.a.MoveFeedback(state=transport_models.a.MoveFeedbackState.ERROR_GENERATING_TRAJECTORY,
                                                   message='Failed to generate trajectory')
        internal_callback(topic, feedback)

        # Wait should raise the appropriate error
        with self.assertRaises(GenerateTrajectoryError):
            cmd.wait()

    @patch('time.sleep')
    @patch('time.monotonic')
    def test_wait_negative_timeout(self, mock_monotonic, mock_sleep):
        """Test waiting with negative timeout (wait indefinitely)."""
        command_id = str(uuid4())
        mock_monotonic.return_value = 0.0

        cmd = MoveCommand(self.robot, command_id, timeout=-1.0, callbacks=[])

        # Complete move immediately
        internal_callback = self.mqtt_client.subscribe.call_args[0][1]
        topic = self.mqtt_client.subscribe.call_args[0][0]
        feedback = transport_models.a.MoveFeedback(state=transport_models.a.MoveFeedbackState.DONE, message='Move completed')
        internal_callback(topic, feedback)

        # Should work with negative timeout (waits indefinitely)
        cmd.wait()

    def test_move_command_pause_delegates_to_robot(self):
        """Test that MoveBaseCommand.pause() delegates to robot.pause()."""
        command_id = str(uuid4())
        cmd = MoveCommand(self.robot, command_id, timeout=0.0, callbacks=[])
        self.http_client.patch.return_value = None

        cmd.pause()

        self.http_client.patch.assert_called_once()
        request = self.http_client.patch.call_args[0][2]
        self.assertEqual(request.status, transport_models.s.ExecutorStatus.PAUSED)

    def test_move_command_resume_delegates_to_robot(self):
        """Test that MoveBaseCommand.resume() delegates to robot.resume()."""
        command_id = str(uuid4())
        cmd = MoveCommand(self.robot, command_id, timeout=0.0, callbacks=[])
        self.http_client.patch.return_value = None

        cmd.resume()

        self.http_client.patch.assert_called_once()
        request = self.http_client.patch.call_args[0][2]
        self.assertEqual(request.status, transport_models.s.ExecutorStatus.RUNNING)

    def test_move_command_stop_delegates_to_robot(self):
        """Test that MoveBaseCommand.stop() delegates to robot.stop()."""
        command_id = str(uuid4())
        cmd = MoveCommand(self.robot, command_id, timeout=0.0, callbacks=[])
        self.http_client.patch.return_value = None

        cmd.stop()

        self.http_client.patch.assert_called_once()
        request = self.http_client.patch.call_args[0][2]
        self.assertEqual(request.status, transport_models.s.ExecutorStatus.STOPPED)


if __name__ == "__main__":
    unittest.main()
