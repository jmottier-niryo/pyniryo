import contextlib
import logging
import sys
import time
import warnings
import xml.etree.ElementTree as ET
from urllib.parse import quote
from collections import deque
from io import BytesIO
from threading import Event
from typing import Callable, List, Any, Generator
from uuid import uuid4

from pydantic import BaseModel

if sys.version_info >= (3, 13):
    from warnings import deprecated
else:
    from typing_extensions import deprecated

from . import BaseAPIComponent, BaseCommand

from .._internal.http import HttpClient
from .._internal.mqtt import MqttClient
from .._internal import transport_models, paths_gen, topics_gen
from ..models import (Pose,
                      Waypoint,
                      Joints,
                      MoveFeedback,
                      MoveState,
                      MoveTarget,
                      RobotConfiguration,
                      ControlMode,
                      Unsubscribe,
                      JointsStamped,
                      Planner,
                      TrajectoryExecutorStatus,
                      LedRing,
                      JogLimitViolation)

logger = logging.getLogger(__name__)

MoveFeedbackCallback = Callable[[MoveFeedback], None]
JointsCallback = Callable[[Joints], None]
PoseCallback = Callable[[Pose], None]
SpeedFactorCallback = Callable[[float], None]
LedRingCallback = Callable[[LedRing], None]
ControlModeCallback = Callable[[ControlMode], None]
JogLimitViolationCallback = Callable[[JogLimitViolation], None]


class Robot(BaseAPIComponent):
    """
    Robot component for controlling robot movements and accessing robot state.
    
    This component provides methods for:
    - Getting and monitoring joint positions
    - Moving the robot using joints, poses, waypoints, or trajectories
    - Managing frames and coordinate systems
    - Controlling robot execution (pause, resume, stop)
    - Jogging in manual mode

    Moving the arm is direct control, which the robot accepts only when its key selector is on manual,
    with one exception for the program it is running. See :doc:`/api/nate/direct_control`.
    """

    def __init__(self, http_client: HttpClient, mqtt_client: MqttClient, correlation_id: str) -> None:
        super().__init__(http_client, mqtt_client, correlation_id)
        self._cmd_queue: deque['MoveCommand'] = deque(maxlen=5)
        self._cmd_freeze = Event()
        self._cmd_freeze.set()

    def _freeze_commands(self) -> None:
        self._cmd_freeze.clear()

    def _unfreeze_commands(self) -> None:
        self._cmd_freeze.set()

    def get_joints(self) -> Joints:
        """
        Get the current joint positions of the robot.

        :return: The current joint positions as a Joints object.
        """
        joints = self._http_client.get(paths_gen.Robot.GET_ROBOT_JOINTS, transport_models.s.Joints)
        return Joints.from_transport_model(joints)

    def on_joints(self, callback: JointsCallback) -> Unsubscribe:
        """
        Set the callback to call when the robot's joint positions are received.

        :param callback: The callback function that takes a Joints parameter.
        """

        def internal_callback(_: str, joints: transport_models.a.Joints) -> None:
            callback(Joints.from_a_transport_model(joints))

        return self._mqtt_client.subscribe(topics_gen.Robot.JOINTS, internal_callback, transport_models.a.Joints)

    def get_all_frames(self) -> List[str]:
        """
        Get all frames defined in the robot.

        :return: A list of frame IDs.
        """
        frames = self._http_client.get(
            paths_gen.Robot.GET_ALL_FRAMES,
            transport_models.FrameIdList,
        )
        return frames.root

    def get_frame_pose(self, frame_id: str) -> Pose:
        """
        Get the current pose of a specific frame.

        :param frame_id: The ID of the frame to retrieve.
        :return: The current pose of the frame as a Pose object.
        """
        pose = self._http_client.get(
            paths_gen.Robot.GET_FRAME_POSE.format(frame_id=quote(frame_id, safe='')),
            transport_models.s.Pose,
        )
        return Pose.from_transport_model(pose)

    def set_tcp_frame(self, frame_id: str, transform: Pose) -> None:
        """
        Create or replace a tool center point frame, attached to the flange.

        :param frame_id: The name of the frame, a ROS name token (letters, digits and underscores, not starting
            with a digit) of at most 36 characters.
        :param transform: The transform of the frame from the flange.
        :raises ClientError: If the name is refused, or another workstation holds the robot
            (``code == 'robot_held'``).
        """
        self._http_client.post(paths_gen.Robot.CREATE_FRAME.format(frame_id=quote(frame_id, safe='')),
                               transport_models.EmptyPayload,
                               transform.to_transport_model())

    def delete_tcp_frame(self, frame_id: str) -> None:
        """
        Delete a tool center point frame created with :meth:`set_tcp_frame`.

        :param frame_id: The name of the frame.
        :raises ClientError: If the name is not a ROS name token, or another workstation holds the robot
            (``code == 'robot_held'``).
        """
        self._http_client.delete(paths_gen.Robot.DELETE_FRAME.format(frame_id=quote(frame_id, safe='')),
                                 transport_models.EmptyPayload)

    @deprecated("Use `set_tcp_frame` instead.")
    def set_frame(self, frame_id: str, transform: Pose) -> None:
        """
        .. deprecated:: use set_tcp_frame instead. set_frame name is too ambiguous and doesn't reflect the fact that
            it only allows to set the TCP frame. It will be removed in a future release.

        :param frame_id:
        :param transform:
        :return:
        """
        warnings.warn("set_frame() is deprecated and will be removed. "
                      "Use set_tcp_frame() instead.",
                      category=DeprecationWarning,
                      stacklevel=2)
        return self.set_tcp_frame(frame_id, transform)

    def on_frame_pose(self, frame_id: str, callback: PoseCallback) -> Unsubscribe:
        """
        Set a callback to be called when the pose of a specific frame is received.

        :param frame_id: The ID of the frame to subscribe to.
        :param callback: The callback function that takes a Pose parameter.
        """

        def internal_callback(_: str, pose: transport_models.a.Pose) -> None:
            callback(Pose.from_transport_model(pose))

        return self._mqtt_client.subscribe(
            self._mqtt_client.format(topics_gen.Robot.FRAME_POSE, frame_id=frame_id),
            internal_callback,
            transport_models.a.Pose,
        )

    def _normalize_move_target(self, target: MoveTarget) -> list[Waypoint]:
        if not isinstance(target, list):
            target = [target]
        waypoints = []
        for t in target:
            if isinstance(t, Joints):
                waypoints.append(Waypoint(joints=t))
            elif isinstance(t, Pose):
                waypoints.append(Waypoint(pose=t))
            elif isinstance(t, Waypoint):
                waypoints.append(t)
            else:
                valid_types = ', '.join(f'{m.__module__}.{m.__qualname__}' for m in MoveTarget.__args__)
                raise TypeError(f'Invalid type {t.__class__.__name__} for target. Expected one of {valid_types}')
        return waypoints

    def __move(self,
               endpoint: str,
               data: BaseModel,
               timeout: float,
               on_feedback: MoveFeedbackCallback | None = None) -> 'MoveCommand':
        self._cmd_freeze.wait()

        command_id = uuid4()
        if hasattr(data, 'command_id'):
            command_id = data.command_id

        callbacks = [on_feedback] if on_feedback is not None else []
        move_command = MoveCommand(self, str(command_id), timeout=timeout, callbacks=callbacks)
        self._cmd_queue.append(move_command)
        try:
            self._http_client.post(endpoint, transport_models.s.FeedbackResponse, data)
        except Exception:
            self._cmd_queue.remove(move_command)
            move_command._stop_tracking()
            raise
        return move_command

    def move_joints(self, *joints: float | Joints) -> 'MoveCommand':
        """
        High-level move: move the robot to the given joint positions, through ``POST /api/robot/waypoints``. This
        covers ``POST /api/robot/joints``, which the client does not expose separately.

        For full trajectory control (custom planners, blending radii, multi-waypoint paths, etc.),
        use the low-level :meth:`move` method with :class:`Waypoint` objects instead.

        Direct control: the key selector must be on manual, or the caller must be a program the robot is
        running, see :doc:`/api/nate/direct_control`.

        :param joints: Target joint positions in radians. Can be passed as individual floats
            (e.g. ``move_joints(0, 0, 0, 0, 0, 0)``) or as a single :class:`Joints` object
            (e.g. ``move_joints(Joints(0, 0, 0, 0, 0, 0))``).
        :return: A :class:`MoveCommand` to track the movement progress.
        :raises ClientError: If the robot refuses the move because its key selector is on automatic.
        """
        if not joints:
            raise TypeError('Joints cannot be empty')

        if len(joints) == 1 and isinstance(joints[0], Joints):
            target = joints[0]
        else:
            float_args: list[float] = []
            for j in joints:
                if not isinstance(j, (float, int)):
                    raise TypeError(f'Invalid type for joint position. '
                                    f'Expected float or int, got "{j.__class__.__name__}"')
                float_args.append(j)

            target = Joints(*float_args)

        return self.move(target, add_start=True)

    def move_pose(self,
                  *pose: float | Pose,
                  frame_id: str | None = None,
                  reference_frame: str | None = None,
                  linear: bool = False) -> 'MoveCommand':
        """
        High-level move: move the robot to the given Cartesian pose, through ``POST /api/robot/waypoints``. This
        covers ``POST /api/robot/frames/{frame_id}/pose``, which the client does not expose separately.

        For full trajectory control (blending radii, velocity/acceleration factors, multi-waypoint
        paths, etc.), use the low-level :meth:`move` method with :class:`Waypoint` objects instead.

        Direct control: the key selector must be on manual, or the caller must be a program the robot is
        running, see :doc:`/api/nate/direct_control`.

        :param pose: Target pose. Can be a single :class:`Pose` object, or a flat sequence of
            floats interpreted by :meth:`Pose.from_sequence` — 6 floats for
            ``(x, y, z, roll, pitch, yaw)`` or 7 floats for ``(x, y, z, qx, qy, qz, qw)``.
        :param frame_id: The frame to bring to the pose (e.g. ``'eef_tcp'``, ``'tool0'``). If ``None``, the
            robot's default tool center point is used.
        :param reference_frame: The frame the pose is expressed in. If ``None``, the robot's base frame is used.
        :param linear: If ``True``, use linear interpolation (:attr:`Planner.LIN`) for a
            straight-line path. If ``False`` (default), use point-to-point planning
            (:attr:`Planner.PTP`).
        :return: A :class:`MoveCommand` to track the movement progress.
        :raises ClientError: If the robot refuses the move because its key selector is on automatic.
        """
        if not pose:
            raise TypeError('pose cannot be empty')

        if len(pose) == 1 and isinstance(pose[0], Pose):
            target = pose[0]
        else:
            float_args: list[float] = []
            for p in pose:
                if not isinstance(p, (float, int)):
                    raise TypeError(f'Invalid type for joint position. '
                                    f'Expected float or int, got "{p.__class__.__name__}"')
                float_args.append(p)
            target = Pose.from_sequence(*float_args)

        waypoint = Waypoint(
            pose=target,
            frame_id=frame_id,
            reference_frame=reference_frame,
            planner=Planner.LIN if linear else Planner.PTP,
        )
        return self.move(waypoint, add_start=True)

    def move(self,
             target: MoveTarget,
             add_start: bool = False,
             timeout: float = -1,
             on_feedback: MoveFeedbackCallback | None = None) -> 'MoveCommand':
        """
        Low-level move: move the robot according to the provided target.

        This is the most flexible move method, giving full control over waypoints, planners,
        blending radii, velocity/acceleration factors, and multi-waypoint trajectories. If joints
        and pose are both provided in the same :class:`Waypoint`, joints take priority.

        Direct control: the key selector must be on manual, or the caller must be a program the robot is
        running, see :doc:`/api/nate/direct_control`.

        :param target: The target to reach. Can be :class:`Joints`, :class:`Pose`,
            :class:`Waypoint`, or a list of these types.
        :param add_start: Whether to automatically prepend the current robot position as the
            first waypoint.
        :param timeout: Maximum time in seconds to wait before considering the move failed, counted from the
            moment the command is sent. Zero or a negative value means no timeout.
        :param on_feedback: Optional callback to receive :class:`MoveFeedback` during the movement.
        :return: A :class:`MoveCommand` to track the movement progress.
        :raises ClientError: If the robot refuses the move because its key selector is on automatic.
        """
        normalized_target = self._normalize_move_target(target)
        data = transport_models.s.MoveWaypoints(command_id=uuid4(),
                                                add_start=add_start,
                                                waypoints=[w.to_transport_model() for w in normalized_target])
        return self.__move(paths_gen.Robot.MOVE_ALONG_WAYPOINTS, data, timeout=timeout, on_feedback=on_feedback)

    def execute_trajectory(self,
                           trajectory: list[JointsStamped],
                           timeout: float = -1,
                           on_feedback: MoveFeedbackCallback | None = None) -> 'MoveCommand':
        """
        Execute a pre-computed trajectory on the robot.

        Direct control: the key selector must be on manual, or the caller must be a program the robot is
        running, see :doc:`/api/nate/direct_control`.

        :param trajectory: The trajectory to execute. For example a trajectory generated by the motion planner.
        :param timeout: Maximum time to wait before considering a trajectory command failed.
        :param on_feedback: Optional callback to receive feedback during the trajectory.

        :return: A MoveCommand object to track the progress of the movement.
        :raises ClientError: If the robot refuses the move because its key selector is on automatic.
        """
        data = transport_models.s.TrajectoryExecution(command_id=uuid4(),
                                                      trajectory=transport_models.s.Trajectory(
                                                          [j.to_transport_model() for j in trajectory]))
        return self.__move(paths_gen.Robot.EXECUTE_TRAJECTORY, data, timeout, on_feedback)

    def get_commands(self) -> list['MoveCommand']:
        """
        Returns the 5 latest commands sent by the client.

        :return: A list of maximum 5 MoveCommand objects.
        """
        return list(self._cmd_queue)

    def get_urdf(self) -> ET.Element:
        """
        Get the robot's URDF (Unified Robot Description Format) description.

        :return: The root XML element of the URDF document.
        """
        buffer = BytesIO()
        self._http_client.download(paths_gen.Robot.GET_ROBOT_URDF, buffer)
        buffer.seek(0)
        tree = ET.parse(buffer)
        return tree.getroot()

    def get_configuration(self) -> RobotConfiguration:
        """
        Get the robot's configuration including joint limits and PID parameters.

        :return: The robot configuration.
        """
        resp = self._http_client.get(paths_gen.Robot.GET_ROBOT_CONFIG, transport_models.s.RobotConfig)
        return RobotConfiguration.from_transport_model(resp)

    def get_control_mode(self) -> ControlMode:
        """
        Get the current control mode of the robot.

        :return: The current control mode (TRAJECTORY, JOG, or SPEED).
        """
        resp = self._http_client.get(paths_gen.Robot.GET_ROBOT_CONTROL_MODE, transport_models.s.ControlMode)
        return ControlMode.from_transport_model(resp)

    def set_control_mode(self, mode: ControlMode) -> None:
        """
        Set the control mode of the robot.

        Direct control: the key selector must be on manual, or the caller must be a program the robot is
        running, see :doc:`/api/nate/direct_control`.

        :param mode: The control mode to set (TRAJECTORY, JOG, or SPEED).
        :raises RuntimeError: If the mode could not be set.
        :raises ClientError: If the robot refuses the change because its key selector is on automatic.
        """
        resp = self._http_client.put(paths_gen.Robot.SET_ROBOT_CONTROL_MODE,
                                     transport_models.s.ControlMode,
                                     mode.to_transport_model(),
                                     exclude={'mode_name'})
        if resp.mode.value != mode.value:
            raise RuntimeError(f'Failed to set control mode to {mode}. Current mode is {resp.mode}')

    @contextlib.contextmanager
    def jog_mode(self) -> Generator[None, Any, None]:
        """
        Context manager to temporarily set the robot in jog mode. The robot will be set back to its previous control
        mode when exiting the context.

        This context manager only sets the control mode, so it is not the guard it looks like. From an
        outside script it raises on automatic, on entry, and a key turned to automatic during the block
        makes the exit raise too, leaving the robot in jog mode. From a program it opens without a word,
        and the jog commands inside the block are then dropped in silence. See
        :doc:`/api/nate/direct_control`.

        Usage:
        ```
        with robot.jog_mode():
            # Do something in jog mode
            ...
        # Back to previous control mode
        ```

        :raises ClientError: If the robot refuses to set the mode because its key selector is on automatic.
        """
        previous_mode = self.get_control_mode()
        self.set_control_mode(ControlMode.JOG)
        try:
            yield
        finally:
            self.set_control_mode(previous_mode)

    def stop_jog(self) -> None:
        """
        Stop the current continuous jog command. The robot must be in jog mode for this to work. An incremental jog is
        not affected and runs to completion.

        Direct control: the key selector must be on manual, with no exception for a running program. On
        automatic the robot drops the command and answers nothing, see :doc:`/api/nate/direct_control`.
        """
        self._mqtt_client.publish(topics_gen.Cmd.ROBOT_JOG_STOP, transport_models.EmptyPayload())

    def jog_joints(self, velocity: Joints) -> None:
        """
        Move the joints continuously at the given velocities. The robot must be in jog mode for this to work.

        The robot decelerates to a stop when the command is not renewed within 50 ms, or on :meth:`stop_jog`. See
        :meth:`jog_joints_increment` for a one-shot move by an increment.

        Direct control: the key selector must be on manual, with no exception for a running program. On
        automatic the robot drops the command and answers nothing, see :doc:`/api/nate/direct_control`.

        :param velocity: The joint velocities of the jog command (in rad/s).
        """
        self._mqtt_client.publish(topics_gen.Cmd.ROBOT_JOG_JOINT,
                                  transport_models.a.JogJoint(velocities=velocity.data))

    def jog_joints_increment(self, increment: Joints) -> None:
        """
        Move each joint once by the given increment. The robot must be in jog mode for this to work.

        The motion runs to completion, :meth:`stop_jog` does not interrupt it. An increment whose end lands past the
        travel the jog controller allows is refused whole and nothing moves; the controller keeps a margin on each
        side of a joint limit and reports that travel, see :meth:`on_jog_limit_violation`. A single command carrying both an increment and velocities moves by
        the increment. See :meth:`jog_joints` for a continuous move at a given velocity.

        Direct control: the key selector must be on manual, with no exception for a running program. On
        automatic the robot drops the command and answers nothing, see :doc:`/api/nate/direct_control`.

        :param increment: The joint increments of the jog command (in rad), counted from the current joint positions.
        """
        self._mqtt_client.publish(topics_gen.Cmd.ROBOT_JOG_JOINT,
                                  transport_models.a.JogJoint(displacements=increment.data))

    def jog_cartesian(self,
                      linear_velocity: list[float],
                      angular_velocity: list[float],
                      frame_id: str | None = None) -> None:
        """
        Move the active tool center point continuously at the given velocities. The robot must be in jog mode for this
        to work.

        The robot decelerates to a stop when the command is not renewed within 50 ms, or on :meth:`stop_jog`. See
        :meth:`jog_cartesian_increment` for a one-shot move by an increment.

        Direct control: the key selector must be on manual, with no exception for a running program. On
        automatic the robot drops the command and answers nothing, see :doc:`/api/nate/direct_control`.

        :param linear_velocity: The linear velocity of the jog command in m/s. It should be a list of 3 floats representing the velocity in the x, y and z directions.
        :param angular_velocity: The angular velocity of the jog command in rad/s. It should be a list of 3 floats representing the angular velocity around the x, y and z axes.
        :param frame_id: The frame whose axes the velocities are expressed in, one of the names returned by :meth:`get_all_frames`. Only its orientation is used. If not specified, the velocities read in the axes of the active tool center point.
        """
        self._mqtt_client.publish(
            topics_gen.Cmd.ROBOT_JOG_CARTESIAN,
            transport_models.a.JogCartesian(linear=linear_velocity, angular=angular_velocity, frame_id=frame_id))

    def jog_cartesian_increment(self,
                                linear_increment: list[float],
                                angular_increment: list[float],
                                frame_id: str | None = None) -> None:
        """
        Move the active tool center point once by the given increment. The robot must be in jog mode for this to work.

        The motion runs to completion, :meth:`stop_jog` does not interrupt it. An increment whose path crosses a joint
        limit or a singularity is refused whole and nothing moves. See :meth:`jog_cartesian` for a continuous move at a
        given velocity.

        Direct control: the key selector must be on manual, with no exception for a running program. On
        automatic the robot drops the command and answers nothing, see :doc:`/api/nate/direct_control`.

        :param linear_increment: The linear increment of the jog command in m, counted from the current position. It should be a list of 3 floats representing the increment in the x, y and z directions.
        :param angular_increment: The angular increment of the jog command in rad, counted from the current orientation. It should be a list of 3 floats representing the angular increment around the x, y and z axes.
        :param frame_id: The frame whose axes the increments are expressed in, one of the names returned by :meth:`get_all_frames`. Only its orientation is used. If not specified, the increments read in the axes of the active tool center point.
        """
        self._mqtt_client.publish(
            topics_gen.Cmd.ROBOT_JOG_CARTESIAN,
            transport_models.a.JogCartesian(linear=linear_increment,
                                            angular=angular_increment,
                                            frame_id=frame_id,
                                            is_increment=True))

    def executor_status(self) -> TrajectoryExecutorStatus:
        """
        Get the current status of the trajectory executor.
        
        :return: The current status of the trajectory executor.
        """
        status = self._http_client.get(paths_gen.Robot.GET_TRAJECTORY_EXECUTOR_STATUS,
                                       transport_models.s.TrajectoryExecutorStatus)
        return TrajectoryExecutorStatus.from_transport_model(status)

    def get_next_trajectory_waypoint(self) -> JointsStamped | None:
        """
        Get the next waypoint of the trajectory currently being executed.

        :return: The next waypoint, or None if there is no active trajectory or no more waypoints.
        """
        resp = self._http_client.get_optional(paths_gen.Robot.GET_NEXT_TRAJECTORY_WAYPOINT,
                                              transport_models.s.JointsStamped)
        return JointsStamped.from_transport_model(resp) if resp is not None else None

    def _update_executor_status(self, status: transport_models.s.ExecutorStatus) -> None:
        self._http_client.patch(paths_gen.Robot.UPDATE_TRAJECTORY_EXECUTOR_STATUS,
                                transport_models.EmptyPayload,
                                transport_models.s.TrajectoryExecutorStatus(status=status))

    def pause(self) -> None:
        """
        Pause the current trajectory execution.
        """
        self._update_executor_status(transport_models.s.ExecutorStatus.PAUSED)

    def stop(self) -> None:
        """
        Stop the current trajectory execution.
        """
        self._update_executor_status(transport_models.s.ExecutorStatus.STOPPED)

    def resume(self) -> None:
        """
        Resume a paused trajectory execution.
        """
        self._update_executor_status(transport_models.s.ExecutorStatus.RUNNING)

    def get_target_speed(self) -> float:
        """
        Get the current target speed factor of the robot.
        The speed factor is a multiplier applied to the robot's maximum speed, allowing you to slow down or speed up movements without changing the underlying trajectories.
        :return: The target speed factor.
        """
        resp = self._http_client.get(paths_gen.Robot.GET_TARGET_SPEED_FACTOR, transport_models.s.SpeedFactor)
        return resp.speed_factor

    def set_target_speed(self, factor: float) -> float:
        """
        Set the target speed factor of the robot.
        :param factor: A factor comprised between 0 and 1. A factor of 1 means the robot will move at its maximum speed,
        while a factor of 0.5 means the robot will move at half of its maximum speed. Setting the speed factor does not
        change the underlying trajectories, it only scales the speed at which they are executed.
        :return: The target speed factor the robot applied.
        :raises ClientError: If another workstation holds the robot (``code == 'robot_held'``).
        """
        resp = self._http_client.patch(paths_gen.Robot.UPDATE_TARGET_SPEED_FACTOR,
                                       transport_models.s.SpeedFactor,
                                       transport_models.s.SpeedFactor(speed_factor=factor))
        return resp.speed_factor

    def on_speed_factor(self, callback: SpeedFactorCallback) -> Unsubscribe:
        """
        Set the callback to call when the speed factor the robot currently applies changes. It follows the target
        set by :meth:`set_target_speed`, capped by the safety chain.

        :param callback: The callback function that takes the new speed factor.
        """

        def internal_callback(_: str, sf: transport_models.a.SpeedFactor) -> None:
            callback(sf.speed_factor)

        return self._mqtt_client.subscribe(topics_gen.Robot.SPEED_FACTOR, internal_callback,
                                           transport_models.a.SpeedFactor)

    def on_target_speed_factor(self, callback: SpeedFactorCallback) -> Unsubscribe:
        """
        Set the callback to call when the robot's target speed factor changes.

        :param callback: The callback function that takes the new target speed factor.
        """

        def internal_callback(_: str, sf: transport_models.a.SpeedFactor) -> None:
            callback(sf.speed_factor)

        return self._mqtt_client.subscribe(topics_gen.Robot.SPEED_FACTOR_TARGET, internal_callback,
                                           transport_models.a.SpeedFactor)

    def on_control_mode(self, callback: ControlModeCallback) -> Unsubscribe:
        """
        Set the callback to call when the control mode of the robot changes.

        :param callback: The callback function that takes the new :class:`ControlMode`.
        """

        def internal_callback(_: str, mode: transport_models.a.ControlMode) -> None:
            callback(ControlMode.from_a_transport_model(mode))

        return self._mqtt_client.subscribe(topics_gen.Robot.CONTROL_MODE, internal_callback,
                                           transport_models.a.ControlMode)

    def on_led_ring(self, callback: LedRingCallback) -> Unsubscribe:
        """
        Set the callback to call when the light ring at the base of the robot changes. The state is retained, so the
        callback receives the current one as soon as the subscription is in place, once the robot has reported a
        first state.

        :param callback: The callback function that takes the new :class:`LedRing` rendering.
        """

        def internal_callback(_: str, ring: transport_models.a.LedRing) -> None:
            callback(LedRing.from_transport_model(ring))

        return self._mqtt_client.subscribe(topics_gen.Robot.LED_RING, internal_callback, transport_models.a.LedRing)

    def on_jog_limit_violation(self, callback: JogLimitViolationCallback) -> Unsubscribe:
        """
        Set the callback to call when the jog controller refuses a position increment for ending past the travel it
        allows on a joint, see :meth:`jog_joints_increment`. The report is not retained: subscribe before sending
        the increment.

        :param callback: The callback function that takes the :class:`JogLimitViolation`.
        """

        def internal_callback(_: str, violation: transport_models.a.JogLimitViolation) -> None:
            callback(JogLimitViolation.from_transport_model(violation))

        return self._mqtt_client.subscribe(topics_gen.Robot.JOG_LIMIT_VIOLATION, internal_callback,
                                           transport_models.a.JogLimitViolation)

    def on_move_feedback(self, command_id: str, callback: MoveFeedbackCallback) -> Unsubscribe:

        def internal_callback(_: str, payload: transport_models.a.MoveFeedback) -> None:
            callback(MoveFeedback.from_transport_model(payload))

        topic = self._mqtt_client.format(topics_gen.Robot.ROBOT_MOVE_FEEDBACK, cmd_id=command_id)
        return self._mqtt_client.subscribe(topic, internal_callback, transport_models.a.MoveFeedback)


class MoveCommand(BaseCommand):
    """
    Represents a move command and tracks its execution progress.

    This class monitors the state of a robot movement operation, receiving feedback
    through MQTT and allowing you to wait for completion.
    """

    def __init__(self, robot: Robot, command_id: str, timeout: float, callbacks: List[MoveFeedbackCallback]) -> None:
        super().__init__(command_id, timeout)
        self._robot = robot

        self._unsubscribe = self._robot.on_move_feedback(self._command_id, self._move_feedback_callback)

        self._callbacks: List[MoveFeedbackCallback] = callbacks
        self._feedbacks: List[MoveFeedback] = [MoveFeedback(state=MoveState.UNKNOWN, message="")]

    def _move_feedback_callback(self, feedback: MoveFeedback) -> None:
        """
        Internal callback to handle move feedback messages.
        """
        self._feedbacks.append(feedback)

        for callback in self._callbacks:
            try:
                callback(feedback)
            except Exception as e:
                logger.error(f'Error in callback "{callback.__qualname__}" for move feedback: {e}')

        if self.state.is_final():
            self._unsubscribe()

    def _stop_tracking(self) -> None:
        self._unsubscribe()

    @property
    def state(self) -> MoveState:
        """
        Get the current state of the move command.

        :return: The current state of the move command.
        """
        return self._feedbacks[-1].state

    def wait(self) -> None:
        """
        Wait for the move command to complete. A timeout of zero or less waits indefinitely. The timeout counts
        from the moment the command was sent. A paused move keeps waiting until it is resumed.

        :raises TimeoutError: If the timeout is exceeded.
        :raises PyNiryoError: If the execution fails.
        """
        while not self.state.is_final():
            if self._timeout > 0 and self._started_at + self._timeout < time.monotonic():
                raise TimeoutError(f'Move command {self._command_id} timed out after {self._timeout} seconds.')
            time.sleep(0.1)
        if self.state.is_error():
            raise self.state.get_exception()(
                f'Move command {self._command_id} failed with error: {self._feedbacks[-1].message}')

    def pause(self) -> None:
        self._robot.pause()

    def resume(self) -> None:
        self._robot.resume()

    def stop(self) -> None:
        self._robot.stop()
