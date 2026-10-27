from dataclasses import dataclass
from enum import Enum
from uuid import UUID

from .._internal import transport_models

from .common import ExecutorStatus


@dataclass
class PID:
    """
    Represents PID controller gains.
    
    :param p: Proportional gain.
    :param i: Integral gain.
    :param d: Derivative gain.
    :param ff: Feed-forward gain.
    :param max_i: Maximum integral term value.
    :param max_out: Maximum output value.
    """
    p: float
    i: float
    d: float
    ff: float
    max_i: float
    max_out: float

    @classmethod
    def from_transport_model(cls, model: transport_models.s.PIDGains) -> 'PID':
        return cls(p=model.p, i=model.i, d=model.d, ff=model.ff, max_i=model.max_i, max_out=model.max_out)

    def to_transport_model(self) -> transport_models.s.PIDGains:
        return transport_models.s.PIDGains(p=self.p,
                                           i=self.i,
                                           d=self.d,
                                           ff=self.ff,
                                           max_i=self.max_i,
                                           max_out=self.max_out)


@dataclass
class JointConfiguration:
    """
    Represents the configuration of a single robot joint.
    
    :param name: The name of the joint.
    :param type: The type of joint (e.g., revolute, prismatic).
    :param position_limit_min: Minimum position limit.
    :param position_limit_max: Maximum position limit.
    :param velocity_limit: Maximum velocity.
    :param acceleration_limit: Maximum acceleration.
    :param effort_limit: Maximum effort/torque.
    :param pid_position: PID gains for position control (optional).
    :param pid_velocity: PID gains for velocity control (optional).
    """
    name: str
    type: str
    position_limit_min: float
    position_limit_max: float
    velocity_limit: float
    acceleration_limit: float | None
    effort_limit: float
    pid_position: PID | None
    pid_velocity: PID | None

    @classmethod
    def from_transport_model(cls, model: transport_models.s.JointConfig) -> 'JointConfiguration':
        return cls(
            name=model.name,
            type=model.type,
            position_limit_min=model.position_limit_min,
            position_limit_max=model.position_limit_max,
            velocity_limit=model.velocity_limit,
            acceleration_limit=model.acceleration_limit,
            effort_limit=model.effort_limit,
            pid_position=PID.from_transport_model(model.pid_position) if model.pid_position else None,
            pid_velocity=PID.from_transport_model(model.pid_velocity) if model.pid_velocity else None,
        )

    def to_transport_model(self) -> transport_models.s.JointConfig:
        return transport_models.s.JointConfig(
            name=self.name,
            type=self.type,
            position_limit_min=self.position_limit_min,
            position_limit_max=self.position_limit_max,
            velocity_limit=self.velocity_limit,
            acceleration_limit=self.acceleration_limit,
            effort_limit=self.effort_limit,
            pid_position=self.pid_position.to_transport_model() if self.pid_position else None,
            pid_velocity=self.pid_velocity.to_transport_model() if self.pid_velocity else None,
        )


@dataclass
class RobotConfiguration:
    """
    Represents the complete configuration of a robot.
    
    :param name: The name of the robot.
    :param n_joint: The number of joints.
    :param joints: List of configurations for each joint.
    """
    name: str
    model: str
    tool_model: str
    hardware_mode: str
    n_joint: int
    joints: list[JointConfiguration]

    @classmethod
    def from_transport_model(cls, model: transport_models.s.RobotConfig) -> 'RobotConfiguration':
        return cls(
            name=model.name,
            model=model.model,
            tool_model=model.tool_model,
            hardware_mode=model.hardware_mode,
            n_joint=model.num_joints,
            joints=[JointConfiguration.from_transport_model(jc) for jc in model.joints],
        )

    def to_transport_model(self) -> transport_models.s.RobotConfig:
        return transport_models.s.RobotConfig(
            name=self.name,
            model=self.model,
            tool_model=self.tool_model,
            hardware_mode=self.hardware_mode,
            num_joints=self.n_joint,
            joints=[jc.to_transport_model() for jc in self.joints],
        )


class ControlMode(Enum):
    """
    Enumeration of robot control modes.
    
    - TRAJECTORY: Follow pre-planned trajectories.
    - JOG: Manual jogging control.
    - SPEED: Direct speed control.
    """
    TRAJECTORY = 1
    JOG = 2
    SPEED = 3

    @classmethod
    def from_transport_model(cls, model: transport_models.s.ControlMode) -> 'ControlMode':
        return cls(int(model.mode.value))

    @classmethod
    def from_a_transport_model(cls, model: transport_models.a.ControlMode) -> 'ControlMode':
        return cls(int(model.mode.value))

    def to_transport_model(self) -> transport_models.s.ControlMode:
        return transport_models.s.ControlMode(mode_name=transport_models.s.ModeName(self.name.lower()),
                                              mode=transport_models.s.Mode(self.value))


@dataclass
class TrajectoryExecutorStatus:
    command_id: UUID | None
    status: ExecutorStatus

    @classmethod
    def from_transport_model(cls, model: transport_models.s.TrajectoryExecutorStatus) -> 'TrajectoryExecutorStatus':
        return cls(command_id=model.command_id, status=ExecutorStatus.from_transport_model(model.status))

    def to_transport_model(self) -> transport_models.s.TrajectoryExecutorStatus:
        return transport_models.s.TrajectoryExecutorStatus(command_id=self.command_id,
                                                           status=self.status.to_transport_model())


@dataclass
class Color:
    """
    A color, one component per channel, each from 0 to 255.
    """
    red: int
    green: int
    blue: int


@dataclass
class LedRing:
    """
    What the light ring at the base of the robot shows. The robot picks the state to display and drives the ring,
    this carries that rendering as is, so a client draws what it receives and needs no rule of its own.

    :param animation: How the ring lights up: ``off``, ``solid``, ``blink``, ``breath`` or ``chase`` today. A robot
        may add a value, always in lowercase; a client that does not know one draws a steady light in the given color.
    :param color: The color of the light.
    :param brightness: 0 for dark, 255 for the brightest the ring produces.
    :param period_ms: Duration of one cycle of the animation in milliseconds, None for ``off`` and ``solid``.
    """
    animation: str
    color: Color
    brightness: int
    period_ms: int | None = None

    @classmethod
    def from_transport_model(cls, model: transport_models.a.LedRing) -> 'LedRing':
        return cls(animation=model.animation,
                   color=Color(red=int(model.color['red']), green=int(model.color['green']),
                               blue=int(model.color['blue'])),
                   brightness=model.brightness,
                   period_ms=model.period_ms)
