from dataclasses import dataclass
from typing import Any

from strenum import StrEnum

from .._internal import transport_models

_TransportFirmwareStatus = transport_models.s.Status1 | transport_models.a.FirmwareStateStatus


class FirmwareStatus(StrEnum):
    """
    Lifecycle of the base node, and not the armed state, which is reported separately by
    :class:`SafetyState`.

    - ACTIVE: the base node is active and the robot accepts commands.
    - DISARMED: the base node is down and the safety module reports the robot as not armed.
      This is the steady state after an emergency stop, and the state a robot boots in
      until the operator presses the initialization button.
    - INACTIVE: the base node is neither active nor in error, and nothing says the robot is
      disarmed. Either no safety state has been received yet, the robot is arming, or the
      firmware container is not running.
    - ERROR: the base node reports a fault. Takes precedence over every other value.
    """
    ACTIVE = 'active'
    DISARMED = 'disarmed'
    INACTIVE = 'inactive'
    ERROR = 'error'

    @classmethod
    def from_transport_model(cls, model: _TransportFirmwareStatus) -> 'FirmwareStatus':
        return cls(model.value)


@dataclass
class SafetyState:
    """
    State reported by the safety module. None when no safety state has been received yet,
    and when the firmware container is not running, since the last state received is then
    stale.

    :param armed: The robot is ready to be commanded. This is the source of truth for the
        armed state, whatever the firmware status says alongside it.
    :param manual_mode: Only manual control modes are authorized; when False, only
        automatic modes are.
    :param movement_authorized: Motion is currently authorized. When False, the robot
        decelerates to a stop and holds position.
    :param protective_stop_active: A recoverable stop is active. The current execution
        pauses and resumes once movement is authorized again.
    :param emergency_stop_active: An emergency stop input is active.
    :param max_velocity_factor: Safety-imposed velocity ceiling.
    """
    armed: bool
    manual_mode: bool
    movement_authorized: bool
    protective_stop_active: bool
    emergency_stop_active: bool
    max_velocity_factor: float

    @classmethod
    def from_transport_model(cls, model: transport_models.s.SafetyState) -> 'SafetyState':
        return cls(
            armed=model.armed,
            manual_mode=model.manual_mode,
            movement_authorized=model.movement_authorized,
            protective_stop_active=model.protective_stop_active,
            emergency_stop_active=model.emergency_stop_active,
            max_velocity_factor=model.max_velocity_factor,
        )

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> 'SafetyState':
        return cls(
            armed=data['armed'],
            manual_mode=data['manual_mode'],
            movement_authorized=data['movement_authorized'],
            protective_stop_active=data['protective_stop_active'],
            emergency_stop_active=data['emergency_stop_active'],
            max_velocity_factor=data['max_velocity_factor'],
        )


@dataclass
class FirmwareState:
    """
    State of the robot firmware.

    :param status: Lifecycle of the base node.
    :param container_status: The status of the ros-robot container (running, stopped,
        not_found, unknown). Anything other than "running" forces ``status`` to INACTIVE
        and omits ``safety``.
    :param safety: State reported by the safety module, if any.
    """
    status: FirmwareStatus
    container_status: str
    safety: SafetyState | None

    @classmethod
    def from_transport_model(cls, model: transport_models.s.FirmwareState) -> 'FirmwareState':
        return cls(
            status=FirmwareStatus.from_transport_model(model.status),
            container_status=model.container_status,
            safety=SafetyState.from_transport_model(model.safety) if model.safety else None,
        )

    @classmethod
    def from_a_transport_model(cls, model: transport_models.a.FirmwareState) -> 'FirmwareState':
        return cls(
            status=FirmwareStatus.from_transport_model(model.status),
            container_status=model.container_status,
            safety=SafetyState.from_dict(model.safety) if model.safety else None,
        )


_TransportPlcSafetyState = transport_models.s.PlcSafetyState | transport_models.a.PlcSafetyState


@dataclass
class PlcSafetyState:
    """
    Raw state of the PLC safety chain.
    """
    robot_initialized: bool
    stop_request: bool
    pause_request: bool
    automatic_move_authorization: bool
    manual_move_authorization: bool
    wait_for_initialization: bool
    wait_for_automatic_restart: bool
    manual_mode: bool
    io_emergency_stop_active: bool
    io_safety_stop_active: bool
    io_initialization_button_active: bool
    io_automatic_restart_button_active: bool
    io3_positions_button_active: bool
    io_emergency_stop_output_active: bool
    io_initialization_output_active: bool

    @classmethod
    def from_transport_model(cls, model: _TransportPlcSafetyState) -> 'PlcSafetyState':
        return cls(
            robot_initialized=model.robot_initialized,
            stop_request=model.stop_request,
            pause_request=model.pause_request,
            automatic_move_authorization=model.automatic_move_authorization,
            manual_move_authorization=model.manual_move_authorization,
            wait_for_initialization=model.wait_for_initialization,
            wait_for_automatic_restart=model.wait_for_automatic_restart,
            manual_mode=model.manual_mode,
            io_emergency_stop_active=model.io_emergency_stop_active,
            io_safety_stop_active=model.io_safety_stop_active,
            io_initialization_button_active=model.io_initialization_button_active,
            io_automatic_restart_button_active=model.io_automatic_restart_button_active,
            io3_positions_button_active=model.io3_positions_button_active,
            io_emergency_stop_output_active=model.io_emergency_stop_output_active,
            io_initialization_output_active=model.io_initialization_output_active,
        )

    def to_transport_model(self) -> transport_models.s.PlcSafetyState:
        return transport_models.s.PlcSafetyState(
            robot_initialized=self.robot_initialized,
            stop_request=self.stop_request,
            pause_request=self.pause_request,
            automatic_move_authorization=self.automatic_move_authorization,
            manual_move_authorization=self.manual_move_authorization,
            wait_for_initialization=self.wait_for_initialization,
            wait_for_automatic_restart=self.wait_for_automatic_restart,
            manual_mode=self.manual_mode,
            io_emergency_stop_active=self.io_emergency_stop_active,
            io_safety_stop_active=self.io_safety_stop_active,
            io_initialization_button_active=self.io_initialization_button_active,
            io_automatic_restart_button_active=self.io_automatic_restart_button_active,
            io3_positions_button_active=self.io3_positions_button_active,
            io_emergency_stop_output_active=self.io_emergency_stop_output_active,
            io_initialization_output_active=self.io_initialization_output_active,
        )
