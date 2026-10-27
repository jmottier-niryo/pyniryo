from dataclasses import dataclass, field
from datetime import datetime
from typing import Any

from strenum import StrEnum

from .._internal import transport_models
from .network import InterfaceType


@dataclass
class NtpConfig:
    """
    Represents the device's NTP (Network Time Protocol) configuration.

    :param enabled: Whether the device is configured to synchronize with an NTP server.
    :param synchronized: Whether the device's clock is currently synchronized with an NTP server.
    :param servers: The list of configured NTP servers.
    :param pools: The list of configured NTP pools.
    :param last_sync: The last time the device synchronized its clock.
    """
    enabled: bool
    synchronized: bool
    servers: list[str]
    pools: list[str]
    last_sync: datetime

    @classmethod
    def from_transport_model(cls, model: transport_models.s.NTPConfig) -> 'NtpConfig':
        return cls(
            enabled=model.enabled,
            synchronized=model.synchronized,
            servers=model.servers,
            pools=model.pools,
            last_sync=model.last_sync,
        )


@dataclass
class DeviceSoftware:
    """
    The embedded software release the robot runs. One release covers everything the robot boots.

    :param release_id: Opaque identifier of the installed release, the value an update names.
    :param version: Version of the release as its publisher named it, None when the release carries none.
    :param released_at: Publication date of the release, None when the release carries none.
    :param updated_at: When the robot last changed release, None until it changes one.
    """
    release_id: str
    version: str | None = None
    released_at: datetime | None = None
    updated_at: datetime | None = None

    @classmethod
    def from_transport_model(cls, model: transport_models.s.DeviceSoftware) -> 'DeviceSoftware':
        return cls(release_id=model.release_id,
                   version=model.version,
                   released_at=model.released_at,
                   updated_at=model.updated_at)


@dataclass
class DeviceIdentity:
    """
    Identity of the robot: what names it, which product it is, and which embedded software it runs. Every field
    but ``device_id`` is None when its source has nothing to give.

    :param device_id: Identifier of the robot, fixed for the life of the device.
    :param device_name: Name the device manager gives the robot.
    :param model: Robot model, as configured.
    :param serial_number: Serial number of the arm base board, None until the firmware has reported it.
    :param software: The embedded software release the robot runs.
    """
    device_id: str
    device_name: str | None = None
    model: str | None = None
    serial_number: str | None = None
    software: DeviceSoftware | None = None

    @classmethod
    def from_transport_model(cls, model: transport_models.s.DeviceID) -> 'DeviceIdentity':
        return cls(device_id=model.device_id,
                   device_name=model.device_name,
                   model=model.model,
                   serial_number=model.serial_number,
                   software=DeviceSoftware.from_transport_model(model.software) if model.software else None)


@dataclass
class DeviceVersion:
    """
    Identity of the API binary, stamped in at build time. This is the build of one service inside the release, not
    the release: see :attr:`DeviceIdentity.software` for that.

    :param api_version: Git tag the binary was built from, without its leading ``v``, ``dev`` when it carried none.
    :param api_commit: Commit the binary was built from, ``unknown`` when it carried none.
    """
    api_version: str
    api_commit: str

    @classmethod
    def from_transport_model(cls, model: transport_models.s.DeviceVersion) -> 'DeviceVersion':
        return cls(api_version=model.apid_version, api_commit=model.apid_commit)


class DeviceStatusValue(StrEnum):
    """
    Exactly one state, resolved in this order: a robot both held by a workstation and in emergency stop reports the
    stop.

    - UNKNOWN: no safety state to read, the firmware container is not running or none has been received yet.
    - EMERGENCY_STOP: an emergency stop input is active.
    - PROTECTIVE_STOP: a recoverable stop is active.
    - RUNNING: a program execution or a move command is under way, a paused one included.
    - IN_USE: a workstation holds the control session and the robot is not moving.
    - AVAILABLE: none of the above, the robot is free. A disarmed robot is available: that is its state at rest.
    """
    AVAILABLE = 'available'
    RUNNING = 'running'
    IN_USE = 'in_use'
    PROTECTIVE_STOP = 'protective_stop'
    EMERGENCY_STOP = 'emergency_stop'
    UNKNOWN = 'unknown'


@dataclass
class DeviceInterface:
    """
    A network interface the robot is connected on, and the address it holds there.

    :param id: Identifier of the interface, the one the network component takes.
    :param type: Ethernet or wifi.
    :param ip: IPv4 address the interface holds.
    """
    id: str
    type: InterfaceType
    ip: str

    @classmethod
    def from_transport_model(cls, model: transport_models.s.DeviceInterface) -> 'DeviceInterface':
        return cls(id=model.id, type=InterfaceType.from_transport_model(model.type), ip=model.ip)


@dataclass
class DeviceExecution:
    """
    The program a robot is running right now, a paused one included, cut down to what names it.

    :param program_id: The ID of the program.
    :param program_name: The name of the program.
    :param project_name: The project the program belongs to, None for a program in no project.
    """
    program_id: str
    program_name: str
    project_name: str | None = None

    @classmethod
    def from_transport_model(cls, model: transport_models.s.DeviceExecution) -> 'DeviceExecution':
        return cls(program_id=str(model.program_id), program_name=model.program_name, project_name=model.project_name)


@dataclass
class DeviceStatus:
    """
    Aggregated state of the robot, for a client that holds no session on it: whether the robot can be used right
    now, and which robot it is. Every field but ``status`` is None when its source has nothing to give.

    :param status: The one state the robot is in.
    :param client_id: MQTT client id of the workstation holding the control session, None while the robot is free.
    :param manual_mode: Only manual control modes are authorized; when False, only automatic modes are.
    :param armed: The robot is armed and accepts commands. Not readable from ``status``.
    :param device_name: Name the device manager gives the robot.
    :param model: Robot model, as configured.
    :param hostname: Fully qualified name the robot answers to on the local network.
    :param ip: Address the client dialled to reach the robot, None when it dialled a name.
    :param interfaces: Every interface the robot is connected on, with the address it holds there.
    :param execution: The program the robot is running, None when it runs none.
    """
    status: DeviceStatusValue
    client_id: str | None = None
    manual_mode: bool | None = None
    armed: bool | None = None
    device_name: str | None = None
    model: str | None = None
    hostname: str | None = None
    ip: str | None = None
    interfaces: list[DeviceInterface] = field(default_factory=list)
    execution: DeviceExecution | None = None

    @classmethod
    def from_transport_model(cls, model: transport_models.s.DeviceStatus) -> 'DeviceStatus':
        return cls(status=DeviceStatusValue(model.status.value),
                   client_id=model.client_id,
                   manual_mode=model.manual_mode,
                   armed=model.armed,
                   device_name=model.device_name,
                   model=model.model,
                   hostname=model.hostname,
                   ip=model.ip,
                   interfaces=[DeviceInterface.from_transport_model(i) for i in model.interfaces or []],
                   execution=DeviceExecution.from_transport_model(model.execution) if model.execution else None)


class DeviceUpdateState(StrEnum):
    """
    Where the robot stands on updating its embedded software.

    - IDLE: nothing waits to be installed and nothing is running.
    - CHECKING: the robot is asking its fleet for a newer release, and leaves this state on its own.
    - DOWNLOADING: the release asked for is being transferred, see ``progress``. The robot stays usable.
    - READY: the fleet publishes a newer release, which waits for :meth:`~pyniryo.nate.components.Device.start_update`.
    - APPLYING: the robot is installing a release. The API and the broker go silent for the minutes it takes.
    - SUCCESS: the release last asked for is the one installed.
    - ERROR: the last update did not go through, see ``error``.
    """
    IDLE = 'idle'
    CHECKING = 'checking'
    DOWNLOADING = 'downloading'
    READY = 'ready'
    APPLYING = 'applying'
    SUCCESS = 'success'
    ERROR = 'error'


@dataclass
class DeviceUpdateTarget:
    """
    The release an update is about. Every field but ``release_id`` comes from what the release itself carries.

    :param release_id: Identifier of the release, the value :meth:`~pyniryo.nate.components.Device.start_update` takes.
    :param version: Version of the release, as its publisher named it.
    :param released_at: Publication date of the release.
    :param size_bytes: Size of the release as its publisher measured it, not what this robot transfers.
    :param release_notes: What changed in the release, in the publisher's own words.
    """
    release_id: str
    version: str | None = None
    released_at: datetime | None = None
    size_bytes: int | None = None
    release_notes: str | None = None

    @classmethod
    def from_transport_model(cls, model: transport_models.s.DeviceUpdateTarget) -> 'DeviceUpdateTarget':
        return cls(release_id=model.release_id,
                   version=model.version,
                   released_at=model.released_at,
                   size_bytes=model.size_bytes,
                   release_notes=model.release_notes)

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> 'DeviceUpdateTarget':
        return cls.from_transport_model(transport_models.s.DeviceUpdateTarget.model_validate(data))


@dataclass
class DeviceUpdate:
    """
    Where the robot stands on updating its embedded software. The robot stays on the release it runs until
    :meth:`~pyniryo.nate.components.Device.start_update` asks for the newer one.

    A client cannot follow an installation to its end from this robot: the API and the broker are part of what a
    release replaces, so both go silent for the minutes the robot takes to come back. Read ``APPLYING``, lose the
    connection, and conclude on reconnection, where the state reads ``SUCCESS`` or ``ERROR``.

    :param state: Where the robot stands.
    :param stale: The robot has had no recent contact with its fleet, so ``target`` may be out of date.
    :param target: The release the update is about, None while idle and after a check that found nothing newer.
    :param progress: How much of the release has been transferred, in percent, while downloading.
    :param error: Why the last update did not go through, when ``state`` is ``ERROR``.
    """
    state: DeviceUpdateState
    stale: bool
    target: DeviceUpdateTarget | None = None
    progress: int | None = None
    error: str | None = None

    @classmethod
    def from_transport_model(cls, model: transport_models.s.DeviceUpdate) -> 'DeviceUpdate':
        return cls(state=DeviceUpdateState(model.state.value),
                   stale=model.stale,
                   target=DeviceUpdateTarget.from_transport_model(model.target) if model.target else None,
                   progress=model.progress,
                   error=model.error)

    @classmethod
    def from_a_transport_model(cls, model: transport_models.a.DeviceUpdateState) -> 'DeviceUpdate':
        return cls(state=DeviceUpdateState(model.state.value),
                   stale=model.stale,
                   target=DeviceUpdateTarget.from_dict(model.target) if model.target else None,
                   progress=model.progress,
                   error=model.error)
