from datetime import datetime
from typing import Callable

from .base_api_component import BaseAPIComponent
from .._internal import paths_gen, transport_models, topics_gen
from ..exceptions import PyNiryoError
from ..models import NtpConfig, DeviceIdentity, DeviceVersion, DeviceStatus, DeviceUpdate, Unsubscribe
from .._internal.utils import ensure_utc


DeviceUpdateCallback = Callable[[DeviceUpdate], None]


class Device(BaseAPIComponent):
    """
    Device component for accessing robot device information and control.

    This component provides methods for:
    - Getting the device identity, version and aggregated status
    - Checking device health and readiness
    - Setting the clock and the NTP configuration
    - Following and starting updates of the embedded software
    - Rebooting or shutting down the device
    - Reading the API specifications the robot serves
    """

    def id(self) -> str:
        """
        Get the device ID of the robot.
        :return: The device ID of the robot.
        """
        resp = self._http_client.get(paths_gen.Device.GET_DEVICE_ID, transport_models.s.DeviceID)
        return resp.device_id

    def get_identity(self) -> DeviceIdentity:
        """
        Get the identity of the robot: its id, name, model, arm serial number and the software release it runs.

        :return: The identity of the robot.
        """
        resp = self._http_client.get(paths_gen.Device.GET_DEVICE_ID, transport_models.s.DeviceID)
        return DeviceIdentity.from_transport_model(resp)

    def is_healthy(self) -> bool:
        """
        Check if the robot device is healthy.
        
        :return: True if the robot is healthy, False otherwise.
        """
        try:
            self._http_client.get(paths_gen.Device.HEALTH_CHECK, transport_models.HealthCheckResponse)
        except PyNiryoError:
            return False
        return True

    def is_ready(self) -> bool:
        """
        Check if the API service is ready. This says nothing about the arm: see :meth:`get_status` for whether the
        robot can be used right now.

        :return: True if the service is ready, False otherwise.
        """
        resp = self._http_client.get(paths_gen.Device.READINESS_CHECK, transport_models.ReadinessCheckResponse)
        return resp.ready is not False

    def get_version(self) -> DeviceVersion:
        """
        Get the version and commit of the API binary. Answers without a token.

        :return: The version of the API binary.
        """
        resp = self._http_client.get(paths_gen.Device.GET_DEVICE_VERSION, transport_models.s.DeviceVersion)
        return DeviceVersion.from_transport_model(resp)

    def get_status(self) -> DeviceStatus:
        """
        Get the aggregated status of the robot: whether it can be used right now, who holds it, and which robot it
        is. Answers without a token.

        :return: The aggregated status of the robot.
        """
        resp = self._http_client.get(paths_gen.Device.GET_DEVICE_STATUS, transport_models.s.DeviceStatus)
        return DeviceStatus.from_transport_model(resp)

    def get_update(self) -> DeviceUpdate:
        """
        Get where the robot stands on updating its embedded software. The robot answers from what it last knew and
        never waits on the network: a robot cut off from its fleet answers with ``stale`` set. The same state goes
        out on :meth:`on_update_state`.

        :return: The update state.
        :raises ServerError: If this robot does not manage its own software (``code == 'updates_unavailable'``). A
            client meeting this stops offering updates rather than retrying.
        """
        resp = self._http_client.get(paths_gen.Device.GET_DEVICE_UPDATE, transport_models.s.DeviceUpdate)
        return DeviceUpdate.from_transport_model(resp)

    def start_update(self, release_id: str) -> None:
        """
        Install a release of the embedded software. The call returns as soon as the robot has accepted the order.
        The state then reads ``DOWNLOADING``, with a progress, then ``APPLYING``, during which the robot stops
        answering for minutes. Conclude on reconnection, where the state reads ``SUCCESS`` or ``ERROR``.

        Nothing queues: a refused call has to be made again.

        :param release_id: The release to install, the ``target.release_id`` of :meth:`get_update`. Asking for a
            release the fleet no longer offers is refused rather than served a different one.
        :raises ClientError: If a program runs or the arm moves (``code == 'execution_running'``), an update is
            already under way (``code == 'update_running'``), or the release is not the one the fleet offers
            (``code == 'release_not_available'``).
        :raises ServerError: If the robot cannot go through its fleet (``code`` is ``'updates_unavailable'`` or
            ``'fleet_unreachable'``).
        """
        self._http_client.post(paths_gen.Device.START_DEVICE_UPDATE,
                               transport_models.EmptyPayload,
                               transport_models.s.DeviceUpdateRequest(release_id=release_id))

    def check_update(self) -> None:
        """
        Ask the fleet now for a newer release, instead of waiting for the robot's next question. The call returns at
        once and the answer lands on :meth:`get_update`: the state reads ``CHECKING`` until the robot knows. A call
        made while the robot is already asking asks nothing again.

        :raises ServerError: If the robot cannot go through its fleet (``code`` is ``'updates_unavailable'`` or
            ``'fleet_unreachable'``).
        """
        self._http_client.post(paths_gen.Device.CHECK_DEVICE_UPDATE,
                               transport_models.EmptyPayload,
                               transport_models.EmptyPayload())

    def on_update_state(self, callback: DeviceUpdateCallback) -> Unsubscribe:
        """
        Set the callback to call when the update state changes. The state is retained, so the callback receives the
        current one as soon as the subscription is in place. The broker goes silent while a release is applied:
        reconnect and read the verdict then.

        :param callback: The callback function that takes the new :class:`DeviceUpdate`.
        """

        def internal_callback(_: str, state: transport_models.a.DeviceUpdateState) -> None:
            callback(DeviceUpdate.from_a_transport_model(state))

        return self._mqtt_client.subscribe(topics_gen.Update.STATE, internal_callback,
                                           transport_models.a.DeviceUpdateState)

    def get_openapi_spec(self) -> str:
        """
        Get the OpenAPI specification of the REST API the robot serves, in YAML.
        """
        return self._http_client.get_text(paths_gen.Documentation.GET_OPEN_API_SPEC, accept='application/yaml')

    def get_asyncapi_spec(self) -> str:
        """
        Get the AsyncAPI specification of the MQTT API the robot serves, in YAML.
        """
        return self._http_client.get_text(paths_gen.Documentation.GET_ASYNC_API_SPEC, accept='application/yaml')

    def get_openapi_ui(self) -> str:
        """
        Get the interactive documentation page of the REST API, as HTML.
        """
        return self._http_client.get_text(paths_gen.Documentation.GET_OPEN_APIUI, accept='text/html')

    def get_asyncapi_ui(self) -> str:
        """
        Get the interactive documentation page of the MQTT API, as HTML.
        """
        return self._http_client.get_text(paths_gen.Documentation.GET_ASYNC_APIUI, accept='text/html')

    def get_datetime(self) -> datetime:
        """
        Get the current date and time of the device.

        :return: The device's current datetime.
        """
        resp = self._http_client.get(paths_gen.Device.GET_DEVICE_DATETIME, transport_models.s.DeviceDatetime)
        return resp.datetime

    def set_datetime(self, dt: datetime) -> None:
        """
        Set the device's date and time.

        Note: This will fail if NTP synchronization is enabled.

        :param dt: The datetime to set on the device, timezone-aware.
        :raises DataValidationError: If ``dt`` carries no timezone.
        """
        self._http_client.post(paths_gen.Device.SET_DEVICE_DATETIME,
                               transport_models.EmptyPayload,
                               transport_models.s.DeviceDatetime(datetime=ensure_utc(dt)))

    def get_ntp_config(self) -> NtpConfig:
        """
        Get the device's NTP configuration.

        :return: The current NTP configuration.
        """
        resp = self._http_client.get(paths_gen.Device.GET_DEVICE_NTP_CONFIG, transport_models.s.NTPConfig)
        return NtpConfig.from_transport_model(resp)

    def set_ntp_config(self,
                       enabled: bool | None = None,
                       servers: list[str] | None = None,
                       pools: list[str] | None = None) -> NtpConfig:
        """
        Update the device's NTP configuration. The robot takes the whole configuration at once, so a parameter left
        to None is read from the current configuration first and sent back unchanged.

        :param enabled: Whether the device synchronizes its clock with NTP.
        :param servers: The NTP servers to synchronize with.
        :param pools: The NTP pools to get servers from.
        :return: The updated NTP configuration.
        """
        if enabled is None or servers is None or pools is None:
            current = self.get_ntp_config()
            enabled = current.enabled if enabled is None else enabled
            servers = current.servers if servers is None else servers
            pools = current.pools if pools is None else pools
        resp = self._http_client.patch(paths_gen.Device.SET_DEVICE_NTP_CONFIG,
                                       transport_models.s.NTPConfig,
                                       transport_models.NTPConfigUpdate(enabled=enabled, servers=servers, pools=pools))
        return NtpConfig.from_transport_model(resp)

    def sync_clock(self) -> None:
        """
        Synchronize the device's clock with the configured NTP server(s).
        """
        self._http_client.post(paths_gen.Device.SYNC_DEVICE_CLOCK,
                               transport_models.EmptyPayload,
                               transport_models.EmptyPayload())

    def clear_system(self) -> None:
        """
        Free disk space by pruning unused Docker resources. The call returns once the prune has started.
        """
        self._http_client.post(paths_gen.Device.CLEAR_SYSTEM,
                               transport_models.EmptyPayload,
                               transport_models.EmptyPayload())

    def reboot(self) -> None:
        """
        Reboot the robot device. The call returns once the robot has accepted the order.

        Note: The connection will be lost during the reboot process.
        """
        self._http_client.post(paths_gen.Device.REBOOT,
                               transport_models.EmptyPayload,
                               transport_models.EmptyPayload())

    def shutdown(self) -> None:
        """
        Shutdown the robot device.
        
        Note: The device will need to be manually powered on after shutdown.
        """
        self._http_client.post(paths_gen.Device.SHUTDOWN,
                               transport_models.EmptyPayload,
                               transport_models.EmptyPayload())
