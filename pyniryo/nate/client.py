import logging
import threading
import uuid
from datetime import datetime, timedelta, timezone
from typing import Callable, Any, TypeVar

from ._internal import paths_gen, transport_models
from ._internal.http import HttpClient
from ._internal.mqtt import MqttClient
from ._internal.signal_handler import SocketSignalHandler
from ._internal.tls import DeviceIdentity, build_context, load_roots
from ._internal.utils import fetch_from_env, get_token_expiration
from .components import Auth, Users, Robot, Device, Firmware, Network, Programs, Projects, MotionPlanner, Metrics, IO
from . import const
from .exceptions import TlsError

logger = logging.getLogger(__name__)


class Nate:
    """
    Main client class to interact with the Nate API. It provides access to all the components of the API,
    such as authentication, users, robot control, device information, and program management.
    """

    auth: Auth
    users: Users
    robot: Robot
    device: Device
    firmware: Firmware
    network: Network
    programs: Programs
    projects: Projects
    motion_planner: MotionPlanner
    metrics: Metrics
    io: IO

    def __init__(self, hostname: str = '', login: tuple[str, str] = ('', ''), device_id: str = ''):
        """
        Initialize a client to communicate with the Nate API.
        
        :param hostname: The hostname of the Nate API. It can be an IP address or a domain name.
            If None, retrieve it from the environment variable NATE_HOSTNAME. If the environment variable is not set, use `HOSTNAME`.
        :param login: A tuple containing the username and password to use for authentication. Omitted if using an auth token.
            If None, retrieve them from the environment variables NATE_USERNAME and NATE_PASSWORD.
        :param device_id: The identifier of the robot expected at ``hostname``, as its certificate names it. An address
            changes from one network to the next and may be taken by another robot; with a device ID, a robot that is not
            this one is refused before any credential is sent. If empty, retrieve it from the environment variable
            NATE_DEVICE_ID; if that is not set either, the first robot reached binds the client for its lifetime.
        """
        hostname = fetch_from_env('NATE_HOSTNAME', str) or hostname or const.HOSTNAME
        expected_device_id = fetch_from_env('NATE_DEVICE_ID', str) or device_id or None

        if len(login) != 2:
            raise ValueError("authentication with username and password requires both username and password")
        username, password = login
        username = fetch_from_env('NATE_USERNAME', str) or username
        password = fetch_from_env('NATE_PASSWORD', str) or password
        if not username or not password:
            raise ValueError("both username and password must be provided for authentication")

        # Advanced options, not exposed in the constructor.
        http_port = _env_or('NATE_HTTP_PORT', int, const.HTTP_PORT)
        http_timeout = _env_or('NATE_HTTP_TIMEOUT', float, const.HTTP_TIMEOUT)
        # roots to trust on top of the Niryo roots shipped with the package
        ca_file = fetch_from_env('NATE_CA_FILE', str)

        mqtt_port = _env_or('NATE_MQTT_PORT', int, const.MQTT_PORT)

        # how many retries should be done when an http call or a mqtt connect fails
        n_retry = _env_or('NATE_RETRY', int, const.N_RETRY)
        # factor to multiply the retry delay with after each retry
        retry_delay_factor = _env_or('NATE_RETRY_DELAY_FACTOR', float, const.RETRY_DELAY_FACTOR)

        # validity duration of the renewed tokens
        token_validity_str = fetch_from_env('NATE_TOKEN_VALIDITY', str) or const.TOKEN_VALIDITY
        execution_token = fetch_from_env('NATE_EXECUTION_TOKEN', str)

        if token_validity_str.endswith('d'):
            token_validity = timedelta(days=float(token_validity_str[:-1]))
        elif token_validity_str.endswith('h'):
            token_validity = timedelta(hours=float(token_validity_str[:-1]))
        elif token_validity_str.endswith('s'):
            token_validity = timedelta(seconds=float(token_validity_str[:-1]))
        else:
            raise ValueError(f'Unsupported token validity type {token_validity_str}.')

        self._correlation_id = fetch_from_env('NATE_CORRELATION_ID', str) or uuid.uuid4().hex

        ##########################################################################################
        ## Bootstrap: fetch all needed data to properly initiate the client and its components. ##
        ##########################################################################################

        # the robot is who its certificate says: the expected device, or else the first one reached
        identity = DeviceIdentity(expected=expected_device_id)
        ssl_context = build_context(load_roots(ca_file), identity)

        self._http_client = HttpClient(hostname,
                                       http_port,
                                       ssl_context=ssl_context,
                                       http_timeout=http_timeout,
                                       n_retry=n_retry,
                                       retry_delay_factor=retry_delay_factor)
        if execution_token:
            self._http_client.set_header('Execution-Token', execution_token)

        # Token
        def provide_token() -> str:
            return self._http_client.post(
                paths_gen.Authentication.LOGIN,
                transport_models.s.Token,
                transport_models.s.Login(login=username,
                                         password=password,
                                         expires_at=datetime.now(timezone.utc) + token_validity)).token

        token = provide_token()

        self._http_client.set_token(token)

        # Device ID
        resp = self._http_client.get(paths_gen.Device.GET_DEVICE_ID, transport_models.s.DeviceID)
        device_id = resp.device_id
        if identity.expected is not None and identity.expected != device_id:
            self._http_client.disconnect()
            raise TlsError(f'The robot certificate names device {identity.expected}, the API reports device '
                           f'{device_id}.')
        identity.bind(device_id)

        self._mqtt_client: MqttClient = MqttClient(hostname,
                                                   mqtt_port,
                                                   device_id=device_id,
                                                   correlation_id=self._correlation_id,
                                                   ssl_context=ssl_context,
                                                   n_retry=n_retry,
                                                   retry_delay_factor=retry_delay_factor)
        self._mqtt_client.set_token(token)

        self._token_renewer = _TokenRenewer(
            provide_token,
            [self._http_client.set_token, self._mqtt_client.set_token],
            initial_token=token,
        )
        self._token_renewer.start()

        self.auth = Auth(self._http_client, self._mqtt_client, self._correlation_id)
        self.users = Users(self._http_client, self._mqtt_client, self._correlation_id)
        self.robot = Robot(self._http_client, self._mqtt_client, self._correlation_id)
        self.device = Device(self._http_client, self._mqtt_client, self._correlation_id)
        self.firmware = Firmware(self._http_client, self._mqtt_client, self._correlation_id)
        self.network = Network(self._http_client, self._mqtt_client, self._correlation_id)
        self.programs = Programs(self._http_client, self._mqtt_client, self._correlation_id)
        self.projects = Projects(self._http_client, self._mqtt_client, self._correlation_id)
        self.motion_planner = MotionPlanner(self._http_client, self._mqtt_client, self._correlation_id)
        self.metrics = Metrics(self._http_client, self._mqtt_client, self._correlation_id)
        self.io = IO(self._http_client, self._mqtt_client, self._correlation_id)

        self._signal_handler = None
        ipc_socket = fetch_from_env('NATE_IPC_SOCKET', str)
        if ipc_socket is not None:
            self._signal_handler = SocketSignalHandler.register(self, ipc_socket)

    def __enter__(self) -> 'Nate':
        return self

    def __exit__(self, type: Any, value: Any, traceback: Any) -> None:
        self.close()

    def close(self) -> None:
        """
        Clean up resources used by the client, and ensure that any background tasks are properly terminated.
        After calling this method, the client should not be used anymore.
        """
        self.auth.close()
        self.users.close()
        self.robot.close()
        self.device.close()
        self.firmware.close()
        self.network.close()
        self.programs.close()
        self.projects.close()
        self.motion_planner.close()
        self.metrics.close()
        self.io.close()

        self._token_renewer.stop()
        self._mqtt_client.disconnect()
        self._http_client.disconnect()

        if self._signal_handler is not None:
            self._signal_handler.unregister(self)


_T = TypeVar('_T', str, int, float, bool)


def _env_or(key: str, _type: type[_T], default: _T) -> _T:
    value = fetch_from_env(key, _type)
    return default if value is None else value


class _TokenRenewer:
    """
    Manages automatic token renewal in the background.

    This class runs a background timer that periodically renews authentication tokens
    to ensure continuous access to the API. A renewal that fails is retried after
    :attr:`RETRY_DELAY` seconds, so a robot that was unreachable for a while gets a fresh
    token once it answers again.
    """

    RETRY_DELAY = 30.0

    def __init__(self, provider: Callable[[], str], setters: list[Callable[[str], None]], initial_token: str) -> None:
        self._provider = provider
        self._setters = setters
        self._token = initial_token
        self._timer = threading.Timer(0, lambda: None)

    def start(self) -> None:
        """
        Start the token renewal timer.
        """
        self._setup_timer()

    def stop(self) -> None:
        """
        Stop the token renewal timer and cancel any pending renewals.
        """
        self._timer.cancel()

    def _setup_timer(self) -> None:
        self._timer.cancel()
        if self._token == '':
            self._run()
            return
        token_exp = get_token_expiration(self._token)
        interval = (token_exp - datetime.now()).total_seconds() * 0.9
        self._timer = threading.Timer(interval, self._run)
        self._timer.daemon = True
        logger.info(f'Next token renewal scheduled in {timedelta(seconds=interval)}.')
        self._timer.start()

    def _retry_later(self, step: Callable[[], None]) -> None:
        self._timer = threading.Timer(self.RETRY_DELAY, step)
        self._timer.daemon = True
        self._timer.start()

    def _run(self) -> None:
        try:
            self._token = self._provider()
        except Exception as e:
            logger.error(f'Token renewal failed, retrying in {self.RETRY_DELAY:.0f}s: {e}')
            self._retry_later(self._run)
            return
        self._apply(self._setters)

    def _apply(self, setters: list[Callable[[str], None]]) -> None:
        failed = []
        for setter in setters:
            try:
                setter(self._token)
            except Exception as e:
                logger.error(f'Error applying the renewed token to {setter}, retrying in {self.RETRY_DELAY:.0f}s: {e}')
                failed.append(setter)
        if failed:
            self._retry_later(lambda: self._apply(failed))
            return
        self._setup_timer()
