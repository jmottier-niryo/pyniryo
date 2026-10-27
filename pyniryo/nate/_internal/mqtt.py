import json
import re
import ssl
import string
import threading
import time
from enum import Enum
from typing import Dict, Callable, TypeVar, Sequence, Any, Mapping, Generic
from uuid import uuid4
import logging

from paho.mqtt.client import Client, MQTTv5, CallbackAPIVersion, MQTTMessage  # type: ignore[attr-defined]
from paho.mqtt.packettypes import PacketTypes
from paho.mqtt.properties import Properties
from pydantic import BaseModel, ValidationError
from strenum import StrEnum

from ..exceptions import get_msg_from_errors, TlsError
from ..models import Unsubscribe

logger = logging.getLogger(__name__)

T = TypeVar("T", bound=BaseModel)
Callback = Callable[[str, T], None]


class Wildcard(StrEnum):
    SINGLE_LEVEL = '+'
    MULTI_LEVEL = '#'


class Qos(Enum):
    AT_MOST_ONCE = 0
    AT_LEAST_ONCE = 1
    EXACTLY_ONCE = 2


class TopicFormatter(string.Formatter):

    def __init__(self, **kwargs: str):
        super().__init__()
        self.base_values = kwargs

    @staticmethod
    def has_placeholders_left(topic: str) -> bool:
        return '{' in topic or '}' in topic

    def get_value(self, key: int | str, args: Sequence[Any], kwargs: Mapping[str, Any]) -> Any:
        if key in self.base_values:
            if isinstance(key, int):
                raise TypeError('Integer placeholders are not supported in topic formatting.')
            return self.base_values[key]

        try:
            return super().get_value(key, args, kwargs)
        except KeyError as e:
            raise ValueError(f'Missing value for placeholder "{key}".') from e


class _Subscription(Generic[T]):

    def __init__(self, topic: str, payload_model: type[T]):
        self._topic = topic
        self._payload_model = payload_model
        self._callbacks_lock = threading.Lock()
        self._callbacks: Dict[int, Callback[T]] = {}
        self._next_ix = 0

    def internal_callback(self, _client: Client, _userdata: Any, message: MQTTMessage) -> None:
        received_topic = message.topic
        payload = message.payload
        if payload == b'':
            # Purge retained messages
            return

        try:
            model = self._payload_model.model_validate_json(payload)
        except ValidationError as e:
            logger.error(f'Dropping a message on topic {received_topic} that does not match '
                         f'{self._payload_model.__name__}: {get_msg_from_errors(e.errors())}')
            return
        with self._callbacks_lock:
            callbacks = list(self._callbacks.values())
        for callback in callbacks:
            try:
                callback(received_topic, model)
            except Exception:
                logger.exception(f'Error in callback {callback.__qualname__} for topic {self._topic}')

    def add(self, callback: Callback[T]) -> int:
        with self._callbacks_lock:
            # a monotonic index, since the count would hand out one that a removal freed
            cb_ix = self._next_ix
            self._next_ix += 1
            self._callbacks[cb_ix] = callback
        return cb_ix

    def rm(self, cb_ix: int) -> None:
        with self._callbacks_lock:
            if cb_ix not in self._callbacks:
                logger.warning(f'No callback at index {cb_ix} for topic {self._topic}.')
                return
            del self._callbacks[cb_ix]

    @property
    def is_stale(self) -> bool:
        return len(self._callbacks) == 0


class MqttClient:

    def __init__(self,
                 hostname: str,
                 port: int,
                 device_id: str,
                 correlation_id: str,
                 ssl_context: ssl.SSLContext,
                 n_retry: int = 0,
                 retry_delay_factor: float = 1.,
                 connack_timeout: float = 5.) -> None:
        """
        Initialize the MQTT client.
        :param hostname: The hostname of the MQTT broker.
        :param port: The port of the MQTT broker.
        :param ssl_context: The TLS context every connection is made with, see :func:`..tls.build_context`.
        :param n_retry: Number of connection retry attempts on initial connect.
        :param retry_delay_factor: Base delay in seconds for exponential backoff between retries.
        :param connack_timeout: How long to wait for the broker's answer to a connection, in seconds. A refused
            connection is raised at once; a connection still unanswered after this delay is left to reconnect on
            its own.
        """
        self._hostname = hostname
        self._port = port
        self._ssl_context = ssl_context
        self._n_retry = n_retry
        self._retry_delay_factor = retry_delay_factor
        self._connack_timeout = connack_timeout
        self._connack = threading.Event()
        self._connack_reason: Any = 0

        self._formatter = TopicFormatter(device_id=device_id)

        trace = re.sub(r'[^A-Za-z0-9_-]', '', correlation_id)[:8]
        self._client_id = f'pyniryo-{trace}.{uuid4().hex[:8]}'

        self._subscribers_lock = threading.Lock()
        self._subscribers: Dict[str, _Subscription[Any]] = {}

        self._mqtt_client = Client(callback_api_version=CallbackAPIVersion.VERSION2)
        self._started = False

    def _on_connect(self, client: Client, _userdata: Any, _flags: Any, reason_code: Any, _properties: Any) -> None:
        self._connack_reason = reason_code
        self._connack.set()
        if reason_code != 0:
            logger.error(f'MQTT connection failed with reason code: {reason_code}')
            return

        logger.info('MQTT connected. Re-subscribing to all active topics.')
        with self._subscribers_lock:
            for topic, subscription in self._subscribers.items():
                client.message_callback_add(topic, subscription.internal_callback)
                client.subscribe(topic, qos=2)

    def _on_disconnect(self, _client: Client, _userdata: Any, _flags: Any, reason_code: Any, _properties: Any) -> None:
        if reason_code != 0:
            logger.warning(f'MQTT connection lost unexpectedly (reason_code={reason_code}). '
                           f'Paho will attempt auto-reconnect.')

    def _init_client(self, token: str) -> None:
        # the previous client may be reconnecting under the same client id, so it is stopped whatever its state
        if self._started:
            self.disconnect()
        self._started = True

        self._connack.clear()
        self._connack_reason = 0
        self._mqtt_client = Client(callback_api_version=CallbackAPIVersion.VERSION2,
                                   client_id=self._client_id,
                                   protocol=MQTTv5)
        self._mqtt_client.on_message = lambda _, __, m: logger.error(
            f'Received message on topic {m.topic} with no '
            f'subscribers. Message ignored: {m.payload.decode()}')
        self._mqtt_client.on_connect = self._on_connect
        self._mqtt_client.on_disconnect = self._on_disconnect
        self._mqtt_client.username_pw_set(username='token', password=token)
        self._mqtt_client.tls_set_context(self._ssl_context)
        self._mqtt_client.tls_insecure_set(True)  # disable the inner client TLS check to use our own
        self._mqtt_client.reconnect_delay_set(min_delay=int(self._retry_delay_factor),
                                              max_delay=int(self._retry_delay_factor * (2**max(self._n_retry, 1))))

        last_error: Exception | None = None
        connection_properties = Properties(PacketTypes.CONNECT)  # type: ignore[no-untyped-call]
        connection_properties.SessionExpiryInterval = 30
        for attempt in range(1 + self._n_retry):
            try:
                self._mqtt_client.connect(self._hostname,
                                          self._port,
                                          clean_start=False,
                                          properties=connection_properties)
                last_error = None
                break
            except ssl.SSLCertVerificationError as e:
                self._started = False
                if isinstance(e, TlsError):
                    raise
                raise TlsError(e) from e
            except (OSError, ConnectionError) as e:
                last_error = e
                if attempt < self._n_retry:
                    delay = self._retry_delay_factor * (2**attempt)
                    logger.warning(f'MQTT connection attempt {attempt + 1} failed: {e}. Retrying in {delay:.1f}s...')
                    time.sleep(delay)

        if last_error is not None:
            raise ConnectionError(f'Failed to connect to MQTT broker at {self._hostname}:{self._port} '
                                  f'after {self._n_retry + 1} attempt(s).') from last_error

        self._mqtt_client.loop_start()

        if not self._connack.wait(self._connack_timeout):
            logger.warning(f'No answer from the MQTT broker at {self._hostname}:{self._port} within '
                           f'{self._connack_timeout:.0f}s, the client keeps trying in the background.')
        elif self._connack_reason != 0:
            self.disconnect()
            raise ConnectionError(f'The MQTT broker at {self._hostname}:{self._port} refused the connection: '
                                  f'{self._connack_reason}. A reason code 134 (0x86) means the token was refused.')

    def set_token(self, token: str) -> None:
        """
        Set the authentication token for the MQTT client.
        :param token: The token to set.
        """
        self._init_client(token)

    def disconnect(self) -> None:
        """
        Disconnect the MQTT client.
        """
        self._mqtt_client.loop_stop()
        self._mqtt_client.disconnect()

    def __del__(self) -> None:
        try:
            self._mqtt_client
        except AttributeError:
            return
        if self._mqtt_client.is_connected():
            self.disconnect()

    def publish(self, topic: str, data: BaseModel, qos: Qos = Qos.AT_MOST_ONCE) -> None:
        """
        Publish a message to a topic.
        :param topic: The topic to publish to.
        :param data: The data to publish.
        :param qos: The QoS level to publish with.
        """
        topic = self.format(topic)

        if self._formatter.has_placeholders_left(topic):
            raise ValueError(f'Topic "{topic}" still have unformatted placeholders.')

        encoded_data = json.dumps(data.model_dump(mode='json', exclude_none=True))

        self._mqtt_client.publish(topic, encoded_data, qos=qos.value)

    def subscribe(self, topic: str, callback: Callback[T], payload_model: type[T]) -> Unsubscribe:
        """
        Subscribe to a topic.
        :param topic: The topic to subscribe to.
        :param callback: The callback to call when a message is received.
        :param payload_model: The model to use for the payload. If None, no model is used.
        """
        if not issubclass(payload_model, BaseModel):
            raise TypeError(f'Invalid type {payload_model.__name__} for response model.')

        topic = self.format(topic)

        if self._formatter.has_placeholders_left(topic):
            raise ValueError(f'Topic "{topic}" still have unformatted placeholders.')

        with self._subscribers_lock:
            new_subscriber = topic not in self._subscribers

            if new_subscriber:
                self._subscribers[topic] = _Subscription(topic, payload_model)
                self._mqtt_client.message_callback_add(topic, self._subscribers[topic].internal_callback)

            cb_ix = self._subscribers[topic].add(callback)

        # subscribe at the end to ensure the callback is registered before receiving messages
        if new_subscriber:
            self._mqtt_client.subscribe(topic, qos=2)
        return lambda: self._unsubscribe(topic, cb_ix)

    def _unsubscribe(self, topic: str, cb_ix: int) -> None:
        with self._subscribers_lock:
            if topic not in self._subscribers:
                logger.warning(f'No subscribers for topic {topic}.')
                return

            self._subscribers[topic].rm(cb_ix)

            if self._subscribers[topic].is_stale:
                self._mqtt_client.unsubscribe(topic)
                del self._subscribers[topic]
                logger.debug(f'subscription to "{topic}" was stale and has been unsubscribed.')

    def format(self, topic: str, **kwargs: str | StrEnum) -> str:
        """
        Format a topic with the given keyword arguments.
        :param topic: The topic to format.
        :param kwargs: The keyword arguments to use for formatting.
        :return: The formatted topic.
        """
        return self._formatter.format(topic, **kwargs)


def get_level_from_wildcard(subscribed_topic: str, received_topic: str, wildcard: str = '+') -> list[str]:
    """
    Get the wildcarded levels by comparing the subscribed topic and the received topic.
    :param subscribed_topic: The topic subscribed to.
    :param received_topic: The topic received.
    :param wildcard: The wildcard to use.
    :return: The level of the received topic.

    Example:
    get_level_from_wildcard('a/+/topic', 'a/fabulous/topic') -> ['fabulous']
    get_level_from_wildcard('a/+/and/+/topic', 'a/fabulous/and/mysterious/topic') -> ['fabulous', 'mysterious']
    """
    levels = []
    for s_level, r_level in zip(subscribed_topic.split('/'), received_topic.split('/'), strict=True):
        if s_level == wildcard:
            levels.append(r_level)
    return levels
