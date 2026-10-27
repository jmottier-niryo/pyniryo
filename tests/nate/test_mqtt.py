import ssl
import unittest
import unittest.mock

from unittest.mock import create_autospec, patch

from pydantic import BaseModel

from pyniryo.nate._internal.mqtt import MqttClient, get_level_from_wildcard
from pyniryo.nate._internal.tls import DeviceIdentity, build_context
from pyniryo.nate.exceptions import TlsError

from .fixtures import tls as fixtures

from paho.mqtt.client import Client as PahoClient
from paho.mqtt.packettypes import PacketTypes

CLIENT_PATH_PATH = 'pyniryo.nate._internal.mqtt.Client'


def get_paho_mock(connack_reason: int = 0):
    paho = create_autospec(PahoClient, spec_set=True, instance=True)

    def connect(*_args, **_kwargs):
        if callable(paho.on_connect):
            paho.on_connect(paho, None, None, connack_reason, None)

    paho.connect.side_effect = connect
    return paho


def new_context(identity: DeviceIdentity | None = None) -> ssl.SSLContext:
    return build_context(fixtures.CA.read_text(), identity or DeviceIdentity())


def new_client(device_id: str = "device_id", correlation_id: str = "correlation_id", port: int = 8883,
               **kwargs) -> MqttClient:
    kwargs.setdefault('ssl_context', new_context())
    return MqttClient("127.0.0.1", port, device_id, correlation_id, **kwargs)


class PayloadModel(BaseModel):
    ...


class Message:
    topic = "test/topic"
    payload = b'{}'


class TestMqttClient(unittest.TestCase):

    @patch(CLIENT_PATH_PATH)
    def test_connect(self, mock_client):
        mock_client.return_value = get_paho_mock()
        client = new_client("device_id", "correlation_id")
        client.set_token('token')
        mock_client.return_value.connect.assert_called_once()
        args, kwargs = mock_client.return_value.connect.call_args
        self.assertEqual(args, ("127.0.0.1", 8883))
        self.assertFalse(kwargs["clean_start"])
        self.assertEqual(kwargs["properties"].packetType, PacketTypes.CONNECT)
        self.assertEqual(kwargs["properties"].SessionExpiryInterval, 30)
        mock_client.return_value.loop_start.assert_called_once()

    @patch(CLIENT_PATH_PATH)
    def test_client_id_keeps_the_correlation_id_as_prefix(self, mock_client):
        mock_client.return_value = get_paho_mock()
        client = new_client("device_id", "9f1c2b7e-4b5a-6978-8796-a5b4c3d2e1f0")
        client.set_token('token')
        client_id = mock_client.call_args.kwargs['client_id']
        self.assertTrue(client_id.startswith('pyniryo-9f1c2b7e.'), client_id)

    @patch(CLIENT_PATH_PATH)
    def test_client_id_keeps_only_safe_characters_of_the_correlation_id(self, mock_client):
        mock_client.return_value = get_paho_mock()
        client = new_client("device_id", "a b/c.d=e")
        client.set_token('token')
        client_id = mock_client.call_args.kwargs['client_id']
        self.assertTrue(client_id.startswith('pyniryo-abcde.'), client_id)

    @patch(CLIENT_PATH_PATH)
    def test_two_clients_with_the_same_correlation_id_get_distinct_client_ids(self, mock_client):
        mock_client.side_effect = [get_paho_mock(), get_paho_mock(), get_paho_mock(), get_paho_mock()]
        first = new_client("device_id", "correlation_id")
        second = new_client("device_id", "correlation_id")
        first.set_token('token')
        second.set_token('token')
        client_ids = [call.kwargs['client_id'] for call in mock_client.call_args_list if 'client_id' in call.kwargs]
        self.assertEqual(len(client_ids), 2)
        self.assertNotEqual(client_ids[0], client_ids[1])

    @patch(CLIENT_PATH_PATH)
    def test_renewing_the_token_keeps_the_client_id(self, mock_client):
        mock_client.side_effect = [get_paho_mock(), get_paho_mock(), get_paho_mock()]
        client = new_client("device_id", "correlation_id")
        client.set_token('token-1')
        client.set_token('token-2')
        client_ids = [call.kwargs['client_id'] for call in mock_client.call_args_list if 'client_id' in call.kwargs]
        self.assertEqual(len(client_ids), 2)
        self.assertEqual(client_ids[0], client_ids[1])

    @patch(CLIENT_PATH_PATH)
    def test_disconnect(self, mock_client):
        mock_client.return_value = get_paho_mock()
        client = new_client("device_id", "correlation_id")
        mock_client.return_value.is_connected = lambda: False
        client.set_token('token')
        client.disconnect()
        mock_client.return_value.loop_stop.assert_called_once()
        mock_client.return_value.disconnect.assert_called_once()

    def __callback(self, _topic, _payload):
        pass

    @patch(CLIENT_PATH_PATH)
    def test_subscribe(self, mock_client):
        mock_client.return_value = get_paho_mock()
        client = new_client("device_id", "correlation_id")
        client.subscribe("test/topic", self.__callback, PayloadModel)
        mock_client.return_value.subscribe.assert_called_once_with("test/topic", 2)

    @patch(CLIENT_PATH_PATH)
    def test_multi_subscribe(self, mock_client):
        mock_client.return_value = get_paho_mock()
        client = new_client("device_id", "correlation_id")
        client.subscribe("test/topic", self.__callback, PayloadModel)
        mock_client.return_value.subscribe.assert_called_once_with("test/topic", 2)

        mock_client.return_value.subscribe.reset_mock()
        client.subscribe("test/topic", self.__callback, PayloadModel)
        mock_client.return_value.subscribe.assert_not_called()

    @patch(CLIENT_PATH_PATH)
    def test_unsubscribing_one_callback_keeps_the_others(self, mock_client):
        """Test that a callback added after a removal gets its own index rather than an freed one."""
        mock_client.return_value = get_paho_mock()
        client = new_client("device_id", "correlation_id")

        received = []
        unsubscribe_first = client.subscribe("test/topic", lambda *_: received.append('first'), PayloadModel)
        client.subscribe("test/topic", lambda *_: received.append('second'), PayloadModel)
        unsubscribe_first()
        client.subscribe("test/topic", lambda *_: received.append('third'), PayloadModel)

        handle_message = mock_client.return_value.message_callback_add.call_args[0][1]
        handle_message(None, None, Message())

        self.assertEqual(sorted(received), ['second', 'third'])


class TestGetLevelFromWildcard(unittest.TestCase):

    def test_get_level_from_wildcard(self):
        result = get_level_from_wildcard('a/+/topic', 'a/fabulous/topic')
        self.assertEqual(result, ['fabulous'])

    def test_get_level_from_wildcard_with_multiple_levels(self):
        result = get_level_from_wildcard('a/+/topic/+', 'a/fabulous/topic/indeed')
        self.assertEqual(result, ['fabulous', 'indeed'])

    def test_get_level_get_level_without_wildcard(self):
        result = get_level_from_wildcard('a/b/topic', 'a/b/topic')
        self.assertEqual(result, [])


class OptionalPayload(BaseModel):
    velocities: list[float] | None = None
    displacements: list[float] | None = None


class TestMqttWireFormat(unittest.TestCase):

    @patch(CLIENT_PATH_PATH)
    def test_unset_optional_fields_are_omitted_from_the_payload(self, mock_client):
        mock_client.return_value = get_paho_mock()
        client = new_client("device_id", "correlation_id")
        client.set_token('token')

        client.publish("device/{device_id}/cmd", OptionalPayload(velocities=[1.0]))

        _topic, payload = mock_client.return_value.publish.call_args.args
        self.assertEqual(payload, '{"velocities": [1.0]}')


class BadMessage:
    topic = "test/topic"
    payload = b'{not json'


class TestMqttRobustness(unittest.TestCase):

    @patch(CLIENT_PATH_PATH)
    def test_a_refused_connack_raises(self, mock_client):
        mock_client.return_value = get_paho_mock(connack_reason=0x86)
        client = new_client("device_id", "correlation_id")

        with self.assertRaises(ConnectionError):
            client.set_token('bad token')

    @patch(CLIENT_PATH_PATH)
    def test_a_malformed_payload_is_logged_not_raised(self, mock_client):
        mock_client.return_value = get_paho_mock()
        client = new_client("device_id", "correlation_id")
        callback = unittest.mock.MagicMock()
        client.subscribe("test/topic", callback, PayloadModel)
        internal_callback = mock_client.return_value.message_callback_add.call_args[0][1]

        with self.assertLogs('pyniryo.nate._internal.mqtt', level='ERROR'):
            internal_callback(mock_client.return_value, None, BadMessage())

        callback.assert_not_called()

    @patch(CLIENT_PATH_PATH)
    def test_renewing_the_token_stops_the_previous_client_even_when_disconnected(self, mock_client):
        first, second = get_paho_mock(), get_paho_mock()
        first.is_connected.return_value = False
        mock_client.side_effect = [get_paho_mock(), first, second]
        client = new_client("device_id", "correlation_id")

        client.set_token('token-1')
        client.set_token('token-2')

        first.loop_stop.assert_called_once()
        first.disconnect.assert_called_once()
        second.connect.assert_called_once()


class TestMqttClientTls(unittest.TestCase):

    @patch(CLIENT_PATH_PATH)
    def test_hands_the_context_to_paho_and_leaves_the_host_name_unchecked(self, mock_client):
        mock_client.return_value = get_paho_mock()
        context = new_context()
        client = new_client(ssl_context=context)

        client.set_token('token')

        mock_client.return_value.tls_set_context.assert_called_once_with(context)
        mock_client.return_value.tls_insecure_set.assert_called_once_with(True)

    @patch('time.sleep')
    @patch(CLIENT_PATH_PATH)
    def test_a_refused_certificate_is_not_retried(self, mock_client, sleep):
        mock_client.return_value = get_paho_mock()
        mock_client.return_value.connect.side_effect = ssl.SSLCertVerificationError('certificate verify failed')
        client = new_client(n_retry=3)

        with self.assertRaises(TlsError) as ctx:
            client.set_token('token')

        self.assertIn('certificate verify failed', str(ctx.exception))
        self.assertEqual(mock_client.return_value.connect.call_count, 1)
        sleep.assert_not_called()

    def test_binds_the_device_of_the_broker_certificate(self):
        identity = DeviceIdentity()
        with fixtures.raw_tls_server(fixtures.ROBOT) as (_, port):
            client = new_client(port=port, ssl_context=new_context(identity), connack_timeout=0.1)

            client.set_token('token')
            client.disconnect()

        self.assertEqual(identity.expected, fixtures.DEVICE_ID)

    def test_refuses_a_broker_without_device_identity_before_sending_the_token(self):
        with fixtures.raw_tls_server(fixtures.NO_URN) as (_, port):
            client = new_client(port=port, connack_timeout=0.1)

            with self.assertRaises(TlsError):
                client.set_token('token')

    def test_refuses_a_broker_naming_another_device(self):
        identity = DeviceIdentity(expected=fixtures.DEVICE_ID)
        with fixtures.raw_tls_server(fixtures.OTHER_DEVICE) as (_, port):
            client = new_client(port=port, ssl_context=new_context(identity), connack_timeout=0.1)

            with self.assertRaises(TlsError) as ctx:
                client.set_token('token')

        self.assertIn(fixtures.OTHER_DEVICE_ID, str(ctx.exception))

    def test_refuses_a_broker_signed_by_another_root(self):
        with fixtures.raw_tls_server(fixtures.OTHER_CA_ROBOT) as (_, port):
            client = new_client(port=port, connack_timeout=0.1)

            with self.assertRaises(TlsError):
                client.set_token('token')
