import unittest
from unittest.mock import patch, MagicMock, ANY
from datetime import datetime, timedelta
from uuid import uuid4

from pyniryo.nate.client import Nate, _TokenRenewer
from pyniryo.nate.const import HTTP_PORT, MQTT_PORT, HTTP_TIMEOUT, N_RETRY, RETRY_DELAY_FACTOR
from pyniryo.nate.exceptions import TlsError
from pyniryo.nate._internal.tls import DeviceIdentity
from pyniryo.nate._internal import transport_models, paths_gen
from pyniryo.nate.components import Auth, Users, Robot, Device, Programs, Metrics, Projects
from pyniryo.nate.components.motion_planner import MotionPlanner


class TlsStubs(unittest.TestCase):
    """
    The roots are fetched at build time and the handshake is what binds the identity: both are stubbed here.
    """

    def setUp(self) -> None:
        self.load_roots = patch('pyniryo.nate.client.load_roots', return_value='ROOTS').start()
        self.build_context = patch('pyniryo.nate.client.build_context').start()
        self.addCleanup(patch.stopall)


class TestNateClient(TlsStubs):

    @patch('pyniryo.nate.client._TokenRenewer')
    @patch('pyniryo.nate.client.MqttClient')
    @patch('pyniryo.nate.client.HttpClient')
    def test_init_with_login(self, mock_http_client_class, mock_mqtt_client_class, mock_token_renewer_class):
        """Test initializing client with username and password."""
        # Setup mocks
        mock_http = MagicMock()
        mock_http_client_class.return_value = mock_http

        # Mock login response
        token_response = transport_models.s.Token(id=uuid4(),
                                                  token='generated-token',
                                                  expires_at=datetime.now() + timedelta(days=1),
                                                  created_at=datetime.now())
        mock_http.post.return_value = token_response
        mock_http.get.return_value = transport_models.s.DeviceID(device_id='test-device-123')

        mock_mqtt = MagicMock()
        mock_mqtt_client_class.return_value = mock_mqtt

        mock_token_renewer = MagicMock()
        mock_token_renewer_class.return_value = mock_token_renewer

        # Create client with login
        client = Nate(hostname='192.168.1.100', login=('user@example.com', 'password123'))

        # Verify HTTP client was created correctly (without token)
        mock_http_client_class.assert_called_once_with('192.168.1.100',
                                                       HTTP_PORT,
                                                       ssl_context=ANY,
                                                       http_timeout=HTTP_TIMEOUT,
                                                       n_retry=N_RETRY,
                                                       retry_delay_factor=RETRY_DELAY_FACTOR)

        # Verify login was called
        mock_http.post.assert_called_once()
        call_args = mock_http.post.call_args[0]
        self.assertEqual(call_args[0], paths_gen.Authentication.LOGIN)
        self.assertEqual(call_args[1], transport_models.s.Token)
        login_request = call_args[2]
        self.assertEqual(login_request.login, 'user@example.com')
        self.assertEqual(login_request.password, 'password123')

        # Verify token was set
        mock_http.set_token.assert_called_once_with('generated-token')
        mock_mqtt.set_token.assert_called_once_with('generated-token')

        # Verify device ID was fetched
        mock_http.get.assert_called_once_with(paths_gen.Device.GET_DEVICE_ID, transport_models.s.DeviceID)

        # Verify MQTT client was created
        mock_mqtt_client_class.assert_called_once()

        # Verify _TokenRenewer was initialized and started
        mock_token_renewer_class.assert_called_once()
        mock_token_renewer.start.assert_called_once()

        # Verify all components were initialized
        self.assertIsInstance(client.auth, Auth)
        self.assertIsInstance(client.users, Users)
        self.assertIsInstance(client.robot, Robot)
        self.assertIsInstance(client.device, Device)
        self.assertIsInstance(client.programs, Programs)
        self.assertIsInstance(client.projects, Projects)
        self.assertIsInstance(client.motion_planner, MotionPlanner)

    @patch('pyniryo.nate.client._TokenRenewer')
    @patch('pyniryo.nate.client.MqttClient')
    @patch('pyniryo.nate.client.HttpClient')
    @patch.dict('os.environ', {'NATE_USERNAME': 'env-user', 'NATE_PASSWORD': 'env-pass'})
    def test_init_from_environment_variables_login(self,
                                                   mock_http_client_class,
                                                   mock_mqtt_client_class,
                                                   mock_token_renewer_class):
        """Test initializing client with username/password from environment variables."""
        # Setup mocks
        mock_http = MagicMock()
        mock_http_client_class.return_value = mock_http

        token_response = transport_models.s.Token(id=uuid4(),
                                                  token='generated-token',
                                                  expires_at=datetime.now() + timedelta(days=1),
                                                  created_at=datetime.now())
        mock_http.post.return_value = token_response
        mock_http.get.return_value = transport_models.s.DeviceID(device_id='test-device-123')

        mock_mqtt = MagicMock()
        mock_mqtt_client_class.return_value = mock_mqtt

        mock_token_renewer = MagicMock()
        mock_token_renewer_class.return_value = mock_token_renewer

        # Create client without parameters
        client = Nate()

        # Verify login was called with env vars
        login_request = mock_http.post.call_args[0][2]
        self.assertEqual(login_request.login, 'env-user')
        self.assertEqual(login_request.password, 'env-pass')

    @patch('pyniryo.nate.client._TokenRenewer')
    @patch('pyniryo.nate.client.MqttClient')
    @patch('pyniryo.nate.client.HttpClient')
    def test_init_defaults_to_localhost(self, mock_http_client_class, mock_mqtt_client_class, mock_token_renewer_class):
        """Test that hostname defaults to localhost if not provided."""
        # Setup mocks
        mock_http = MagicMock()
        mock_http_client_class.return_value = mock_http

        token_response = transport_models.s.Token(id=uuid4(),
                                                  token='generated-token',
                                                  expires_at=datetime.now() + timedelta(days=1),
                                                  created_at=datetime.now())
        mock_http.post.return_value = token_response
        mock_http.get.return_value = transport_models.s.DeviceID(device_id='test-device-123')

        mock_mqtt = MagicMock()
        mock_mqtt_client_class.return_value = mock_mqtt

        mock_token_renewer = MagicMock()
        mock_token_renewer_class.return_value = mock_token_renewer

        # Create client with login but no hostname
        client = Nate(login=('user@example.com', 'password123'))

        # Verify localhost was used
        mock_http_client_class.assert_called_once_with('localhost',
                                                       HTTP_PORT,
                                                       ssl_context=ANY,
                                                       http_timeout=HTTP_TIMEOUT,
                                                       n_retry=N_RETRY,
                                                       retry_delay_factor=RETRY_DELAY_FACTOR)

    @patch('pyniryo.nate.client.MqttClient')
    @patch('pyniryo.nate.client.HttpClient')
    def test_init_missing_credentials_raises_error(self, mock_http_client_class, mock_mqtt_client_class):
        """Test that missing credentials raises ValueError."""
        # Setup mocks
        mock_http = MagicMock()
        mock_http_client_class.return_value = mock_http

        # Should raise error when incomplete login
        with self.assertRaises(ValueError) as context:
            client = Nate(hostname='192.168.1.100', login=('user@example.com', None))

        self.assertIn('username and password', str(context.exception))

    @patch('pyniryo.nate.client._TokenRenewer')
    @patch('pyniryo.nate.client.MqttClient')
    @patch('pyniryo.nate.client.HttpClient')
    @patch.dict('os.environ', {'NATE_HTTP_PORT': '9000', 'NATE_MQTT_PORT': '2000'})
    def test_init_custom_ports_from_env(self, mock_http_client_class, mock_mqtt_client_class, mock_token_renewer_class):
        """Test using custom ports from environment variables."""
        # Setup mocks
        mock_http = MagicMock()
        mock_http_client_class.return_value = mock_http

        token_response = transport_models.s.Token(id=uuid4(),
                                                  token='generated-token',
                                                  expires_at=datetime.now() + timedelta(days=1),
                                                  created_at=datetime.now())
        mock_http.post.return_value = token_response
        mock_http.get.return_value = transport_models.s.DeviceID(device_id='test-device-123')

        mock_mqtt = MagicMock()
        mock_mqtt_client_class.return_value = mock_mqtt

        mock_token_renewer = MagicMock()
        mock_token_renewer_class.return_value = mock_token_renewer

        # Create client
        client = Nate(hostname='192.168.1.100', login=('user@example.com', 'password123'))

        # Verify custom ports were used
        mock_http_client_class.assert_called_once_with('192.168.1.100',
                                                       9000,
                                                       ssl_context=ANY,
                                                       http_timeout=HTTP_TIMEOUT,
                                                       n_retry=N_RETRY,
                                                       retry_delay_factor=RETRY_DELAY_FACTOR)

        # Check MQTT client was called with custom port
        mqtt_call_args = mock_mqtt_client_class.call_args
        self.assertEqual(mqtt_call_args[0][1], 2000)

    def _bootstrap(self, mock_http_client_class, mock_mqtt_client_class, mock_token_renewer_class,
                   device_id='test-device-123'):
        mock_http = MagicMock()
        mock_http_client_class.return_value = mock_http
        mock_http.post.return_value = transport_models.s.Token(id=uuid4(),
                                                               token='generated-token',
                                                               expires_at=datetime.now() + timedelta(days=1),
                                                               created_at=datetime.now())
        mock_http.get.return_value = transport_models.s.DeviceID(device_id=device_id)
        mock_mqtt_client_class.return_value = MagicMock()
        mock_token_renewer_class.return_value = MagicMock()

    def test_the_default_ports_are_the_encrypted_ones(self):
        self.assertEqual(HTTP_PORT, 8443)
        self.assertEqual(MQTT_PORT, 8883)

    @patch('pyniryo.nate.client._TokenRenewer')
    @patch('pyniryo.nate.client.MqttClient')
    @patch('pyniryo.nate.client.HttpClient')
    def test_both_transports_share_one_context_built_on_the_bundled_roots(self, mock_http_client_class,
                                                                          mock_mqtt_client_class,
                                                                          mock_token_renewer_class):
        self._bootstrap(mock_http_client_class, mock_mqtt_client_class, mock_token_renewer_class)

        Nate(hostname='192.168.1.100', login=('user@example.com', 'password123'))

        self.load_roots.assert_called_once_with(None)
        roots, identity = self.build_context.call_args.args
        self.assertEqual(roots, 'ROOTS')
        self.assertIsInstance(identity, DeviceIdentity)
        context = self.build_context.return_value
        self.assertIs(mock_http_client_class.call_args.kwargs['ssl_context'], context)
        self.assertIs(mock_mqtt_client_class.call_args.kwargs['ssl_context'], context)

    @patch('pyniryo.nate.client._TokenRenewer')
    @patch('pyniryo.nate.client.MqttClient')
    @patch('pyniryo.nate.client.HttpClient')
    @patch.dict('os.environ', {'NATE_CA_FILE': '/etc/niryo/dev-ca.pem'})
    def test_an_extra_ca_file_comes_from_the_environment(self, mock_http_client_class, mock_mqtt_client_class,
                                                         mock_token_renewer_class):
        self._bootstrap(mock_http_client_class, mock_mqtt_client_class, mock_token_renewer_class)

        Nate(hostname='192.168.1.100', login=('user@example.com', 'password123'))

        self.load_roots.assert_called_once_with('/etc/niryo/dev-ca.pem')

    @patch('pyniryo.nate.client._TokenRenewer')
    @patch('pyniryo.nate.client.MqttClient')
    @patch('pyniryo.nate.client.HttpClient')
    def test_an_expected_device_id_binds_the_client_before_any_handshake(self, mock_http_client_class,
                                                                         mock_mqtt_client_class,
                                                                         mock_token_renewer_class):
        self._bootstrap(mock_http_client_class, mock_mqtt_client_class, mock_token_renewer_class, device_id='robot-1')

        Nate(hostname='192.168.1.100', login=('user@example.com', 'password123'), device_id='robot-1')

        _, identity = self.build_context.call_args.args
        self.assertEqual(identity.expected, 'robot-1')

    @patch('pyniryo.nate.client._TokenRenewer')
    @patch('pyniryo.nate.client.MqttClient')
    @patch('pyniryo.nate.client.HttpClient')
    @patch.dict('os.environ', {'NATE_DEVICE_ID': 'robot-env'})
    def test_the_expected_device_id_comes_from_the_environment_first(self, mock_http_client_class,
                                                                     mock_mqtt_client_class,
                                                                     mock_token_renewer_class):
        self._bootstrap(mock_http_client_class, mock_mqtt_client_class, mock_token_renewer_class, device_id='robot-env')

        Nate(hostname='192.168.1.100', login=('user@example.com', 'password123'), device_id='robot-1')

        _, identity = self.build_context.call_args.args
        self.assertEqual(identity.expected, 'robot-env')

    @patch('pyniryo.nate.client._TokenRenewer')
    @patch('pyniryo.nate.client.MqttClient')
    @patch('pyniryo.nate.client.HttpClient')
    def test_another_robot_than_the_expected_one_is_refused(self, mock_http_client_class, mock_mqtt_client_class,
                                                             mock_token_renewer_class):
        self._bootstrap(mock_http_client_class, mock_mqtt_client_class, mock_token_renewer_class, device_id='robot-2')

        with self.assertRaises(TlsError):
            Nate(hostname='192.168.1.100', login=('user@example.com', 'password123'), device_id='robot-1')

        mock_mqtt_client_class.assert_not_called()

    @patch('pyniryo.nate.client._TokenRenewer')
    @patch('pyniryo.nate.client.MqttClient')
    @patch('pyniryo.nate.client.HttpClient')
    def test_the_api_must_report_the_device_named_by_the_certificate(self, mock_http_client_class,
                                                                     mock_mqtt_client_class,
                                                                     mock_token_renewer_class):
        self._bootstrap(mock_http_client_class, mock_mqtt_client_class, mock_token_renewer_class,
                        device_id='api-device')
        with patch('pyniryo.nate.client.DeviceIdentity', return_value=DeviceIdentity(expected='cert-device')):
            with self.assertRaises(TlsError) as ctx:
                Nate(hostname='192.168.1.100', login=('user@example.com', 'password123'))

        self.assertIn('cert-device', str(ctx.exception))
        self.assertIn('api-device', str(ctx.exception))
        mock_mqtt_client_class.assert_not_called()

    @patch('pyniryo.nate.client._TokenRenewer')
    @patch('pyniryo.nate.client.MqttClient')
    @patch('pyniryo.nate.client.HttpClient')
    def test_the_api_device_id_binds_the_identity_the_mqtt_broker_must_match(self, mock_http_client_class,
                                                                             mock_mqtt_client_class,
                                                                             mock_token_renewer_class):
        self._bootstrap(mock_http_client_class, mock_mqtt_client_class, mock_token_renewer_class)

        Nate(hostname='192.168.1.100', login=('user@example.com', 'password123'))

        _, identity = self.build_context.call_args.args
        self.assertEqual(identity.expected, 'test-device-123')

    @patch('pyniryo.nate.client._TokenRenewer')
    @patch('pyniryo.nate.client.MqttClient')
    @patch('pyniryo.nate.client.HttpClient')
    @patch.dict('os.environ', {'NATE_EXECUTION_TOKEN': 'exec-token-123'})
    def test_init_with_execution_token(self, mock_http_client_class, mock_mqtt_client_class, mock_token_renewer_class):
        """Test initializing with execution token header."""
        # Setup mocks
        mock_http = MagicMock()
        mock_http_client_class.return_value = mock_http

        token_response = transport_models.s.Token(id=uuid4(),
                                                  token='generated-token',
                                                  expires_at=datetime.now() + timedelta(days=1),
                                                  created_at=datetime.now())
        mock_http.post.return_value = token_response
        mock_http.get.return_value = transport_models.s.DeviceID(device_id='test-device-123')

        mock_mqtt = MagicMock()
        mock_mqtt_client_class.return_value = mock_mqtt

        mock_token_renewer = MagicMock()
        mock_token_renewer_class.return_value = mock_token_renewer

        # Create client
        client = Nate(hostname='192.168.1.100', login=('user@example.com', 'password123'))

        # Verify execution token header was set
        mock_http.set_header.assert_called_once_with('Execution-Token', 'exec-token-123')

    @patch('pyniryo.nate.client._TokenRenewer')
    @patch('pyniryo.nate.client.MqttClient')
    @patch('pyniryo.nate.client.HttpClient')
    def test_no_execution_token_header_when_unset(self, mock_http_client_class, mock_mqtt_client_class,
                                                  mock_token_renewer_class):
        mock_http = MagicMock()
        mock_http_client_class.return_value = mock_http
        mock_http.post.return_value = transport_models.s.Token(id=uuid4(),
                                                               token='generated-token',
                                                               expires_at=datetime.now() + timedelta(days=1),
                                                               created_at=datetime.now())
        mock_http.get.return_value = transport_models.s.DeviceID(device_id='test-device-123')
        mock_mqtt_client_class.return_value = MagicMock()
        mock_token_renewer_class.return_value = MagicMock()

        Nate(hostname='192.168.1.100', login=('user@example.com', 'password123'))

        for call in mock_http.set_header.call_args_list:
            self.assertNotEqual(call.args[0], 'Execution-Token')

    @patch('pyniryo.nate.client._TokenRenewer')
    @patch('pyniryo.nate.client.MqttClient')
    @patch('pyniryo.nate.client.HttpClient')
    def test_component_initialization(self, mock_http_client_class, mock_mqtt_client_class, mock_token_renewer_class):
        """Test that all components are properly initialized with clients."""
        # Setup mocks
        mock_http = MagicMock()
        mock_http_client_class.return_value = mock_http

        token_response = transport_models.s.Token(id=uuid4(),
                                                  token='generated-token',
                                                  expires_at=datetime.now() + timedelta(days=1),
                                                  created_at=datetime.now())
        mock_http.post.return_value = token_response
        mock_http.get.return_value = transport_models.s.DeviceID(device_id='test-device-123')

        mock_mqtt = MagicMock()
        mock_mqtt_client_class.return_value = mock_mqtt

        mock_token_renewer = MagicMock()
        mock_token_renewer_class.return_value = mock_token_renewer

        # Create client
        client = Nate(hostname='192.168.1.100', login=('user@example.com', 'password123'))

        # Verify all components exist and are of correct type
        self.assertIsInstance(client.auth, Auth)
        self.assertIsInstance(client.users, Users)
        self.assertIsInstance(client.robot, Robot)
        self.assertIsInstance(client.device, Device)
        self.assertIsInstance(client.programs, Programs)
        self.assertIsInstance(client.projects, Projects)
        self.assertIsInstance(client.motion_planner, MotionPlanner)

        # Verify components have access to clients
        # (This is implicit since they're BaseAPIComponent subclasses)
        self.assertIsNotNone(client.auth)
        self.assertIsNotNone(client.robot)

    @patch('pyniryo.nate.client._TokenRenewer')
    @patch('pyniryo.nate.client.MqttClient')
    @patch('pyniryo.nate.client.HttpClient')
    def test_metrics_component_initialized(self,
                                           mock_http_client_class,
                                           mock_mqtt_client_class,
                                           mock_token_renewer_class):
        """Test that Metrics component is initialized."""
        # Setup mocks
        mock_http = MagicMock()
        mock_http_client_class.return_value = mock_http

        token_response = transport_models.s.Token(id=uuid4(),
                                                  token='generated-token',
                                                  expires_at=datetime.now() + timedelta(days=1),
                                                  created_at=datetime.now())
        mock_http.post.return_value = token_response
        mock_http.get.return_value = transport_models.s.DeviceID(device_id='test-device-123')

        mock_mqtt = MagicMock()
        mock_mqtt_client_class.return_value = mock_mqtt

        mock_token_renewer = MagicMock()
        mock_token_renewer_class.return_value = mock_token_renewer

        # Create client
        client = Nate(hostname='192.168.1.100', login=('user@example.com', 'password123'))

        # Verify metrics component exists
        self.assertIsInstance(client.metrics, Metrics)

    @patch('pyniryo.nate.client._TokenRenewer')
    @patch('pyniryo.nate.client.MqttClient')
    @patch('pyniryo.nate.client.HttpClient')
    def test_close_method_calls_component_close(self,
                                                mock_http_client_class,
                                                mock_mqtt_client_class,
                                                mock_token_renewer_class):
        """Test that close() method calls close() on all components."""
        # Setup mocks
        mock_http = MagicMock()
        mock_http_client_class.return_value = mock_http

        token_response = transport_models.s.Token(id=uuid4(),
                                                  token='generated-token',
                                                  expires_at=datetime.now() + timedelta(days=1),
                                                  created_at=datetime.now())
        mock_http.post.return_value = token_response
        mock_http.get.return_value = transport_models.s.DeviceID(device_id='test-device-123')

        mock_mqtt = MagicMock()
        mock_mqtt_client_class.return_value = mock_mqtt

        mock_token_renewer = MagicMock()
        mock_token_renewer_class.return_value = mock_token_renewer

        # Create client
        client = Nate(hostname='192.168.1.100', login=('user@example.com', 'password123'))

        # Mock close methods on components
        client.auth.close = MagicMock()
        client.users.close = MagicMock()
        client.robot.close = MagicMock()
        client.device.close = MagicMock()
        client.programs.close = MagicMock()
        client.projects.close = MagicMock()
        client.metrics.close = MagicMock()

        # Call close
        client.close()

        # Verify all component close methods were called
        client.auth.close.assert_called_once()
        client.users.close.assert_called_once()
        client.robot.close.assert_called_once()
        client.device.close.assert_called_once()
        client.programs.close.assert_called_once()
        client.projects.close.assert_called_once()
        client.metrics.close.assert_called_once()

    @patch('pyniryo.nate.client._TokenRenewer')
    @patch('pyniryo.nate.client.MqttClient')
    @patch('pyniryo.nate.client.HttpClient')
    def test_close_method_stops_token_renewer(self,
                                              mock_http_client_class,
                                              mock_mqtt_client_class,
                                              mock_token_renewer_class):
        """Test that close() method stops the token renewer."""
        # Setup mocks
        mock_http = MagicMock()
        mock_http_client_class.return_value = mock_http

        token_response = transport_models.s.Token(id=uuid4(),
                                                  token='generated-token',
                                                  expires_at=datetime.now() + timedelta(days=1),
                                                  created_at=datetime.now())
        mock_http.post.return_value = token_response
        mock_http.get.return_value = transport_models.s.DeviceID(device_id='test-device-123')

        mock_mqtt = MagicMock()
        mock_mqtt_client_class.return_value = mock_mqtt

        mock_token_renewer = MagicMock()
        mock_token_renewer_class.return_value = mock_token_renewer

        # Create client
        client = Nate(hostname='192.168.1.100', login=('user@example.com', 'password123'))

        # Call close
        client.close()

        # Verify token renewer stop was called
        mock_token_renewer.stop.assert_called_once()

    @patch('pyniryo.nate.client._TokenRenewer')
    @patch('pyniryo.nate.client.MqttClient')
    @patch('pyniryo.nate.client.HttpClient')
    def test_close_method_disconnects_clients(self,
                                              mock_http_client_class,
                                              mock_mqtt_client_class,
                                              mock_token_renewer_class):
        """Test that close() method disconnects HTTP and MQTT clients."""
        # Setup mocks
        mock_http = MagicMock()
        mock_http_client_class.return_value = mock_http

        token_response = transport_models.s.Token(id=uuid4(),
                                                  token='generated-token',
                                                  expires_at=datetime.now() + timedelta(days=1),
                                                  created_at=datetime.now())
        mock_http.post.return_value = token_response
        mock_http.get.return_value = transport_models.s.DeviceID(device_id='test-device-123')

        mock_mqtt = MagicMock()
        mock_mqtt_client_class.return_value = mock_mqtt

        mock_token_renewer = MagicMock()
        mock_token_renewer_class.return_value = mock_token_renewer

        # Create client
        client = Nate(hostname='192.168.1.100', login=('user@example.com', 'password123'))

        # Call close
        client.close()

        # Verify disconnect methods were called
        mock_mqtt.disconnect.assert_called_once()
        mock_http.disconnect.assert_called_once()

    @patch('pyniryo.nate.client.uuid')
    @patch('pyniryo.nate.client._TokenRenewer')
    @patch('pyniryo.nate.client.MqttClient')
    @patch('pyniryo.nate.client.HttpClient')
    def test_correlation_id_auto_generated(self,
                                           mock_http_client_class,
                                           mock_mqtt_client_class,
                                           mock_token_renewer_class,
                                           mock_uuid):
        """Test that correlation_id is auto-generated when not provided via env var."""
        # Setup mocks
        mock_http = MagicMock()
        mock_http_client_class.return_value = mock_http

        token_response = transport_models.s.Token(id=uuid4(),
                                                  token='generated-token',
                                                  expires_at=datetime.now() + timedelta(days=1),
                                                  created_at=datetime.now())
        mock_http.post.return_value = token_response
        mock_http.get.return_value = transport_models.s.DeviceID(device_id='test-device-123')

        mock_mqtt = MagicMock()
        mock_mqtt_client_class.return_value = mock_mqtt

        mock_token_renewer = MagicMock()
        mock_token_renewer_class.return_value = mock_token_renewer

        # Mock UUID generation
        mock_uuid4_instance = MagicMock()
        mock_uuid4_instance.hex = 'auto-generated-uuid-hex'
        mock_uuid.uuid4.return_value = mock_uuid4_instance

        # Create client
        client = Nate(hostname='192.168.1.100', login=('user@example.com', 'password123'))

        # Verify correlation_id was set
        self.assertEqual(client._correlation_id, 'auto-generated-uuid-hex')

    @patch('pyniryo.nate.client._TokenRenewer')
    @patch('pyniryo.nate.client.MqttClient')
    @patch('pyniryo.nate.client.HttpClient')
    @patch.dict('os.environ', {'NATE_CORRELATION_ID': 'custom-correlation-id-123'})
    def test_correlation_id_from_environment(self,
                                             mock_http_client_class,
                                             mock_mqtt_client_class,
                                             mock_token_renewer_class):
        """Test that correlation_id is read from environment variable when provided."""
        # Setup mocks
        mock_http = MagicMock()
        mock_http_client_class.return_value = mock_http

        token_response = transport_models.s.Token(id=uuid4(),
                                                  token='generated-token',
                                                  expires_at=datetime.now() + timedelta(days=1),
                                                  created_at=datetime.now())
        mock_http.post.return_value = token_response
        mock_http.get.return_value = transport_models.s.DeviceID(device_id='test-device-123')

        mock_mqtt = MagicMock()
        mock_mqtt_client_class.return_value = mock_mqtt

        mock_token_renewer = MagicMock()
        mock_token_renewer_class.return_value = mock_token_renewer

        # Create client
        client = Nate(hostname='192.168.1.100', login=('user@example.com', 'password123'))

        # Verify correlation_id was read from env
        self.assertEqual(client._correlation_id, 'custom-correlation-id-123')


class TestClientEnvironment(TlsStubs):

    def _bootstrap(self, mock_http_client_class, mock_mqtt_client_class, mock_token_renewer_class):
        mock_http = MagicMock()
        mock_http_client_class.return_value = mock_http
        mock_http.post.return_value = transport_models.s.Token(id=uuid4(),
                                                               token='generated-token',
                                                               expires_at=datetime.now() + timedelta(days=1),
                                                               created_at=datetime.now())
        mock_http.get.return_value = transport_models.s.DeviceID(device_id='test-device-123')
        mock_mqtt_client_class.return_value = MagicMock()
        mock_token_renewer_class.return_value = MagicMock()

    @patch('pyniryo.nate.client._TokenRenewer')
    @patch('pyniryo.nate.client.MqttClient')
    @patch('pyniryo.nate.client.HttpClient')
    @patch.dict('os.environ', {'NATE_RETRY': '0'})
    def test_falsy_environment_values_are_honoured(self, mock_http_client_class, mock_mqtt_client_class,
                                                   mock_token_renewer_class):
        self._bootstrap(mock_http_client_class, mock_mqtt_client_class, mock_token_renewer_class)

        Nate(hostname='192.168.1.100', login=('user@example.com', 'password123'))

        kwargs = mock_http_client_class.call_args.kwargs
        self.assertEqual(kwargs['n_retry'], 0)
        self.assertEqual(mock_mqtt_client_class.call_args.kwargs['n_retry'], 0)

    @patch('pyniryo.nate.client._TokenRenewer')
    @patch('pyniryo.nate.client.MqttClient')
    @patch('pyniryo.nate.client.HttpClient')
    @patch.dict('os.environ', {'NATE_USERNAME': '', 'NATE_PASSWORD': ''})
    def test_empty_credentials_are_refused(self, mock_http_client_class, mock_mqtt_client_class,
                                           mock_token_renewer_class):
        self._bootstrap(mock_http_client_class, mock_mqtt_client_class, mock_token_renewer_class)

        with self.assertRaises(ValueError):
            Nate(hostname='192.168.1.100')


class Test_TokenRenewer(unittest.TestCase):
    """Test the _TokenRenewer class."""

    @patch('pyniryo.nate.client.get_token_expiration')
    @patch('threading.Timer')
    def test_a_setter_that_fails_is_retried_soon_with_the_same_token(self, mock_timer_class, mock_get_token_expiration):
        """A broker that refuses the fresh token must be retried soon, not at the next renewal hours away."""
        mock_get_token_expiration.return_value = datetime.now() + timedelta(hours=1)
        token_provider = MagicMock(return_value='fresh-token')
        http_setter = MagicMock()
        mqtt_setter = MagicMock(side_effect=ConnectionError('CONNACK refused'))
        renewer = _TokenRenewer(token_provider, [http_setter, mqtt_setter], initial_token='test-token')
        mock_timer_class.reset_mock()

        renewer._run()

        http_setter.assert_called_once_with('fresh-token')
        delay, callback = mock_timer_class.call_args[0]
        self.assertLessEqual(delay, 60)
        mock_timer_class.reset_mock()
        token_provider.reset_mock()
        mqtt_setter.side_effect = None

        callback()

        token_provider.assert_not_called()
        mqtt_setter.assert_called_with('fresh-token')
        self.assertGreater(mock_timer_class.call_args[0][0], 60)

    @patch('pyniryo.nate.client.get_token_expiration')
    @patch('threading.Timer')
    def test_a_failed_renewal_is_retried_later(self, mock_timer_class, mock_get_token_expiration):
        """A renewal that fails must not end the renewals for good."""
        mock_get_token_expiration.return_value = datetime.now() + timedelta(hours=1)
        token_provider = MagicMock(side_effect=ConnectionError('robot unreachable'))
        setter = MagicMock()
        renewer = _TokenRenewer(token_provider, [setter], initial_token='test-token')
        mock_timer_class.reset_mock()

        renewer._run()

        setter.assert_not_called()
        mock_timer_class.assert_called_once()
        delay = mock_timer_class.call_args[0][0]
        self.assertGreater(delay, 0)
        self.assertLessEqual(delay, 60)
        mock_timer_class.return_value.start.assert_called_once()

    @patch('pyniryo.nate.client.get_token_expiration')
    @patch('threading.Timer')
    def test_stop_cancels_timer(self, mock_timer_class, mock_get_token_expiration):
        """Test that stop() method cancels the active timer."""
        # Setup mock timer
        mock_timer = MagicMock()
        mock_timer_class.return_value = mock_timer

        # Mock get_token_expiration to return a future datetime
        mock_get_token_expiration.return_value = datetime.now() + timedelta(hours=1)

        # Create token provider and callbacks
        token_provider = MagicMock(return_value='test-token')
        callbacks = [MagicMock()]
        initial_token = 'test-token'

        # Create renewer and start it
        renewer = _TokenRenewer(token_provider, callbacks, initial_token=initial_token)
        renewer.start()

        # Verify timer was created
        mock_timer_class.assert_called()

        # Call stop
        renewer.stop()

        # Verify timer was cancelled
        mock_timer.cancel.assert_called()

    @patch('pyniryo.nate.client.get_token_expiration')
    @patch('threading.Timer')
    def test_stop_when_no_timer_active(self, mock_timer_class, mock_get_token_expiration):
        """Test that stop() handles case when no timer is active."""
        # Mock get_token_expiration to return a future datetime
        mock_get_token_expiration.return_value = datetime.now() + timedelta(hours=1)

        # Create token provider and callbacks
        token_provider = MagicMock(return_value='test-token')
        callbacks = [MagicMock()]
        initial_token = 'test-token'

        # Create renewer without starting
        renewer = _TokenRenewer(token_provider, callbacks, initial_token=initial_token)

        # Call stop (should not raise)
        renewer.stop()

        # No timer to cancel, so cancel should not be called
        mock_timer_class.return_value.cancel.assert_called_once()


if __name__ == "__main__":
    unittest.main()
