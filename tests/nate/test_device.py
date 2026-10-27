import unittest
from datetime import datetime, timedelta, timezone

from pyniryo.nate._internal import transport_models, paths_gen
from pyniryo.nate.components.device import Device
from unittest.mock import MagicMock

from pyniryo.nate._internal import topics_gen
from pyniryo.nate.exceptions import ServerError, DataValidationError
from pyniryo.nate.models.network import InterfaceType
from pyniryo.nate.models import device as device_models

from .base import BaseTestComponent


class TestDevice(BaseTestComponent):

    def setUp(self):
        super().setUp()
        self.device = Device(http_client=self.http_client, mqtt_client=self.mqtt_client, correlation_id=self.correlation_id)

    def tearDown(self):
        del self.device

    def test_id(self):
        """Test getting the device ID."""
        expected_id = "test-device-123"
        self.http_client.get.return_value = transport_models.s.DeviceID(device_id=expected_id)

        device_id = self.device.id()

        self.http_client.get.assert_called_once_with(paths_gen.Device.GET_DEVICE_ID, transport_models.s.DeviceID)
        self.assertEqual(device_id, expected_id)

    def test_is_healthy_success(self):
        """Test health check when robot is healthy."""
        self.http_client.get.return_value = transport_models.HealthCheckResponse(status='healthy')

        is_healthy = self.device.is_healthy()

        self.http_client.get.assert_called_once_with(paths_gen.Device.HEALTH_CHECK,
                                                     transport_models.HealthCheckResponse)
        self.assertTrue(is_healthy)

    def test_is_healthy_failure(self):
        """Test health check when robot is not healthy."""
        self.http_client.get.side_effect = ServerError(500, "Health check failed")

        is_healthy = self.device.is_healthy()

        self.http_client.get.assert_called_once_with(paths_gen.Device.HEALTH_CHECK,
                                                     transport_models.HealthCheckResponse)
        self.assertFalse(is_healthy)

    def test_is_ready_true(self):
        """Test readiness check when robot is ready."""
        self.http_client.get.return_value = transport_models.ReadinessCheckResponse(ready=True)

        is_ready = self.device.is_ready()

        self.http_client.get.assert_called_once_with(paths_gen.Device.READINESS_CHECK,
                                                     transport_models.ReadinessCheckResponse)
        self.assertTrue(is_ready)

    def test_is_ready_false(self):
        """Test readiness check when robot is not ready."""
        self.http_client.get.return_value = transport_models.ReadinessCheckResponse(ready=False)

        is_ready = self.device.is_ready()

        self.http_client.get.assert_called_once_with(paths_gen.Device.READINESS_CHECK,
                                                     transport_models.ReadinessCheckResponse)
        self.assertFalse(is_ready)

    def test_is_healthy_with_an_empty_body(self):
        self.http_client.get.return_value = transport_models.HealthCheckResponse()

        self.assertTrue(self.device.is_healthy())

    def test_get_identity(self):
        released = datetime(2026, 8, 28, 9, 12, tzinfo=timezone.utc)
        self.http_client.get.return_value = transport_models.s.DeviceID(
            device_id='dev-1', device_name='App - QA', model='nr2m', serial_number='1042',
            software=transport_models.s.DeviceSoftware(release_id='3195000', version='0.60.1', released_at=released))

        identity = self.device.get_identity()

        self.http_client.get.assert_called_once_with(paths_gen.Device.GET_DEVICE_ID, transport_models.s.DeviceID)
        self.assertEqual(identity, device_models.DeviceIdentity(
            device_id='dev-1', device_name='App - QA', model='nr2m', serial_number='1042',
            software=device_models.DeviceSoftware(release_id='3195000', version='0.60.1', released_at=released,
                                                  updated_at=None)))

    def test_get_identity_with_the_id_alone(self):
        self.http_client.get.return_value = transport_models.s.DeviceID(device_id='dev-1')

        identity = self.device.get_identity()

        self.assertEqual(identity.device_id, 'dev-1')
        self.assertIsNone(identity.software)

    def test_set_datetime_sends_utc(self):
        self.http_client.post.return_value = transport_models.EmptyPayload()
        paris = timezone(timedelta(hours=2))

        self.device.set_datetime(datetime(2026, 1, 1, 12, 0, tzinfo=paris))

        request = self.http_client.post.call_args[0][2]
        self.assertEqual(request.datetime, datetime(2026, 1, 1, 10, 0, tzinfo=timezone.utc))

    def test_set_datetime_rejects_a_naive_datetime(self):
        with self.assertRaises(DataValidationError):
            self.device.set_datetime(datetime(2026, 1, 1, 12, 0))
        self.http_client.post.assert_not_called()

    def test_set_ntp_config_sends_the_writable_fields_filled_from_the_current_config(self):
        current = transport_models.s.NTPConfig(enabled=False, synchronized=False, servers=['old.ntp'],
                                               pools=['pool.ntp.org'], last_sync=datetime(2026, 1, 1, tzinfo=timezone.utc))
        self.http_client.get.return_value = current
        self.http_client.patch.return_value = transport_models.s.NTPConfig(
            enabled=True, synchronized=True, servers=['ntp.local'], pools=['pool.ntp.org'],
            last_sync=datetime(2026, 1, 1, tzinfo=timezone.utc))

        config = self.device.set_ntp_config(enabled=True, servers=['ntp.local'])

        self.http_client.get.assert_called_once_with(paths_gen.Device.GET_DEVICE_NTP_CONFIG, transport_models.s.NTPConfig)
        path, model, request = self.http_client.patch.call_args[0]
        self.assertEqual(path, paths_gen.Device.SET_DEVICE_NTP_CONFIG)
        self.assertEqual(model, transport_models.s.NTPConfig)
        self.assertEqual(request.model_dump(exclude_none=True),
                         {'enabled': True, 'servers': ['ntp.local'], 'pools': ['pool.ntp.org']})
        self.assertTrue(config.synchronized)

    def test_set_ntp_config_with_every_field_does_not_read_the_current_config(self):
        self.http_client.patch.return_value = transport_models.s.NTPConfig(
            enabled=True, synchronized=False, servers=[], pools=['pool.ntp.org'],
            last_sync=datetime(2026, 1, 1, tzinfo=timezone.utc))

        self.device.set_ntp_config(enabled=True, servers=[], pools=['pool.ntp.org'])

        self.http_client.get.assert_not_called()
        request = self.http_client.patch.call_args[0][2]
        self.assertEqual(request.model_dump(exclude_none=True), {'enabled': True, 'servers': [], 'pools': ['pool.ntp.org']})

    def test_get_version(self):
        self.http_client.get.return_value = transport_models.s.DeviceVersion(apid_version='1.4.0', apid_commit='4131ba7')

        version = self.device.get_version()

        self.http_client.get.assert_called_once_with(paths_gen.Device.GET_DEVICE_VERSION,
                                                     transport_models.s.DeviceVersion)
        self.assertEqual(version, device_models.DeviceVersion(api_version='1.4.0', api_commit='4131ba7'))

    def test_get_status(self):
        program_id = '0b7d1e9c-3f21-4c8a-9a3e-1d7c6b5a4e33'
        self.http_client.get.return_value = transport_models.s.DeviceStatus(
            status=transport_models.s.Status.DEVICE_RUNNING,
            client_id='studio-9f1c2b7e',
            manual_mode=False,
            armed=True,
            device_name='App - QA',
            model='nr2m',
            hostname='6ff4823.local',
            ip='192.168.1.19',
            interfaces=[transport_models.s.DeviceInterface(id='eth0', type=transport_models.s.InterfaceType.ETHERNET,
                                                           ip='192.168.1.19')],
            execution=transport_models.s.DeviceExecution(program_id=program_id, program_name='pick-and-place',
                                                         project_name='Palletizing'))

        status = self.device.get_status()

        self.http_client.get.assert_called_once_with(paths_gen.Device.GET_DEVICE_STATUS, transport_models.s.DeviceStatus)
        self.assertEqual(status, device_models.DeviceStatus(
            status=device_models.DeviceStatusValue.RUNNING,
            client_id='studio-9f1c2b7e',
            manual_mode=False,
            armed=True,
            device_name='App - QA',
            model='nr2m',
            hostname='6ff4823.local',
            ip='192.168.1.19',
            interfaces=[device_models.DeviceInterface(id='eth0', type=InterfaceType.ETHERNET, ip='192.168.1.19')],
            execution=device_models.DeviceExecution(program_id=program_id, program_name='pick-and-place',
                                                    project_name='Palletizing')))

    def test_get_status_with_the_status_alone(self):
        self.http_client.get.return_value = transport_models.s.DeviceStatus(status=transport_models.s.Status.DEVICE_UNKNOWN)

        status = self.device.get_status()

        self.assertEqual(status.status, device_models.DeviceStatusValue.UNKNOWN)
        self.assertEqual(status.interfaces, [])
        self.assertIsNone(status.execution)
        self.assertIsNone(status.armed)

    def test_get_update(self):
        released = datetime(2026, 9, 1, 8, 30, tzinfo=timezone.utc)
        self.http_client.get.return_value = transport_models.s.DeviceUpdate(
            state=transport_models.s.State.UPDATE_STATE_DOWNLOADING,
            stale=False,
            progress=42,
            target=transport_models.s.DeviceUpdateTarget(release_id='3195392', version='0.61.0', released_at=released,
                                                         size_bytes=1932735283, release_notes='Faster planning'))

        update = self.device.get_update()

        self.http_client.get.assert_called_once_with(paths_gen.Device.GET_DEVICE_UPDATE, transport_models.s.DeviceUpdate)
        self.assertEqual(update, device_models.DeviceUpdate(
            state=device_models.DeviceUpdateState.DOWNLOADING,
            stale=False,
            progress=42,
            error=None,
            target=device_models.DeviceUpdateTarget(release_id='3195392', version='0.61.0', released_at=released,
                                                    size_bytes=1932735283, release_notes='Faster planning')))

    def test_get_update_idle(self):
        self.http_client.get.return_value = transport_models.s.DeviceUpdate(state=transport_models.s.State.UPDATE_STATE_IDLE,
                                                                            stale=True)

        update = self.device.get_update()

        self.assertEqual(update.state, device_models.DeviceUpdateState.IDLE)
        self.assertTrue(update.stale)
        self.assertIsNone(update.target)

    def test_start_update(self):
        self.http_client.post.return_value = transport_models.EmptyPayload()

        self.device.start_update('3195392')

        path, model, body = self.http_client.post.call_args[0]
        self.assertEqual(path, paths_gen.Device.START_DEVICE_UPDATE)
        self.assertEqual(model, transport_models.EmptyPayload)
        self.assertEqual(body, transport_models.s.DeviceUpdateRequest(release_id='3195392'))

    def test_check_update(self):
        self.http_client.post.return_value = transport_models.EmptyPayload()

        self.device.check_update()

        path, model, body = self.http_client.post.call_args[0]
        self.assertEqual(path, paths_gen.Device.CHECK_DEVICE_UPDATE)
        self.assertIsInstance(body, transport_models.EmptyPayload)

    def test_on_update_state(self):
        callback = MagicMock()

        unsubscribe = self.device.on_update_state(callback)

        topic, internal_callback, model = self.mqtt_client.subscribe.call_args[0]
        self.assertEqual(topic, topics_gen.Update.STATE)
        self.assertEqual(model, transport_models.a.DeviceUpdateState)
        self.assertIs(unsubscribe, self.mqtt_client.subscribe.return_value)
        internal_callback(topic, transport_models.a.DeviceUpdateState(
            state=transport_models.a.DeviceUpdateStateState('error'),
            stale=False,
            error='the robot did not come back',
            target={'release_id': '3195392', 'version': '0.61.0'}))
        callback.assert_called_once_with(device_models.DeviceUpdate(
            state=device_models.DeviceUpdateState.ERROR,
            stale=False,
            progress=None,
            error='the robot did not come back',
            target=device_models.DeviceUpdateTarget(release_id='3195392', version='0.61.0')))

    def test_spec_documents(self):
        self.http_client.get_text.return_value = 'doc'
        for method, path, accept in (
            (self.device.get_openapi_spec, paths_gen.Documentation.GET_OPEN_API_SPEC, 'application/yaml'),
            (self.device.get_asyncapi_spec, paths_gen.Documentation.GET_ASYNC_API_SPEC, 'application/yaml'),
            (self.device.get_openapi_ui, paths_gen.Documentation.GET_OPEN_APIUI, 'text/html'),
            (self.device.get_asyncapi_ui, paths_gen.Documentation.GET_ASYNC_APIUI, 'text/html'),
        ):
            with self.subTest(method=method.__name__):
                self.http_client.get_text.reset_mock()

                self.assertEqual(method(), 'doc')

                self.http_client.get_text.assert_called_once_with(path, accept=accept)

    def test_reboot(self):
        """Test rebooting the robot."""
        self.http_client.post.return_value = transport_models.EmptyPayload()

        self.device.reboot()

        self.http_client.post.assert_called_once()
        call_args = self.http_client.post.call_args[0]
        self.assertEqual(call_args[0], paths_gen.Device.REBOOT)
        self.assertEqual(call_args[1], transport_models.EmptyPayload)
        self.assertIsInstance(call_args[2], transport_models.EmptyPayload)

    def test_shutdown(self):
        """Test shutting down the robot."""
        self.http_client.post.return_value = transport_models.EmptyPayload()

        self.device.shutdown()

        self.http_client.post.assert_called_once()
        call_args = self.http_client.post.call_args[0]
        self.assertEqual(call_args[0], paths_gen.Device.SHUTDOWN)
        self.assertEqual(call_args[1], transport_models.EmptyPayload)
        self.assertIsInstance(call_args[2], transport_models.EmptyPayload)


if __name__ == "__main__":
    unittest.main()
