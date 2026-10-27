import unittest
from unittest.mock import MagicMock

from pyniryo.nate._internal import transport_models, paths_gen, topics_gen
from pyniryo.nate.components.firmware import Firmware
from pyniryo.nate.models.firmware import FirmwareState, FirmwareStatus, SafetyState, PlcSafetyState

from .base import BaseTestComponent

PLC_SAFETY_FIELDS = dict(
    robot_initialized=True,
    stop_request=False,
    pause_request=False,
    automatic_move_authorization=True,
    manual_move_authorization=True,
    wait_for_initialization=False,
    wait_for_automatic_restart=False,
    manual_mode=False,
    io_emergency_stop_active=False,
    io_safety_stop_active=False,
    io_initialization_button_active=False,
    io_automatic_restart_button_active=False,
    io3_positions_button_active=False,
    io_emergency_stop_output_active=False,
    io_initialization_output_active=False,
)


class TestFirmware(BaseTestComponent):

    def setUp(self):
        super().setUp()
        self.firmware = Firmware(http_client=self.http_client,
                                 mqtt_client=self.mqtt_client,
                                 correlation_id=self.correlation_id)

    def tearDown(self):
        del self.firmware

    def test_get_state(self):
        self.http_client.get.return_value = transport_models.s.FirmwareState(
            status=transport_models.s.Status1.FW_STATE_ACTIVE,
            container_status='running',
            safety=transport_models.s.SafetyState(armed=True,
                                                   manual_mode=False,
                                                   movement_authorized=True,
                                                   protective_stop_active=False,
                                                   emergency_stop_active=False,
                                                   max_velocity_factor=1.0),
        )

        state = self.firmware.get_state()

        self.http_client.get.assert_called_once_with(paths_gen.Firmware.GET_FIRMWARE_STATE,
                                                     transport_models.s.FirmwareState)
        self.assertIsInstance(state, FirmwareState)
        self.assertEqual(state.status, FirmwareStatus.ACTIVE)
        self.assertEqual(state.container_status, 'running')
        self.assertIsInstance(state.safety, SafetyState)
        self.assertTrue(state.safety.armed)
        self.assertEqual(state.safety.max_velocity_factor, 1.0)

    def test_get_state_without_safety(self):
        self.http_client.get.return_value = transport_models.s.FirmwareState(
            status=transport_models.s.Status1.FW_STATE_INACTIVE,
            container_status='stopped',
            safety=None,
        )

        state = self.firmware.get_state()

        self.assertEqual(state.status, FirmwareStatus.INACTIVE)
        self.assertIsNone(state.safety)

    def test_get_error_report(self):
        self.http_client.get.return_value = transport_models.s.ErrorReport(message='something failed')

        message = self.firmware.get_error_report()

        self.http_client.get.assert_called_once_with(paths_gen.Firmware.GET_ERROR_REPORT,
                                                     transport_models.s.ErrorReport)
        self.assertEqual(message, 'something failed')

    def test_clear_errors(self):
        self.http_client.delete.return_value = transport_models.EmptyPayload()

        self.firmware.clear_errors()

        self.http_client.delete.assert_called_once_with(paths_gen.Firmware.CLEAR_FIRMWARE_ERRORS,
                                                        transport_models.EmptyPayload)

    def test_get_plc_safety_state(self):
        self.http_client.get.return_value = transport_models.s.PlcSafetyState(**PLC_SAFETY_FIELDS)

        state = self.firmware.get_plc_safety_state()

        self.http_client.get.assert_called_once_with(paths_gen.Firmware.GET_PLC_SAFETY_STATE,
                                                     transport_models.s.PlcSafetyState)
        self.assertIsInstance(state, PlcSafetyState)
        self.assertTrue(state.robot_initialized)
        self.assertFalse(state.stop_request)

    def test_set_plc_safety_state(self):
        self.http_client.patch.return_value = transport_models.EmptyPayload()
        state = PlcSafetyState(**PLC_SAFETY_FIELDS)

        self.firmware.set_plc_safety_state(state)

        self.http_client.patch.assert_called_once()
        call_args = self.http_client.patch.call_args[0]
        self.assertEqual(call_args[0], paths_gen.Firmware.SET_PLC_SAFETY_STATE)
        self.assertEqual(call_args[1], transport_models.EmptyPayload)
        self.assertEqual(call_args[2], state.to_transport_model())

    def test_start(self):
        self.http_client.post.return_value = transport_models.EmptyPayload()

        self.firmware.start()

        self.http_client.post.assert_called_once_with(paths_gen.Firmware.FIRMWARE_START,
                                                      transport_models.EmptyPayload,
                                                      transport_models.EmptyPayload())

    def test_stop(self):
        self.http_client.post.return_value = transport_models.EmptyPayload()

        self.firmware.stop()

        self.http_client.post.assert_called_once_with(paths_gen.Firmware.FIRMWARE_STOP, transport_models.EmptyPayload,
                                                      transport_models.EmptyPayload())

    def test_restart(self):
        self.http_client.post.return_value = transport_models.EmptyPayload()

        self.firmware.restart()

        self.http_client.post.assert_called_once_with(paths_gen.Firmware.FIRMWARE_RESTART,
                                                      transport_models.EmptyPayload,
                                                      transport_models.EmptyPayload())

    def test_on_state_subscription(self):
        callback = MagicMock()

        self.firmware.on_state(callback)

        self.mqtt_client.subscribe.assert_called_once()
        call_args = self.mqtt_client.subscribe.call_args[0]
        self.assertEqual(call_args[0], topics_gen.FirmwareState.FIRMWARE_STATE)
        self.assertEqual(call_args[2], transport_models.a.FirmwareState)

    def test_on_state_callback_execution(self):
        callback = MagicMock()

        self.firmware.on_state(callback)
        internal_callback = self.mqtt_client.subscribe.call_args[0][1]

        internal_callback(
            None,
            transport_models.a.FirmwareState(status=transport_models.a.FirmwareStateStatus.ERROR,
                                              container_status='running',
                                              safety=None))

        callback.assert_called_once()
        state = callback.call_args[0][0]
        self.assertIsInstance(state, FirmwareState)
        self.assertEqual(state.status, FirmwareStatus.ERROR)
        self.assertIsNone(state.safety)

    def test_on_state_callback_with_safety_dict(self):
        callback = MagicMock()

        self.firmware.on_state(callback)
        internal_callback = self.mqtt_client.subscribe.call_args[0][1]

        internal_callback(
            None,
            transport_models.a.FirmwareState(status=transport_models.a.FirmwareStateStatus.ACTIVE,
                                              container_status='running',
                                              safety={
                                                  'armed': True,
                                                  'manual_mode': False,
                                                  'movement_authorized': True,
                                                  'protective_stop_active': False,
                                                  'emergency_stop_active': False,
                                                  'max_velocity_factor': 0.5,
                                              }))

        state = callback.call_args[0][0]
        self.assertIsInstance(state.safety, SafetyState)
        self.assertTrue(state.safety.armed)
        self.assertEqual(state.safety.max_velocity_factor, 0.5)

    def test_on_plc_safety_state_subscription(self):
        callback = MagicMock()

        self.firmware.on_plc_safety_state(callback)

        self.mqtt_client.subscribe.assert_called_once()
        call_args = self.mqtt_client.subscribe.call_args[0]
        self.assertEqual(call_args[0], topics_gen.PlcSafety.PLC_SAFETY)
        self.assertEqual(call_args[2], transport_models.a.PlcSafetyState)

    def test_on_plc_safety_state_callback_execution(self):
        callback = MagicMock()

        self.firmware.on_plc_safety_state(callback)
        internal_callback = self.mqtt_client.subscribe.call_args[0][1]

        internal_callback(None, transport_models.a.PlcSafetyState(**PLC_SAFETY_FIELDS))

        callback.assert_called_once()
        state = callback.call_args[0][0]
        self.assertIsInstance(state, PlcSafetyState)
        self.assertTrue(state.robot_initialized)

    def test_enable(self):
        self.http_client.post.return_value = transport_models.EmptyPayload()

        self.firmware.enable()

        path, model, body = self.http_client.post.call_args[0]
        self.assertEqual(path, paths_gen.Firmware.FIRMWARE_ENABLE)
        self.assertEqual(model, transport_models.EmptyPayload)
        self.assertIsInstance(body, transport_models.EmptyPayload)

    def test_wait_armed_returns_once_the_safety_module_reports_armed(self):
        self.mqtt_client.subscribe.return_value = MagicMock()

        def subscribe(topic, callback, model):
            callback(topic, transport_models.a.FirmwareState(
                status=transport_models.a.FirmwareStateStatus.ACTIVE,
                container_status='running',
                safety=dict(armed=True, manual_mode=True, movement_authorized=True, protective_stop_active=False,
                            emergency_stop_active=False, max_velocity_factor=1.0)))
            return self.mqtt_client.subscribe.return_value

        self.mqtt_client.subscribe.side_effect = subscribe

        self.firmware.wait_armed(timeout=1)

        self.mqtt_client.subscribe.return_value.assert_called_once()

    def test_wait_armed_times_out(self):
        self.mqtt_client.subscribe.return_value = MagicMock()

        with self.assertRaises(TimeoutError):
            self.firmware.wait_armed(timeout=0.05)

        self.mqtt_client.subscribe.return_value.assert_called_once()


if __name__ == "__main__":
    unittest.main()
