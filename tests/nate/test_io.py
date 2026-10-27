import threading
import unittest
from unittest.mock import MagicMock

from pyniryo.nate._internal import transport_models, paths_gen, topics_gen, mqtt
from pyniryo.nate.components.io import IO, DigitalIO, AnalogIO
from pyniryo.nate.exceptions import ClientError, DataValidationError, ServerError
from pyniryo.nate.models.io import AnalogUnavailableReason, IOStates, IOLabel

from .base import BaseTestComponent, mock_format

CONTROLLER_DIGITAL_OUTPUTS = [IOLabel[f'CDO{number}'] for number in range(1, 9)]
CONTROLLER_ANALOG_OUTPUTS = [IOLabel.CAO1, IOLabel.CAO2]


def topic_matches(subscription: str, topic: str) -> bool:
    """Whether an MQTT topic is served by a subscription holding single level wildcards."""
    subscribed_levels = subscription.split('/')
    topic_levels = topic.split('/')
    if len(subscribed_levels) != len(topic_levels):
        return False
    return all(subscribed == mqtt.Wildcard.SINGLE_LEVEL or subscribed == received
               for subscribed, received in zip(subscribed_levels, topic_levels))


class TestIOLabel(unittest.TestCase):
    """Test the IOLabel enumeration and its parse helper."""

    def test_tool_labels(self):
        """Test the six labels of the tool connector."""
        self.assertEqual([label for label in IOLabel if label.startswith('T')],
                         ['TDI1', 'TDI2', 'TDO1', 'TDO2', 'TAI1', 'TAI2'])

    def test_controller_labels(self):
        """Test the twenty labels of the controller."""
        digital = [f'CDI{number}' for number in range(1, 9)] + [f'CDO{number}' for number in range(1, 9)]
        expected = digital + ['CAI1', 'CAI2', 'CAO1', 'CAO2']
        self.assertEqual([label for label in IOLabel if label.startswith('C')], expected)

    def test_parse_with_valid_label(self):
        """Test that a known label string resolves to the matching member."""
        self.assertEqual(IOLabel.parse('TDO1'), IOLabel.TDO1)
        self.assertIsInstance(IOLabel.parse('CDO8'), IOLabel)

    def test_parse_with_member(self):
        """Test that passing a member through is a no-op."""
        self.assertEqual(IOLabel.parse(IOLabel.TAI2), IOLabel.TAI2)

    def test_parse_uppercases_the_label(self):
        self.assertEqual(IOLabel.parse('tdo2'), IOLabel.TDO2)
        self.assertEqual(IOLabel.parse('cdi9'), 'CDI9')

    def test_parse_with_unknown_label(self):
        """Test that a label of a hardware this version does not know is returned as it is."""
        self.assertEqual(IOLabel.parse('CDI9'), 'CDI9')
        self.assertNotIsInstance(IOLabel.parse('CDI9'), IOLabel)


class TestDigitalIO(unittest.TestCase):
    """Test the DigitalIO model class."""

    def test_from_transport_model_high_value(self):
        """Test converting transport model with HIGH value to DigitalIO."""
        t_model = transport_models.s.DigitalIO(id="TDO1", value=True)
        digital_io = DigitalIO.from_transport_model(t_model)

        self.assertEqual(digital_io.id, IOLabel.TDO1)
        self.assertTrue(digital_io.value)

    def test_from_transport_model_low_value(self):
        """Test converting transport model with LOW value to DigitalIO."""
        t_model = transport_models.s.DigitalIO(id="TDI2", value=False)
        digital_io = DigitalIO.from_transport_model(t_model)

        self.assertEqual(digital_io.id, IOLabel.TDI2)
        self.assertFalse(digital_io.value)

    def test_to_transport_model_true_value(self):
        """Test converting DigitalIO with True value to transport model."""
        digital_io = DigitalIO(id=IOLabel.TDO1, value=True)
        t_model = digital_io.to_transport_model()

        self.assertEqual(t_model.id, IOLabel.TDO1)
        self.assertTrue(t_model.value)

    def test_to_transport_model_false_value(self):
        """Test converting DigitalIO with False value to transport model."""
        digital_io = DigitalIO(id=IOLabel.TDI1, value=False)
        t_model = digital_io.to_transport_model()

        self.assertEqual(t_model.id, IOLabel.TDI1)
        self.assertFalse(t_model.value)

    def test_from_transport_model_with_string_id(self):
        """Test converting transport model with string ID."""
        t_model = transport_models.s.DigitalIO(id="CDO2", value=True)
        digital_io = DigitalIO.from_transport_model(t_model)

        self.assertEqual(digital_io.id, IOLabel.CDO2)
        self.assertTrue(digital_io.value)

    def test_from_transport_model_with_unknown_id(self):
        """Test that an IO of a hardware this version does not know is reported with its raw id."""
        t_model = transport_models.s.DigitalIO(id="CDI9", value=True)
        digital_io = DigitalIO.from_transport_model(t_model)

        self.assertEqual(digital_io.id, "CDI9")
        self.assertTrue(digital_io.value)


class TestAnalogIO(unittest.TestCase):
    """Test the AnalogIO model class."""

    def test_from_transport_model(self):
        """Test converting transport model to AnalogIO."""
        t_model = transport_models.s.AnalogIO(id="TAI1", value=3.14)
        analog_io = AnalogIO.from_transport_model(t_model)

        self.assertEqual(analog_io.id, IOLabel.TAI1)
        self.assertEqual(analog_io.value, 3.14)

    def test_to_transport_model(self):
        """Test converting AnalogIO to transport model."""
        analog_io = AnalogIO(id=IOLabel.CAO1, value=5.0)
        t_model = analog_io.to_transport_model()

        self.assertEqual(t_model.id, IOLabel.CAO1)
        self.assertEqual(t_model.value, 5.0)

    def test_from_transport_model_with_zero_value(self):
        """Test converting transport model with zero value."""
        t_model = transport_models.s.AnalogIO(id="TAI1", value=0.0)
        analog_io = AnalogIO.from_transport_model(t_model)

        self.assertEqual(analog_io.id, IOLabel.TAI1)
        self.assertEqual(analog_io.value, 0.0)

    def test_from_transport_model_keeps_a_reading_out_of_range(self):
        """Test that a reading is exposed as measured, the robot owning the range of its channels."""
        t_model = transport_models.s.AnalogIO(id="CAI2", value=-2.5)
        analog_io = AnalogIO.from_transport_model(t_model)

        self.assertEqual(analog_io.id, IOLabel.CAI2)
        self.assertEqual(analog_io.value, -2.5)

    def test_from_transport_model_available(self):
        """Test that a measured input is available and carries no reason."""
        t_model = transport_models.s.AnalogIO(id="TAI1", value=0.0)
        analog_io = AnalogIO.from_transport_model(t_model)

        self.assertTrue(analog_io.is_available)
        self.assertIsNone(analog_io.unavailable_reason)

    def test_from_transport_model_unavailable(self):
        """Test converting an input served without a measurement."""
        for reason, expected in [(transport_models.s.UnavailableReason.HIGH_Z, AnalogUnavailableReason.HIGH_Z),
                                 (transport_models.s.UnavailableReason.RS485, AnalogUnavailableReason.RS485)]:
            with self.subTest(reason=reason):
                t_model = transport_models.s.AnalogIO(id="TAI1", unavailable_reason=reason)
                analog_io = AnalogIO.from_transport_model(t_model)

                self.assertFalse(analog_io.is_available)
                self.assertIsNone(analog_io.value)
                self.assertEqual(analog_io.unavailable_reason, expected)

    def test_to_transport_model_omits_unavailable_reason(self):
        """Test that the reason is never sent, since a request carrying one writes no value."""
        analog_io = AnalogIO(id=IOLabel.TAI1, value=5.0, unavailable_reason=AnalogUnavailableReason.RS485)
        t_model = analog_io.to_transport_model()

        self.assertEqual(t_model.value, 5.0)
        self.assertIsNone(t_model.unavailable_reason)

    def test_from_transport_model_with_unknown_id(self):
        """Test that an IO of a hardware this version does not know is reported with its raw id."""
        t_model = transport_models.s.AnalogIO(id="CAI3", value=1.0)
        analog_io = AnalogIO.from_transport_model(t_model)

        self.assertEqual(analog_io.id, "CAI3")
        self.assertEqual(analog_io.value, 1.0)


class TestIOStates(unittest.TestCase):
    """Test the IOStates model class."""

    def test_from_transport_model_full(self):
        """Test converting full IOStates from transport model."""
        t_model = transport_models.s.IOStates(
            digital_inputs=[
                transport_models.s.DigitalIO(id="TDI1", value=True),
                transport_models.s.DigitalIO(id="CDI5", value=False),
            ],
            digital_outputs=[
                transport_models.s.DigitalIO(id="TDO1", value=True),
            ],
            analog_inputs=[
                transport_models.s.AnalogIO(id="TAI1", value=3.3),
            ],
            analog_outputs=[
                transport_models.s.AnalogIO(id="CAO1", value=5.0),
            ],
        )

        io_states = IOStates.from_transport_model(t_model)

        self.assertEqual(len(io_states.digital_inputs), 2)
        self.assertEqual(len(io_states.digital_outputs), 1)
        self.assertEqual(len(io_states.analog_inputs), 1)
        self.assertEqual(len(io_states.analog_outputs), 1)

        self.assertIn(IOLabel.TDI1, io_states.digital_inputs)
        self.assertIn(IOLabel.CDI5, io_states.digital_inputs)
        self.assertIn(IOLabel.TDO1, io_states.digital_outputs)
        self.assertIn(IOLabel.TAI1, io_states.analog_inputs)
        self.assertIn(IOLabel.CAO1, io_states.analog_outputs)

        self.assertTrue(io_states.digital_inputs[IOLabel.TDI1].value)
        self.assertFalse(io_states.digital_inputs[IOLabel.CDI5].value)

    def test_from_transport_model_empty(self):
        """Test converting empty IOStates from transport model."""
        t_model = transport_models.s.IOStates(
            digital_inputs=[],
            digital_outputs=[],
            analog_inputs=[],
            analog_outputs=[],
        )

        io_states = IOStates.from_transport_model(t_model)

        self.assertEqual(len(io_states.digital_inputs), 0)
        self.assertEqual(len(io_states.digital_outputs), 0)
        self.assertEqual(len(io_states.analog_inputs), 0)
        self.assertEqual(len(io_states.analog_outputs), 0)

    def test_to_transport_model(self):
        """Test converting IOStates to transport model."""
        io_states = IOStates(
            digital_inputs={IOLabel.TDI1: DigitalIO(id=IOLabel.TDI1, value=True)},
            digital_outputs={IOLabel.CDO1: DigitalIO(id=IOLabel.CDO1, value=False)},
            analog_inputs={IOLabel.TAI1: AnalogIO(id=IOLabel.TAI1, value=3.3)},
            analog_outputs={IOLabel.CAO2: AnalogIO(id=IOLabel.CAO2, value=5.0)},
        )

        t_model = io_states.to_transport_model()

        self.assertIsNotNone(t_model.digital_inputs)
        self.assertEqual(len(t_model.digital_inputs), 1)
        self.assertIsNotNone(t_model.digital_outputs)
        self.assertEqual(len(t_model.digital_outputs), 1)
        self.assertIsNotNone(t_model.analog_inputs)
        self.assertEqual(len(t_model.analog_inputs), 1)
        self.assertIsNotNone(t_model.analog_outputs)
        self.assertEqual(len(t_model.analog_outputs), 1)

        self.assertEqual(t_model.digital_inputs[0].id, IOLabel.TDI1)
        self.assertTrue(t_model.digital_inputs[0].value)

        self.assertEqual(t_model.digital_outputs[0].id, IOLabel.CDO1)
        self.assertFalse(t_model.digital_outputs[0].value)

        self.assertEqual(t_model.analog_inputs[0].id, IOLabel.TAI1)
        self.assertEqual(t_model.analog_inputs[0].value, 3.3)

        self.assertEqual(t_model.analog_outputs[0].id, IOLabel.CAO2)
        self.assertEqual(t_model.analog_outputs[0].value, 5.0)

    def test_from_transport_model_partial(self):
        """Test converting partial IOStates with only some fields populated."""
        t_model = transport_models.s.IOStates(
            digital_inputs=[
                transport_models.s.DigitalIO(id="TDI1", value=True),
            ],
            digital_outputs=[],
            analog_inputs=[],
            analog_outputs=[],
        )

        io_states = IOStates.from_transport_model(t_model)

        self.assertEqual(len(io_states.digital_inputs), 1)
        self.assertEqual(len(io_states.digital_outputs), 0)
        self.assertEqual(len(io_states.analog_inputs), 0)
        self.assertEqual(len(io_states.analog_outputs), 0)

    def test_from_transport_model_with_unknown_id(self):
        """Test that an IO of a hardware this version does not know is keyed by its raw id."""
        t_model = transport_models.s.IOStates(
            digital_inputs=[
                transport_models.s.DigitalIO(id="CDI9", value=True),
                transport_models.s.DigitalIO(id="CDI1", value=False),
            ],
            digital_outputs=[],
            analog_inputs=[],
            analog_outputs=[],
        )

        io_states = IOStates.from_transport_model(t_model)

        self.assertEqual(len(io_states.digital_inputs), 2)
        self.assertTrue(io_states.digital_inputs['CDI9'].value)
        self.assertFalse(io_states.digital_inputs[IOLabel.CDI1].value)


class TestIO(BaseTestComponent):
    """Test the IO component class."""

    def setUp(self):
        super().setUp()
        self.mqtt_client.format.side_effect = mock_format
        self.io = IO(self.http_client, self.mqtt_client, self.correlation_id)

    def test_get_all(self):
        """Test getting all IO states."""
        # Mock the HTTP response
        mock_response = transport_models.s.IOStates(
            digital_inputs=[
                transport_models.s.DigitalIO(id="TDI1", value=True),
                transport_models.s.DigitalIO(id="TDI2", value=False),
            ],
            digital_outputs=[
                transport_models.s.DigitalIO(id="TDO1", value=True),
                transport_models.s.DigitalIO(id="TDO2", value=False),
            ],
            analog_inputs=[
                transport_models.s.AnalogIO(id="TAI1", value=3.3),
                transport_models.s.AnalogIO(id="TAI2", value=5.0),
            ],
            analog_outputs=[],
        )
        self.http_client.get.return_value = mock_response

        # Call get_all
        result = self.io.get_all()

        # Verify HTTP GET was called
        self.http_client.get.assert_called_once_with(paths_gen.Ios.GET_IO_STATES, transport_models.s.IOStates)

        # Verify the result
        self.assertIsInstance(result, IOStates)
        self.assertEqual(len(result.digital_inputs), 2)
        self.assertEqual(len(result.digital_outputs), 2)
        self.assertEqual(len(result.analog_inputs), 2)
        self.assertEqual(len(result.analog_outputs), 0)

    def test_get_all_with_the_controller_ios(self):
        """Test that the twenty controller IOs are all reported, keyed by their label."""
        self.http_client.get.return_value = transport_models.s.IOStates(
            digital_inputs=[
                transport_models.s.DigitalIO(id=f'CDI{number}', value=number % 2 == 0) for number in range(1, 9)
            ],
            digital_outputs=[
                transport_models.s.DigitalIO(id=f'CDO{number}', value=number % 2 == 0) for number in range(1, 9)
            ],
            analog_inputs=[
                transport_models.s.AnalogIO(id='CAI1', value=1.5), transport_models.s.AnalogIO(id='CAI2', value=9.9)
            ],
            analog_outputs=[
                transport_models.s.AnalogIO(id='CAO1', value=0.0), transport_models.s.AnalogIO(id='CAO2', value=10.0)
            ],
        )

        result = self.io.get_all()

        self.assertEqual(len(result.digital_inputs), 8)
        self.assertEqual(len(result.digital_outputs), 8)
        self.assertEqual(result.analog_inputs[IOLabel.CAI2].value, 9.9)
        self.assertEqual(result.analog_outputs[IOLabel.CAO2].value, 10.0)
        for number in range(1, 9):
            self.assertEqual(result.digital_inputs[IOLabel[f'CDI{number}']].value, number % 2 == 0)

    def test_get_all_with_an_unknown_label(self):
        """Test that a payload holding an IO this version does not know is served whole."""
        self.http_client.get.return_value = transport_models.s.IOStates(
            digital_inputs=[
                transport_models.s.DigitalIO(id='TDI1', value=True),
                transport_models.s.DigitalIO(id='CDI9', value=True),
            ],
            digital_outputs=[],
            analog_inputs=[],
            analog_outputs=[],
        )

        result = self.io.get_all()

        self.assertTrue(result.digital_inputs[IOLabel.TDI1].value)
        self.assertTrue(result.digital_inputs['CDI9'].value)

    def test_get_all_when_the_io_chain_is_not_running(self):
        """Test that the caller gets the status code and the explanation the robot gave to act on."""
        self.http_client.get.side_effect = ServerError(503, '{"error": "IO chain is not running"}')

        with self.assertRaises(ServerError) as ctx:
            self.io.get_all()

        self.assertEqual(ctx.exception.status_code, 503)
        self.assertIn('IO chain is not running', str(ctx.exception))

    def test_set_digital_output_high(self):
        """Test setting digital output to HIGH (True)."""
        # Call set_digital_output
        self.io.set_digital_output(IOLabel.TDO1, True)

        # Verify HTTP PUT was called
        self.http_client.put.assert_called_once()
        call_args = self.http_client.put.call_args[0]

        self.assertEqual(call_args[0], paths_gen.Ios.UPDATE_IO_STATES)
        self.assertEqual(call_args[1], transport_models.EmptyPayload)

        # Verify request payload
        request = call_args[2]
        self.assertIsInstance(request, transport_models.s.IOStates)
        self.assertEqual(len(request.digital_outputs), 1)
        self.assertEqual(request.digital_outputs[0].id, IOLabel.TDO1)
        self.assertTrue(request.digital_outputs[0].value)
        self.assertEqual(len(request.analog_outputs), 0)

    def test_set_digital_output_low(self):
        """Test setting digital output to LOW (False)."""
        # Call set_digital_output
        self.io.set_digital_output("TDO2", False)

        # Verify HTTP PUT was called
        self.http_client.put.assert_called_once()
        call_args = self.http_client.put.call_args[0]

        # Verify request payload
        request = call_args[2]
        self.assertEqual(len(request.digital_outputs), 1)
        self.assertEqual(request.digital_outputs[0].id, "TDO2")
        self.assertFalse(request.digital_outputs[0].value)

    def test_set_digital_output_on_every_controller_output(self):
        """Test that each of the eight digital outputs of the controller is written on its own label."""
        for label in CONTROLLER_DIGITAL_OUTPUTS:
            with self.subTest(label=label):
                self.http_client.put.reset_mock()

                self.io.set_digital_output(label, True)

                request = self.http_client.put.call_args[0][2]
                self.assertEqual(len(request.digital_outputs), 1)
                self.assertEqual(request.digital_outputs[0].id, label)
                self.assertTrue(request.digital_outputs[0].value)

    def test_set_digital_output_with_unknown_string_id(self):
        """Test that a label this version does not know is sent as it is, for the robot to refuse."""
        self.io.set_digital_output("CDO9", True)

        request = self.http_client.put.call_args[0][2]
        self.assertEqual(request.digital_outputs[0].id, "CDO9")

    def test_set_analog_output(self):
        """Test setting analog output value."""
        # Call set_analog_output
        self.io.set_analog_output(IOLabel.CAO1, 4.5)

        # Verify HTTP PUT was called
        self.http_client.put.assert_called_once()
        call_args = self.http_client.put.call_args[0]

        self.assertEqual(call_args[0], paths_gen.Ios.UPDATE_IO_STATES)
        self.assertEqual(call_args[1], transport_models.EmptyPayload)

        # Verify request payload
        request = call_args[2]
        self.assertIsInstance(request, transport_models.s.IOStates)
        self.assertEqual(len(request.analog_outputs), 1)
        self.assertEqual(request.analog_outputs[0].id, IOLabel.CAO1)
        self.assertEqual(request.analog_outputs[0].value, 4.5)
        self.assertEqual(len(request.digital_outputs), 0)

    def test_set_analog_output_with_string_id(self):
        """Test setting analog output using string ID."""
        # Call set_analog_output with string ID
        self.io.set_analog_output("CAO2", 2.7)

        # Verify HTTP PUT was called
        self.http_client.put.assert_called_once()
        call_args = self.http_client.put.call_args[0]

        # Verify request payload
        request = call_args[2]
        self.assertEqual(len(request.analog_outputs), 1)
        self.assertEqual(request.analog_outputs[0].id, "CAO2")
        self.assertEqual(request.analog_outputs[0].value, 2.7)

    def test_set_analog_output_on_every_controller_output(self):
        """Test that both analog outputs of the controller are written in volts on their own label."""
        for label, volts in zip(CONTROLLER_ANALOG_OUTPUTS, [0.0, 7.5]):
            with self.subTest(label=label):
                self.http_client.put.reset_mock()

                self.io.set_analog_output(label, volts)

                request = self.http_client.put.call_args[0][2]
                self.assertEqual(len(request.analog_outputs), 1)
                self.assertEqual(request.analog_outputs[0].id, label)
                self.assertEqual(request.analog_outputs[0].value, volts)

    def test_set_analog_output_out_of_range(self):
        """Test that a refusal names the channel, the value sent and what the robot said about it."""
        self.http_client.put.side_effect = ClientError(400, '{"error": "CAO1 accepts 0.0 V to 10.0 V"}')

        with self.assertRaises(ClientError) as ctx:
            self.io.set_analog_output(IOLabel.CAO1, 12.5)

        message = str(ctx.exception)
        self.assertIn('CAO1', message)
        self.assertIn('12.5', message)
        self.assertIn('0.0 V to 10.0 V', message)

    def test_update_states_keeps_the_family_of_the_error(self):
        """Test that naming the write in the message does not move an error between the two families."""
        for error, expected in [(ClientError(400, '{"error": "no such output"}'), ClientError),
                                (ServerError(503, '{"error": "IO chain is not running"}'), ServerError)]:
            with self.subTest(status_code=error.status_code):
                self.http_client.put.side_effect = error

                with self.assertRaises(expected) as ctx:
                    self.io.set_digital_output(IOLabel.CDO1, True)

                self.assertIs(type(ctx.exception), expected)
                self.assertEqual(ctx.exception.status_code, error.status_code)
                self.assertIn('CDO1', str(ctx.exception))
                self.assertIn(error.reason, str(ctx.exception))

    def test_update_states_digital_only(self):
        """Test updating only digital outputs."""
        outputs = [
            DigitalIO(id=IOLabel.TDO1, value=True),
            DigitalIO(id=IOLabel.TDO2, value=False),
        ]

        # Call update_states
        self.io.update_states(outputs)

        # Verify HTTP PUT was called
        self.http_client.put.assert_called_once()
        call_args = self.http_client.put.call_args[0]

        # Verify request payload
        request = call_args[2]
        self.assertEqual(len(request.digital_outputs), 2)
        self.assertEqual(len(request.analog_outputs), 0)

        self.assertEqual(request.digital_outputs[0].id, IOLabel.TDO1)
        self.assertTrue(request.digital_outputs[0].value)
        self.assertEqual(request.digital_outputs[1].id, IOLabel.TDO2)
        self.assertFalse(request.digital_outputs[1].value)

    def test_update_states_analog_only(self):
        """Test updating only analog outputs."""
        outputs = [
            AnalogIO(id=IOLabel.CAO1, value=3.3),
            AnalogIO(id=IOLabel.CAO2, value=5.0),
        ]

        # Call update_states
        self.io.update_states(outputs)

        # Verify HTTP PUT was called
        self.http_client.put.assert_called_once()
        call_args = self.http_client.put.call_args[0]

        # Verify request payload
        request = call_args[2]
        self.assertEqual(len(request.digital_outputs), 0)
        self.assertEqual(len(request.analog_outputs), 2)

        self.assertEqual(request.analog_outputs[0].id, IOLabel.CAO1)
        self.assertEqual(request.analog_outputs[0].value, 3.3)
        self.assertEqual(request.analog_outputs[1].id, IOLabel.CAO2)
        self.assertEqual(request.analog_outputs[1].value, 5.0)

    def test_update_states_mixed(self):
        """Test updating both digital and analog outputs."""
        outputs = [
            DigitalIO(id=IOLabel.TDO1, value=True),
            AnalogIO(id=IOLabel.CAO1, value=4.2),
            DigitalIO(id=IOLabel.TDO2, value=False),
            AnalogIO(id=IOLabel.CAO2, value=1.8),
        ]

        # Call update_states
        self.io.update_states(outputs)

        # Verify HTTP PUT was called
        self.http_client.put.assert_called_once()
        call_args = self.http_client.put.call_args[0]

        # Verify request payload
        request = call_args[2]
        self.assertEqual(len(request.digital_outputs), 2)
        self.assertEqual(len(request.analog_outputs), 2)

    def test_update_states_rejects_analog_output_without_value(self):
        """Test that an analog output with no value is refused instead of written as zero."""
        with self.assertRaises(ValueError):
            self.io.update_states([AnalogIO(id=IOLabel.CAO1)])

        self.http_client.put.assert_not_called()

    def test_update_states_refusal_names_every_output_sent(self):
        """Test that a refused grouped write names the whole batch, the robot refusing it as a whole."""
        self.http_client.put.side_effect = ClientError(400, '{"error": "CDO9 is not an output of this robot"}')

        with self.assertRaises(ClientError) as ctx:
            self.io.update_states([DigitalIO(id=IOLabel.CDO1, value=True), DigitalIO(id='CDO9', value=True)])

        message = str(ctx.exception)
        self.assertIn('CDO1', message)
        self.assertIn('CDO9', message)

    def test_update_states_empty_list(self):
        """Test updating with empty list."""
        outputs = []

        # Call update_states
        self.io.update_states(outputs)

        # Verify HTTP PUT was called
        self.http_client.put.assert_called_once()
        call_args = self.http_client.put.call_args[0]

        # Verify request payload
        request = call_args[2]
        self.assertEqual(len(request.digital_outputs), 0)
        self.assertEqual(len(request.analog_outputs), 0)

    def test_update_states_says_a_timed_out_write_may_have_been_applied(self):
        self.http_client.put.side_effect = ServerError(504, '{"error": "CDO1 did not report in time"}')

        with self.assertRaises(ServerError) as ctx:
            self.io.set_digital_output(IOLabel.CDO1, True)

        self.assertIn('may still have been applied', str(ctx.exception))

    def test_on_digital_input_and_output_subscribe_to_their_own_topic(self):
        for method, io_id, topic in ((self.io.on_digital_input, IOLabel.CDI1, 'device/{device_id}/io/digital/input/CDI1/state'),
                                     (self.io.on_digital_output, IOLabel.CDO1, 'device/{device_id}/io/digital/output/CDO1/state')):
            with self.subTest(method=method.__name__):
                self.mqtt_client.subscribe.reset_mock()
                callback = MagicMock()

                method(io_id, callback)

                subscribed_topic, internal_callback, model = self.mqtt_client.subscribe.call_args[0]
                self.assertEqual(subscribed_topic, topic)
                self.assertEqual(model, transport_models.a.DigitalIOState)
                internal_callback(subscribed_topic, transport_models.a.DigitalIOState(value=True))
                callback.assert_called_once_with(DigitalIO(id=io_id, value=True))

    def test_on_analog_input_and_output_subscribe_to_their_own_topic(self):
        for method, io_id, topic in ((self.io.on_analog_input, IOLabel.CAI1, 'device/{device_id}/io/analog/input/CAI1/state'),
                                     (self.io.on_analog_output, IOLabel.CAO1, 'device/{device_id}/io/analog/output/CAO1/state')):
            with self.subTest(method=method.__name__):
                self.mqtt_client.subscribe.reset_mock()
                callback = MagicMock()

                method(io_id, callback)

                subscribed_topic, internal_callback, model = self.mqtt_client.subscribe.call_args[0]
                self.assertEqual(subscribed_topic, topic)
                self.assertEqual(model, transport_models.a.AnalogIOState)
                internal_callback(subscribed_topic, transport_models.a.AnalogIOState(value=2.5))
                callback.assert_called_once_with(AnalogIO(id=io_id, value=2.5))

    def test_on_digital_io_subscription(self):
        """Test subscribing to digital IO state changes."""
        callback = MagicMock()

        # Call on_digital_io
        self.io.on_digital_io(IOLabel.TDI1, callback)

        # Verify MQTT subscribe was called
        self.mqtt_client.subscribe.assert_called()
        call_args = self.mqtt_client.subscribe.call_args[0]

        self.assertEqual(call_args[0], 'device/{device_id}/io/digital/+/TDI1/state')
        self.assertEqual(call_args[2], transport_models.a.DigitalIOState)

    def test_on_digital_io_callback_execution(self):
        """Test that digital IO callback receives correct DigitalIO object."""
        callback = MagicMock()

        # Call on_digital_io
        self.io.on_digital_io(IOLabel.TDI2, callback)

        # Get the internal callback
        internal_callback = self.mqtt_client.subscribe.call_args[0][1]

        # Simulate MQTT message
        mock_transport_state = transport_models.a.DigitalIOState(value=True)
        internal_callback(None, mock_transport_state)

        # Verify callback was called with correct DigitalIO
        callback.assert_called()
        digital_io = callback.call_args[0][0]
        self.assertIsInstance(digital_io, DigitalIO)
        self.assertEqual(digital_io.id, IOLabel.TDI2)
        self.assertTrue(digital_io.value)

    def test_on_digital_io_callback_with_false_value(self):
        """Test digital IO callback with False value."""
        callback = MagicMock()

        # Call on_digital_io
        self.io.on_digital_io("CDO1", callback)

        # Get the internal callback
        internal_callback = self.mqtt_client.subscribe.call_args[0][1]

        # Simulate MQTT message with False value
        mock_transport_state = transport_models.a.DigitalIOState(value=False)
        internal_callback(None, mock_transport_state)

        # Verify callback was called with correct DigitalIO
        callback.assert_called_once()
        digital_io = callback.call_args[0][0]
        self.assertFalse(digital_io.value)

    def test_on_digital_io_with_unknown_string_id(self):
        """Test that a label this version does not know is subscribed to as it is."""
        callback = MagicMock()

        self.io.on_digital_io("CDI9", callback)

        internal_callback = self.mqtt_client.subscribe.call_args[0][1]
        internal_callback(None, transport_models.a.DigitalIOState(value=True))

        self.assertEqual(callback.call_args[0][0].id, "CDI9")

    def test_on_digital_io_returns_the_unsubscribe_handle(self):
        """Test that the caller gets the handle that stops the callbacks, and not a new one."""
        handle = MagicMock()
        self.mqtt_client.subscribe.return_value = handle

        self.assertIs(self.io.on_digital_io(IOLabel.CDI1, MagicMock()), handle)
        self.assertIs(self.io.on_analog_io(IOLabel.CAI1, MagicMock()), handle)

    def test_on_analog_io_subscription(self):
        """Test subscribing to analog IO state changes."""
        callback = MagicMock()

        # Call on_analog_io
        self.io.on_analog_io(IOLabel.CAI1, callback)

        # Verify MQTT subscribe was called
        self.mqtt_client.subscribe.assert_called()
        call_args = self.mqtt_client.subscribe.call_args[0]

        self.assertEqual(call_args[0], 'device/{device_id}/io/analog/+/CAI1/state')
        self.assertEqual(call_args[2], transport_models.a.AnalogIOState)

    def test_on_analog_io_callback_execution(self):
        """Test that analog IO callback receives correct AnalogIO object."""
        callback = MagicMock()

        # Call on_analog_io
        self.io.on_analog_io(IOLabel.CAI2, callback)

        # Get the internal callback
        internal_callback = self.mqtt_client.subscribe.call_args[0][1]

        # Simulate MQTT message
        mock_transport_state = transport_models.a.AnalogIOState(value=3.7)
        internal_callback(None, mock_transport_state)

        # Verify callback was called with correct AnalogIO
        callback.assert_called_once()
        analog_io = callback.call_args[0][0]
        self.assertIsInstance(analog_io, AnalogIO)
        self.assertEqual(analog_io.id, IOLabel.CAI2)
        self.assertEqual(analog_io.value, 3.7)

    def test_on_analog_io_callback_with_zero_value(self):
        """Test analog IO callback with zero value."""
        callback = MagicMock()

        # Call on_analog_io
        self.io.on_analog_io("TAI1", callback)

        # Get the internal callback
        internal_callback = self.mqtt_client.subscribe.call_args[0][1]

        # Simulate MQTT message with zero value
        mock_transport_state = transport_models.a.AnalogIOState(value=0.0)
        internal_callback(None, mock_transport_state)

        # Verify callback was called with correct AnalogIO
        callback.assert_called_once()
        analog_io = callback.call_args[0][0]
        self.assertEqual(analog_io.value, 0.0)

    def test_on_analog_io_callback_unavailable(self):
        """Test that the analog IO callback carries the reason when nothing is measured."""
        callback = MagicMock()

        # Call on_analog_io
        self.io.on_analog_io(IOLabel.TAI1, callback)

        # Get the internal callback
        internal_callback = self.mqtt_client.subscribe.call_args[0][1]

        # Simulate MQTT message from an input that measures nothing
        mock_transport_state = transport_models.a.AnalogIOState(
            unavailable_reason=transport_models.a.AnalogIOStateUnavailableReason.HIGH_Z)
        internal_callback(None, mock_transport_state)

        # Verify the callback can tell the absence of a measurement from a measurement
        callback.assert_called_once()
        analog_io = callback.call_args[0][0]
        self.assertFalse(analog_io.is_available)
        self.assertIsNone(analog_io.value)
        self.assertEqual(analog_io.unavailable_reason, AnalogUnavailableReason.HIGH_Z)

    def test_on_analog_io_with_string_id(self):
        """Test subscribing to analog IO using string ID."""
        callback = MagicMock()

        # Call on_analog_io with string ID
        self.io.on_analog_io("CAI2", callback)

        # Verify MQTT subscribe was called
        self.mqtt_client.subscribe.assert_called()
        call_args = self.mqtt_client.subscribe.call_args[0]

        self.assertEqual(call_args[0], 'device/{device_id}/io/analog/+/CAI2/state')

    def test_on_analog_io_with_unknown_string_id(self):
        """Test that a label this version does not know is subscribed to as it is."""
        callback = MagicMock()

        self.io.on_analog_io("CAI3", callback)

        internal_callback = self.mqtt_client.subscribe.call_args[0][1]
        internal_callback(None, transport_models.a.AnalogIOState(value=1.0))

        self.assertEqual(callback.call_args[0][0].id, "CAI3")

    def test_on_digital_io_with_string_id(self):
        """Test subscribing to digital IO using string ID."""
        callback = MagicMock()

        # Call on_digital_io with string ID
        self.io.on_digital_io("CDI2", callback)

        # Verify MQTT subscribe was called
        self.mqtt_client.subscribe.assert_called()
        call_args = self.mqtt_client.subscribe.call_args[0]

        self.assertEqual(call_args[0], 'device/{device_id}/io/digital/+/CDI2/state')


class TestWaitForDigitalIO(BaseTestComponent):
    """Test the helper that blocks until a digital IO holds a value."""

    def setUp(self):
        super().setUp()
        self.mqtt_client.format.side_effect = mock_format
        self.io = IO(self.http_client, self.mqtt_client, self.correlation_id)
        self.unsubscribe = MagicMock()
        self.mqtt_client.subscribe.return_value = self.unsubscribe
        self.serve_state(False)

    def serve_state(self, value, output=False):
        """What GET /api/io answers for CDI1. A value of None serves a robot without that IO."""
        ios = [] if value is None else [transport_models.s.DigitalIO(id='CDI1', value=value)]
        self.http_client.get.return_value = transport_models.s.IOStates(
            digital_inputs=[] if output else ios,
            digital_outputs=ios if output else [],
            analog_inputs=[],
            analog_outputs=[],
        )

    def publish_on_subscribe(self, *values: bool, then: bool | None = None):
        """Make the subscription serve the given states, and a later one from another thread."""

        def subscribe(topic, callback, payload_model):
            for value in values:
                callback(topic, transport_models.a.DigitalIOState(value=value))
            if then is not None:
                threading.Thread(target=callback, args=(topic, transport_models.a.DigitalIOState(value=then))).start()
            return self.unsubscribe

        self.mqtt_client.subscribe.side_effect = subscribe

    def test_returns_on_the_state_read_after_subscribing(self):
        """Test that the wait ends on the current state even when the subscription serves nothing."""
        self.serve_state(True)

        self.io.wait_for_digital_io(IOLabel.CDI1, True, timeout=5)

        self.unsubscribe.assert_called_once()

    def test_returns_on_the_state_the_subscription_serves(self):
        """Test that the wait ends on the state the robot published, without waiting for the read."""
        self.publish_on_subscribe(True)

        self.io.wait_for_digital_io(IOLabel.CDI1, True, timeout=5)

        self.unsubscribe.assert_called_once()

    def test_returns_on_a_transition(self):
        """Test that the wait ends on the change of state."""
        self.publish_on_subscribe(False, then=True)

        self.io.wait_for_digital_io(IOLabel.CDI1, True, timeout=5)

        self.unsubscribe.assert_called_once()

    def test_watches_an_output_too(self):
        """Test that the current state of an output is read like the one of an input."""
        self.serve_state(True, output=True)

        self.io.wait_for_digital_io(IOLabel.CDI1, True, timeout=5)

        self.unsubscribe.assert_called_once()

    def test_raises_on_timeout(self):
        """Test that an IO that never holds the value raises rather than blocking the program."""
        self.publish_on_subscribe(False)

        with self.assertRaises(TimeoutError) as ctx:
            self.io.wait_for_digital_io(IOLabel.CDI1, True, timeout=0.05)

        self.assertIn('CDI1', str(ctx.exception))
        self.unsubscribe.assert_called_once()

    def test_raises_on_a_label_the_robot_serves_no_digital_io_under(self):
        """Test that a label with nothing behind it is refused rather than waited on forever."""
        self.serve_state(None)

        with self.assertRaises(DataValidationError) as ctx:
            self.io.wait_for_digital_io(IOLabel.CDI1, True)

        self.assertIn('CDI1', str(ctx.exception))
        self.unsubscribe.assert_called_once()


class TestIOSubscriptionFilter(BaseTestComponent):
    """
    Test that the subscription of one IO serves that IO only.

    on_digital_io and on_analog_io replace the direction of the topic with a wildcard, so
    that one call covers an input as well as an output.
    """

    def setUp(self):
        super().setUp()
        self.mqtt_client.format.side_effect = mock_format
        self.io = IO(self.http_client, self.mqtt_client, self.correlation_id)

    def subscribed_topic(self, subscribe) -> str:
        self.mqtt_client.subscribe.reset_mock()
        subscribe()
        return str(self.mqtt_client.subscribe.call_args[0][0])

    def test_only_the_direction_is_wildcarded(self):
        """Test that the wildcard replaces the direction and nothing else in the topic."""
        for subscribe in [
                lambda: self.io.on_digital_io(IOLabel.CDI1, MagicMock()),
                lambda: self.io.on_analog_io(IOLabel.CAI1, MagicMock())
        ]:
            topic = self.subscribed_topic(subscribe)
            with self.subTest(topic=topic):
                self.assertEqual(topic.count(mqtt.Wildcard.SINGLE_LEVEL), 1)
                self.assertEqual(topic.split('/')[-3], mqtt.Wildcard.SINGLE_LEVEL)

    def test_a_subscription_serves_both_directions_of_its_own_label_only(self):
        """Test that no label is caught by the subscription of another one."""
        states = {
            label: [
                mock_format(topic, io_id=label) for topic in [
                    topics_gen.Io.DIGITAL_INPUT_STATE,
                    topics_gen.Io.DIGITAL_OUTPUT_STATE,
                    topics_gen.Io.ANALOG_INPUT_STATE,
                    topics_gen.Io.ANALOG_OUTPUT_STATE, ]
            ]
            for label in IOLabel
        }

        for label in IOLabel:
            digital = self.subscribed_topic(lambda io_id=label: self.io.on_digital_io(io_id, MagicMock()))
            analog = self.subscribed_topic(lambda io_id=label: self.io.on_analog_io(io_id, MagicMock()))
            with self.subTest(label=label):
                digital_input, digital_output, analog_input, analog_output = states[label]
                self.assertTrue(topic_matches(digital, digital_input))
                self.assertTrue(topic_matches(digital, digital_output))
                self.assertTrue(topic_matches(analog, analog_input))
                self.assertTrue(topic_matches(analog, analog_output))
                self.assertFalse(topic_matches(digital, analog_input))
                self.assertFalse(topic_matches(analog, digital_input))

                for other, other_states in states.items():
                    if other == label:
                        continue
                    for state in other_states:
                        self.assertFalse(topic_matches(digital, state), f'{digital} serves {state}')
                        self.assertFalse(topic_matches(analog, state), f'{analog} serves {state}')


class TestIOIntegration(BaseTestComponent):
    """Integration tests for IO component."""

    def setUp(self):
        super().setUp()
        self.mqtt_client.format.side_effect = mock_format
        self.io = IO(self.http_client, self.mqtt_client, self.correlation_id)

    def test_full_workflow_get_and_update(self):
        """Test complete workflow: get states, modify, and update."""
        # Mock the HTTP GET response
        mock_response = transport_models.s.IOStates(
            digital_inputs=[],
            digital_outputs=[
                transport_models.s.DigitalIO(id="CDO1", value=False),
            ],
            analog_inputs=[],
            analog_outputs=[
                transport_models.s.AnalogIO(id="CAO1", value=0.0),
            ],
        )
        self.http_client.get.return_value = mock_response

        # Get current states
        states = self.io.get_all()

        # Verify get was called
        self.http_client.get.assert_called_once()
        self.assertFalse(states.digital_outputs[IOLabel.CDO1].value)

        # Update outputs
        self.io.set_digital_output(IOLabel.CDO1, True)
        self.io.set_analog_output(IOLabel.CAO1, 5.0)

        # Verify put was called twice
        self.assertEqual(self.http_client.put.call_count, 2)

    def test_multiple_subscriptions(self):
        """Test subscribing to multiple IOs."""
        callback1 = MagicMock()
        callback2 = MagicMock()
        callback3 = MagicMock()

        # Subscribe to multiple IOs
        self.io.on_digital_io(IOLabel.TDI1, callback1)
        self.io.on_digital_io(IOLabel.CDI1, callback2)
        self.io.on_analog_io(IOLabel.CAI1, callback3)

        # Verify all subscriptions were made
        self.assertEqual(self.mqtt_client.subscribe.call_count, 3)


if __name__ == "__main__":
    unittest.main()
