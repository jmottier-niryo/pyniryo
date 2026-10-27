import logging
import threading
from typing import Callable

from pyniryo.nate.components import BaseAPIComponent

from .._internal import paths_gen, transport_models, topics_gen, mqtt
from ..exceptions import ApiError, DataValidationError
from ..models import IOStates, DigitalIO, AnalogIO, IOLabel, AnyIO, AnyIOLabel, Unsubscribe

logger = logging.getLogger(__name__)

AnalogIOCallback = Callable[[AnalogIO], None]
DigitalIOCallback = Callable[[DigitalIO], None]


def _describe_write(outputs: list[AnyIO]) -> str:
    return ', '.join(f'{io.id}={io.value}' for io in outputs) or 'nothing'


class IO(BaseAPIComponent):

    def get_all(self) -> IOStates:
        """
        Get the current states of all digital and analog inputs and outputs.

        The states are keyed by label, with a label this client does not know keyed by its
        raw string, see :meth:`~pyniryo.nate.models.io.IOLabel.parse`.

        An analog input that measures nothing is reported with ``value`` set to None, see
        :class:`~pyniryo.nate.models.io.AnalogIO`.

        :return: The state of every input and output.
        """
        t_ios = self._http_client.get(paths_gen.Ios.GET_IO_STATES, transport_models.s.IOStates)
        return IOStates.from_transport_model(t_ios)

    def set_digital_output(self, io_id: AnyIOLabel, value: bool) -> None:
        """
        Set the state of a digital output.

        :param io_id: The label of the digital output to set (e.g., IOLabel.CDO1, 'tdo2').
        :param value: The value to set (True for HIGH, False for LOW).
        :raises ClientError: If the robot refuses the write.
        """
        self.update_states([DigitalIO(id=IOLabel.parse(io_id), value=value)])

    def set_analog_output(self, io_id: AnyIOLabel, value: float) -> None:
        """
        Set the voltage of an analog output.

        :param io_id: The label of the analog output to set (e.g., IOLabel.CAO1, 'cao2').
          The tool connector carries no analog output, see :doc:`/api/nate/tool_io`.
        :param value: The voltage to apply, in volts. The robot owns the range of the
          channel and refuses a value outside it.
        :raises ClientError: If the robot refuses the write.
        """
        self.update_states([AnalogIO(id=IOLabel.parse(io_id), value=value)])

    def update_states(self, outputs: list[AnyIO]) -> None:
        """
        Update the states of multiple digital and analog outputs at once.

        :param outputs: A list of DigitalIO and AnalogIO objects representing the outputs to update.
        :raises ValueError: If an analog output carries no value to write.
        :raises ClientError: If the robot refuses the write.
        """
        for output in outputs:
            if isinstance(output, AnalogIO) and output.value is None:
                raise ValueError(f'analog output {output.id} has no value to write')

        req = transport_models.s.IOStates(
            digital_outputs=[i.to_transport_model() for i in outputs if isinstance(i, DigitalIO)],
            analog_outputs=[i.to_transport_model() for i in outputs if isinstance(i, AnalogIO)],
        )
        try:
            self._http_client.put(paths_gen.Ios.UPDATE_IO_STATES, transport_models.EmptyPayload, req)
        except ApiError as e:
            message = f'writing {_describe_write(outputs)} failed: {e.reason}'
            if e.status_code == 504:
                message += '; the outputs did not report the value in time, the write may still have been applied'
            raise type(e)(e.status_code, e.response, message) from e

    def _subscribe_analog(self, topic: str, io_id: AnyIOLabel, callback: AnalogIOCallback) -> Unsubscribe:
        io_label = IOLabel.parse(io_id)

        def internal_callback(_: str, aio: transport_models.a.AnalogIOState) -> None:
            callback(AnalogIO.from_a_transport_model(io_label, aio))

        return self._mqtt_client.subscribe(self._mqtt_client.format(topic, io_id=io_label),
                                           internal_callback,
                                           transport_models.a.AnalogIOState)

    def _subscribe_digital(self, topic: str, io_id: AnyIOLabel, callback: DigitalIOCallback) -> Unsubscribe:
        io_label = IOLabel.parse(io_id)

        def internal_callback(_: str, dio: transport_models.a.DigitalIOState) -> None:
            callback(DigitalIO(id=io_label, value=dio.value))

        return self._mqtt_client.subscribe(self._mqtt_client.format(topic, io_id=io_label),
                                           internal_callback,
                                           transport_models.a.DigitalIOState)

    def on_analog_io(self, io_id: AnyIOLabel, callback: AnalogIOCallback) -> Unsubscribe:
        """
        Set a callback to be called when the state of the specified analog IO changes, whether it is an input or
        an output. See :meth:`on_analog_input` and :meth:`on_analog_output` to follow one direction only.

        :param io_id: An IO label to monitor (e.g., IOLabel.CAI1).
        :param callback: A function that takes an AnalogIO object as an argument and returns None.
          This function will be called whenever the state of the specified analog IOs changes.
          An input that measures nothing is passed with ``value`` set to None, see
          :class:`~pyniryo.nate.models.io.AnalogIO`.
        """
        topic = topics_gen.Io.ANALOG_INPUT_STATE.replace('input', mqtt.Wildcard.SINGLE_LEVEL)
        return self._subscribe_analog(topic, io_id, callback)

    def on_analog_input(self, io_id: AnyIOLabel, callback: AnalogIOCallback) -> Unsubscribe:
        """
        Set a callback to be called when the state of the specified analog input changes. The state is retained,
        so the callback receives the current state as soon as the subscription is in place.

        :param io_id: An analog input label (e.g., IOLabel.CAI1, IOLabel.TAI1).
        :param callback: A function that takes an AnalogIO object as an argument and returns None.
        """
        return self._subscribe_analog(topics_gen.Io.ANALOG_INPUT_STATE, io_id, callback)

    def on_analog_output(self, io_id: AnyIOLabel, callback: AnalogIOCallback) -> Unsubscribe:
        """
        Set a callback to be called when the state of the specified analog output changes. The state is retained,
        so the callback receives the current state as soon as the subscription is in place.

        :param io_id: An analog output label (e.g., IOLabel.CAO1). The tool carries no analog output.
        :param callback: A function that takes an AnalogIO object as an argument and returns None.
        """
        return self._subscribe_analog(topics_gen.Io.ANALOG_OUTPUT_STATE, io_id, callback)

    def on_digital_io(self, io_id: AnyIOLabel, callback: DigitalIOCallback) -> Unsubscribe:
        """
        Set a callback to be called when the state of the specified digital IO changes, whether it is an input or
        an output. See :meth:`on_digital_input` and :meth:`on_digital_output` to follow one direction only.

        :param io_id: An IO label to monitor (e.g., IOLabel.CDI1).
        :param callback: A function that takes a DigitalIO object as an argument and returns None.
          This function will be called whenever the state of the specified digital IOs changes.
        """
        topic = topics_gen.Io.DIGITAL_INPUT_STATE.replace('input', mqtt.Wildcard.SINGLE_LEVEL)
        return self._subscribe_digital(topic, io_id, callback)

    def on_digital_input(self, io_id: AnyIOLabel, callback: DigitalIOCallback) -> Unsubscribe:
        """
        Set a callback to be called when the state of the specified digital input changes. The state is retained,
        so the callback receives the current state as soon as the subscription is in place.

        :param io_id: A digital input label (e.g., IOLabel.CDI1, IOLabel.TDI1).
        :param callback: A function that takes a DigitalIO object as an argument and returns None.
        """
        return self._subscribe_digital(topics_gen.Io.DIGITAL_INPUT_STATE, io_id, callback)

    def on_digital_output(self, io_id: AnyIOLabel, callback: DigitalIOCallback) -> Unsubscribe:
        """
        Set a callback to be called when the state of the specified digital output changes. The state is retained,
        so the callback receives the current state as soon as the subscription is in place.

        :param io_id: A digital output label (e.g., IOLabel.CDO1, IOLabel.TDO1).
        :param callback: A function that takes a DigitalIO object as an argument and returns None.
        """
        return self._subscribe_digital(topics_gen.Io.DIGITAL_OUTPUT_STATE, io_id, callback)

    def wait_for_digital_io(self, io_id: AnyIOLabel, value: bool, timeout: float = -1) -> None:
        """
        Block until a digital input or output holds the given value.

        The current state is read once the subscription is in place, so an IO that already
        holds ``value`` returns at once and a change happening meanwhile is not missed.

        :param io_id: The label of the IO to wait on (e.g., IOLabel.CDI1).
        :param value: The value to wait for.
        :param timeout: Maximum time to wait, in seconds. Zero or a negative value waits
          indefinitely.
        :raises DataValidationError: If the robot serves no digital IO under this label.
        :raises TimeoutError: If the timeout expires before the IO holds the value.
        """
        reached = threading.Event()

        def on_change(io: DigitalIO) -> None:
            if io.value == value:
                reached.set()

        unsubscribe = self.on_digital_io(io_id, on_change)
        try:
            # read after subscribing, so that a change happening in between is caught by the callback
            if self._digital_value(io_id) == value:
                reached.set()
            if not reached.wait(timeout if timeout > 0 else None):
                raise TimeoutError(f'digital IO {io_id} did not reach {value} within {timeout} seconds')
        finally:
            unsubscribe()

    def _digital_value(self, io_id: AnyIOLabel) -> bool:
        states = self.get_all()
        io = states.digital_inputs.get(io_id, states.digital_outputs.get(io_id))
        if io is None:
            raise DataValidationError(f'the robot serves no digital IO {str(io_id)!r}')
        return io.value
