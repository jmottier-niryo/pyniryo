import threading
from typing import Callable

from .base_api_component import BaseAPIComponent
from .._internal import paths_gen, transport_models, topics_gen
from ..models import FirmwareState, PlcSafetyState, Unsubscribe

FirmwareStateCallback = Callable[[FirmwareState], None]
PlcSafetyStateCallback = Callable[[PlcSafetyState], None]


class Firmware(BaseAPIComponent):
    """
    Firmware component for accessing the state of the robot firmware and the PLC safety
    chain, and for controlling the firmware container.
    """

    def get_state(self) -> FirmwareState:
        """
        Get the current state of the robot firmware.

        :return: The current firmware state.
        """
        resp = self._http_client.get(paths_gen.Firmware.GET_FIRMWARE_STATE, transport_models.s.FirmwareState)
        return FirmwareState.from_transport_model(resp)

    def get_error_report(self) -> str:
        """
        Get the current firmware error report.

        :return: A human-readable message describing the current firmware error, if any.
        """
        resp = self._http_client.get(paths_gen.Firmware.GET_ERROR_REPORT, transport_models.s.ErrorReport)
        return resp.message

    def clear_errors(self) -> None:
        """
        Clear the current firmware errors.
        """
        self._http_client.delete(paths_gen.Firmware.CLEAR_FIRMWARE_ERRORS, transport_models.EmptyPayload)

    def get_plc_safety_state(self) -> PlcSafetyState:
        """
        Get the current state of the PLC safety chain.

        :return: The current PLC safety state.
        """
        resp = self._http_client.get(paths_gen.Firmware.GET_PLC_SAFETY_STATE, transport_models.s.PlcSafetyState)
        return PlcSafetyState.from_transport_model(resp)

    def set_plc_safety_state(self, state: PlcSafetyState) -> None:
        """
        Set the state of the PLC safety chain. Only available when safety is simulated: on a
        real robot the PLC state is owned by the wired safety chain and cannot be set through
        the API.

        :param state: The PLC safety state to set.
        """
        self._http_client.patch(paths_gen.Firmware.SET_PLC_SAFETY_STATE, transport_models.EmptyPayload,
                                state.to_transport_model())

    def enable(self) -> None:
        """
        Request the arming of the robot through the safety controller, the same path as the physical initialization
        button. Available while disarmed.

        Acceptance is not arming: observe ``safety.armed`` with :meth:`on_state` or :meth:`get_state`, or call
        :meth:`wait_armed`. The robot bounds the request to two seconds. Do not retry after a timeout, the impulse
        may already have been transmitted; this client retries connection failures only, never a response.

        :raises ClientError: If the request is refused, an impulse already pending included.
        :raises ServerError: If the safety controller cannot be reached (503), did not answer in time (504), or
            failed.
        """
        self._http_client.post(paths_gen.Firmware.FIRMWARE_ENABLE, transport_models.EmptyPayload,
                               transport_models.EmptyPayload())

    def wait_armed(self, timeout: float = -1) -> None:
        """
        Block until the safety module reports the robot armed. The firmware state is retained, so a robot already
        armed returns at once.

        :param timeout: Maximum time to wait, in seconds. Zero or a negative value waits indefinitely.
        :raises TimeoutError: If the timeout expires before the robot is armed.
        """
        armed = threading.Event()

        def on_state(state: FirmwareState) -> None:
            if state.safety is not None and state.safety.armed:
                armed.set()

        unsubscribe = self.on_state(on_state)
        try:
            if not armed.wait(timeout if timeout > 0 else None):
                raise TimeoutError(f'the robot was not armed within {timeout} seconds')
        finally:
            unsubscribe()

    def start(self) -> None:
        """
        Start the firmware container.
        """
        self._http_client.post(paths_gen.Firmware.FIRMWARE_START, transport_models.EmptyPayload,
                               transport_models.EmptyPayload())

    def stop(self) -> None:
        """
        Stop the firmware container.
        """
        self._http_client.post(paths_gen.Firmware.FIRMWARE_STOP, transport_models.EmptyPayload,
                               transport_models.EmptyPayload())

    def restart(self) -> None:
        """
        Restart the firmware container.
        """
        self._http_client.post(paths_gen.Firmware.FIRMWARE_RESTART, transport_models.EmptyPayload,
                               transport_models.EmptyPayload())

    def on_state(self, callback: FirmwareStateCallback) -> Unsubscribe:
        """
        Set a callback to be called when the firmware state changes.

        :param callback: The callback function that takes a FirmwareState parameter.
        """

        def internal_callback(_: str, state: transport_models.a.FirmwareState) -> None:
            callback(FirmwareState.from_a_transport_model(state))

        return self._mqtt_client.subscribe(topics_gen.FirmwareState.FIRMWARE_STATE, internal_callback,
                                           transport_models.a.FirmwareState)

    def on_plc_safety_state(self, callback: PlcSafetyStateCallback) -> Unsubscribe:
        """
        Set a callback to be called when the PLC safety state changes.

        :param callback: The callback function that takes a PlcSafetyState parameter.
        """

        def internal_callback(_: str, state: transport_models.a.PlcSafetyState) -> None:
            callback(PlcSafetyState.from_transport_model(state))

        return self._mqtt_client.subscribe(topics_gen.PlcSafety.PLC_SAFETY, internal_callback,
                                           transport_models.a.PlcSafetyState)
