from dataclasses import dataclass
from strenum import StrEnum

from .._internal import transport_models


class IOLabel(StrEnum):
    """
    Labels of the inputs and outputs. Tool labels are prefixed with ``T``, controller
    labels with ``C``.

    See :doc:`/api/nate/tool_io` and :doc:`/api/nate/controller_io` for the board signal
    each label maps to, and the robot user manual for the wiring of the connectors.

    A robot may serve inputs and outputs whose labels are absent from this enumeration,
    added to the hardware after this client was installed. They are reported and
    addressed as raw strings, see :meth:`parse`.
    """
    TDI1 = 'TDI1'
    TDI2 = 'TDI2'
    TDO1 = 'TDO1'
    TDO2 = 'TDO2'
    TAI1 = 'TAI1'
    TAI2 = 'TAI2'

    CDI1 = 'CDI1'
    CDI2 = 'CDI2'
    CDI3 = 'CDI3'
    CDI4 = 'CDI4'
    CDI5 = 'CDI5'
    CDI6 = 'CDI6'
    CDI7 = 'CDI7'
    CDI8 = 'CDI8'
    CDO1 = 'CDO1'
    CDO2 = 'CDO2'
    CDO3 = 'CDO3'
    CDO4 = 'CDO4'
    CDO5 = 'CDO5'
    CDO6 = 'CDO6'
    CDO7 = 'CDO7'
    CDO8 = 'CDO8'
    CAI1 = 'CAI1'
    CAI2 = 'CAI2'
    CAO1 = 'CAO1'
    CAO2 = 'CAO2'

    @classmethod
    def parse(cls, label: str) -> 'AnyIOLabel':
        """
        Resolve a label to its member, or return it uppercased when it is not one of them.

        An unknown label is passed through rather than refused, so that a client keeps
        working against a robot that serves inputs and outputs it does not know. The
        robot is the authority on which labels exist, and refuses a write to a label it
        does not serve.

        :param label: The label to resolve, in any case.
        :return: The matching member, or the uppercased label itself.
        """
        label = str(label).upper()
        try:
            return cls(label)
        except ValueError:
            return label


AnyIOLabel = IOLabel | str

_TransportUnavailableReason = transport_models.s.UnavailableReason | transport_models.a.AnalogIOStateUnavailableReason


class AnalogUnavailableReason(StrEnum):
    """
    Why an analog input carries no measurement.

    ``HIGH_Z`` means the input sits in high impedance, wired to no measurement chain.
    ``RS485`` means the input is driven as an RS485 line, which excludes analog
    acquisition on it.
    """
    HIGH_Z = 'high_z'
    RS485 = 'rs485'

    @classmethod
    def from_transport_model(cls, model: _TransportUnavailableReason | None) -> 'AnalogUnavailableReason | None':
        return None if model is None else cls(model.value)


@dataclass
class DigitalIO:
    id: AnyIOLabel
    value: bool

    @classmethod
    def from_transport_model(cls, model: transport_models.s.DigitalIO) -> 'DigitalIO':
        return cls(id=IOLabel.parse(model.id), value=model.value)

    def to_transport_model(self) -> transport_models.s.DigitalIO:
        return transport_models.s.DigitalIO(id=self.id, value=self.value)


@dataclass
class AnalogIO:
    """
    An analog input or output of the tool or of the controller.

    An input that measures nothing is served without a value: ``value`` is then None and
    ``unavailable_reason`` says why. Test :attr:`is_available` or ``value is None``, never
    a default value, since ``0.0`` is a legitimate measurement.

    :param id: The label of the input or output.
    :param value: The measurement, in volts in voltage mode and in amperes in current
      mode. None when no measurement is available.
    :param unavailable_reason: Why no measurement is available. Set if and only if
      ``value`` is None.
    """
    id: AnyIOLabel
    value: float | None = None
    unavailable_reason: AnalogUnavailableReason | None = None

    @property
    def is_available(self) -> bool:
        """
        Whether the input carries a measurement. False means ``value`` is None and
        ``unavailable_reason`` says why.
        """
        return self.value is not None

    @classmethod
    def from_transport_model(cls, model: transport_models.s.AnalogIO) -> 'AnalogIO':
        return cls(id=IOLabel.parse(model.id),
                   value=model.value,
                   unavailable_reason=AnalogUnavailableReason.from_transport_model(model.unavailable_reason))

    @classmethod
    def from_a_transport_model(cls, io_id: AnyIOLabel, model: transport_models.a.AnalogIOState) -> 'AnalogIO':
        return cls(id=io_id,
                   value=model.value,
                   unavailable_reason=AnalogUnavailableReason.from_transport_model(model.unavailable_reason))

    def to_transport_model(self) -> transport_models.s.AnalogIO:
        # unavailable_reason is left out: on a write the server reads it as "no value to set".
        return transport_models.s.AnalogIO(id=self.id, value=self.value)


@dataclass
class IOStates:
    """
    Every input and output the robot serves, keyed by label.

    A label the client does not know is a plain string key, so a robot serving inputs and
    outputs added after this client was installed is reported whole rather than refused.
    """
    digital_inputs: dict[AnyIOLabel, DigitalIO]
    digital_outputs: dict[AnyIOLabel, DigitalIO]
    analog_inputs: dict[AnyIOLabel, AnalogIO]
    analog_outputs: dict[AnyIOLabel, AnalogIO]

    @classmethod
    def from_transport_model(cls, model: transport_models.s.IOStates) -> 'IOStates':
        return cls(
            digital_inputs={IOLabel.parse(io.id): DigitalIO.from_transport_model(io)
                            for io in model.digital_inputs} if model.digital_inputs is not None else {},
            digital_outputs={IOLabel.parse(io.id): DigitalIO.from_transport_model(io)
                             for io in model.digital_outputs} if model.digital_outputs is not None else {},
            analog_inputs={IOLabel.parse(io.id): AnalogIO.from_transport_model(io)
                           for io in model.analog_inputs} if model.analog_inputs is not None else {},
            analog_outputs={IOLabel.parse(io.id): AnalogIO.from_transport_model(io)
                            for io in model.analog_outputs} if model.analog_outputs is not None else {},
        )

    def to_transport_model(self) -> transport_models.s.IOStates:
        return transport_models.s.IOStates(
            digital_inputs=[io.to_transport_model() for io in self.digital_inputs.values()],
            digital_outputs=[io.to_transport_model() for io in self.digital_outputs.values()],
            analog_inputs=[io.to_transport_model() for io in self.analog_inputs.values()],
            analog_outputs=[io.to_transport_model() for io in self.analog_outputs.values()],
        )


AnyIO = DigitalIO | AnalogIO
