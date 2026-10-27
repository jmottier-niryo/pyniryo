Tool inputs and outputs
=====================================

The tool connector on the flange carries six inputs and outputs. The IOs of the tool are
prefixed with ``t``, and the robot user manual documents their wiring on the connector:
pin assignment, wire colors, ground and supply. The controller carries twenty more, see
:doc:`/api/nate/controller_io`.

Mapping
-------------------------------------

The labels of this API and the board signals of the robot documentation name the same six
signals:

.. list-table:: Tool IO mapping
   :header-rows: 1
   :widths: auto
   :align: center

   *  -  Label
      -  Board signal
   *  -  ``tdi1``
      -  ``TI0``
   *  -  ``tdi2``
      -  ``TI1``
   *  -  ``tdo1``
      -  ``TO0``
   *  -  ``tdo2``
      -  ``TO1``
   *  -  ``tai1``
      -  ``AI1``
   *  -  ``tai2``
      -  ``AI2``

A digital label does not repeat the number of its board signal: ``tdi1`` is ``TI0`` and
``tdo1`` is ``TO0``. The analog labels do, ``tai1`` is ``AI1``. The connector exposes no
analog output.

Reading and writing
-------------------------------------

:meth:`~pyniryo.nate.components.io.IO.get_all` returns every input and output at once,
keyed by label:

.. code-block:: python
   :linenos:

    from pyniryo.nate import Nate
    from pyniryo.nate.models import IOLabel

    nate = Nate(hostname="10.10.10.10", login=("user", "secret"))

    states = nate.io.get_all()
    print(states.digital_inputs[IOLabel.TDI1].value)

    nate.io.set_digital_output(IOLabel.TDO1, True)

:meth:`~pyniryo.nate.components.io.IO.on_digital_io` and
:meth:`~pyniryo.nate.components.io.IO.on_analog_io` report a single IO as it changes,
without polling:

.. code-block:: python
   :linenos:

    unsubscribe = nate.io.on_digital_io(IOLabel.TDI1, lambda io: print(io.value))

    # later, to stop receiving the changes
    unsubscribe()

Controlling a gripper
-------------------------------------

This section covers a gripper whose tool IOs are configured for digital IO control, where
the gripper is driven through the digital outputs ``tdo1`` and ``tdo2``. The tool IOs can
also be configured for RS485 control, where the gripper is driven over serial through the
analog IOs instead; that mode is out of the scope of this document.

There is no dedicated method for a gripper in this API: it is controlled with the same
:meth:`~pyniryo.nate.components.io.IO.update_states` used for any digital output. Prefer
sending ``tdo1`` and ``tdo2`` together in a single grouped call over two separate
:meth:`~pyniryo.nate.components.io.IO.set_digital_output` calls: written one after the
other, the outputs pass through an intermediate combined state that may not correspond to
any behavior the gripper was configured for.

.. code-block:: python
   :linenos:

    from pyniryo.nate import Nate
    from pyniryo.nate.models import DigitalIO, IOLabel

    nate = Nate(hostname="10.10.10.10", login=("user", "secret"))

    # The states to write for a given behavior depend on how the gripper is configured.
    nate.io.update_states([
        DigitalIO(id=IOLabel.TDO1, value=False),
        DigitalIO(id=IOLabel.TDO2, value=False),
    ])

.. danger::
    A gripper's behavior for a given pair of ``tdo1``/``tdo2`` states is configured by the
    user, on the gripper or in its configuration. Nothing guarantees that, say,
    ``tdo1=False, tdo2=False`` opens the gripper and ``tdo1=True, tdo2=True`` closes it:
    another gripper, or the same one reconfigured, can wire these states to the opposite
    behavior, or to something else entirely. This is why this API exposes no
    ``open_gripper`` or ``close_gripper`` function: it has no way to know a mapping that
    is user configurable. Check how your gripper is configured before writing to its
    outputs.

Analog inputs without a measurement
-------------------------------------

An analog input does not always measure. An input that is connected to no measurement
chain reports nothing to read, and a reading is then unavailable rather than zero.
:class:`~pyniryo.nate.models.io.AnalogIO` carries that case explicitly: ``value`` is None,
and ``unavailable_reason`` says why.

.. code-block:: python
   :linenos:

    from pyniryo.nate import Nate
    from pyniryo.nate.models import AnalogUnavailableReason, IOLabel

    nate = Nate(hostname="10.10.10.10", login=("user", "secret"))

    tai1 = nate.io.get_all().analog_inputs[IOLabel.TAI1]

    if tai1.is_available:
        print(f"tai1 measures {tai1.value} V")
    elif tai1.unavailable_reason == AnalogUnavailableReason.HIGH_Z:
        print("tai1 is in high impedance, it is connected to no measurement chain")
    else:
        print(f"tai1 has no measurement: {tai1.unavailable_reason}")

The two reasons are:

.. list-table:: Reasons for an unavailable measurement
   :header-rows: 1
   :widths: auto
   :align: center

   *  -  Reason
      -  Meaning
   *  -  ``AnalogUnavailableReason.HIGH_Z``
      -  The input sits in high impedance, connected to no measurement chain. This is the
         mode an input holds until it is configured for voltage or current.
   *  -  ``AnalogUnavailableReason.RS485``
      -  The input is driven as an RS485 line. Analog acquisition and serial
         communication are mutually exclusive on it.

.. note::
    ``tai1`` and ``tai2`` share their lines with the RS485 bus of the tool board, which is
    what the second reason refers to. On this generation the RS485 mode is out of service
    and both inputs read a voltage.

.. danger::
    Never fall back to a default value when ``value`` is None. ``0.0`` is a legitimate
    measurement, so a program that reads ``io.value or 0.0`` cannot tell a grounded sensor
    from an unwired one. Test :attr:`~pyniryo.nate.models.io.AnalogIO.is_available`, or
    ``value is None``, before using the value.

The same holds for the values pushed to
:meth:`~pyniryo.nate.components.io.IO.on_analog_io`: the callback receives an
:class:`~pyniryo.nate.models.io.AnalogIO` whose ``value`` may be None.

An :class:`~pyniryo.nate.models.io.AnalogIO` that was read is not meant to be written back
as it is. :meth:`~pyniryo.nate.components.io.IO.update_states` raises ``ValueError`` on an
analog output whose ``value`` is None, rather than writing a zero to the board.
