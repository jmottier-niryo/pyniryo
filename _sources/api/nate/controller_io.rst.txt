Controller inputs and outputs
=====================================

The controller carries twenty inputs and outputs, on top of the six of the tool connector
documented in :doc:`/api/nate/tool_io`. The IOs of the controller are prefixed with ``c``,
and the robot user manual documents their wiring on the connector: pin assignment, wire
colors, ground and supply.

.. list-table:: Controller IO labels
   :header-rows: 1
   :widths: auto
   :align: center

   *  -  Labels
      -  What they are
   *  -  ``cdi1`` to ``cdi8``
      -  Digital inputs, read only.
   *  -  ``cdo1`` to ``cdo8``
      -  Digital outputs.
   *  -  ``cai1``, ``cai2``
      -  Analog inputs, read only, in volts.
   *  -  ``cao1``, ``cao2``
      -  Analog outputs, in volts.

Reading and writing
-------------------------------------

:meth:`~pyniryo.nate.components.io.IO.get_all` returns the inputs and outputs of the
controller and of the tool at once, keyed by label:

.. code-block:: python
   :linenos:

    from pyniryo.nate import Nate
    from pyniryo.nate.models import IOLabel

    nate = Nate(hostname="10.10.10.10", login=("user", "secret"))

    states = nate.io.get_all()
    print(states.digital_inputs[IOLabel.CDI3].value)

    nate.io.set_digital_output(IOLabel.CDO1, True)

Outputs that must change together are written in one call, so that the robot never passes
through an intermediate combination:

.. code-block:: python
   :linenos:

    from pyniryo.nate.models import DigitalIO, IOLabel

    nate.io.update_states([
        DigitalIO(id=IOLabel.CDO1, value=True),
        DigitalIO(id=IOLabel.CDO2, value=False),
    ])

Analog outputs
-------------------------------------

``cao1`` and ``cao2`` are driven in volts by
:meth:`~pyniryo.nate.components.io.IO.set_analog_output`. The tool connector carries no
analog output, so these two are the only ones a program can write.

.. code-block:: python
   :linenos:

    nate.io.set_analog_output(IOLabel.CAO1, 4.5)

The range of a channel belongs to the robot, which refuses a write outside it with
:class:`~pyniryo.nate.exceptions.ClientError`. The message of the exception names the
channel, the value that was sent and the range the robot serves:

.. code-block:: python
   :linenos:

    from pyniryo.nate.exceptions import ClientError

    try:
        nate.io.set_analog_output(IOLabel.CAO1, 42.0)
    except ClientError as error:
        print(error)

.. warning::
    Reading an analog input gives back what is measured, out of range values included. The
    robot checks what a program writes, not what the world does to its inputs.

Waiting on an input
-------------------------------------

:meth:`~pyniryo.nate.components.io.IO.wait_for_digital_io` blocks until an input holds a
value. A program waiting on a sensor, a button or a handshake from a machine needs nothing
more:

.. code-block:: python
   :linenos:

    nate.io.wait_for_digital_io(IOLabel.CDI1, True, timeout=30)

The current state is read once the subscription is in place, so an input that already
holds the value returns at once rather than waiting for a change that will not come, and a
change happening meanwhile is not missed. A label the robot serves no digital IO under is
refused instead of waited on. Without a timeout the call waits forever; with one it raises
``TimeoutError``.

:meth:`~pyniryo.nate.components.io.IO.on_digital_io` and
:meth:`~pyniryo.nate.components.io.IO.on_analog_io` report a single IO as it changes,
without polling and without blocking:

.. code-block:: python
   :linenos:

    unsubscribe = nate.io.on_digital_io(IOLabel.CDI1, lambda io: print(io.value))

    # later, to stop receiving the changes
    unsubscribe()

Labels this client does not know
-------------------------------------

:class:`~pyniryo.nate.models.io.IOLabel` holds the labels that existed when this version
was released. A robot may serve more. Those are reported with their raw label, and the
rest of the answer is served as usual:

.. code-block:: python
   :linenos:

    states = nate.io.get_all()

    print(states.digital_inputs['cdi9'].value)
    nate.io.set_digital_output('cdo9', True)

An unknown label is sent as it is and the robot decides, so a program keeps working
against hardware released after the client that drives it. A label the robot does not
serve is refused with :class:`~pyniryo.nate.exceptions.ClientError`.

When the robot serves no IO
-------------------------------------

A read and a write both raise :class:`~pyniryo.nate.exceptions.ServerError` with a status
code of 503 while the chain that carries the inputs and outputs is not running. A robot
that is not ready is told apart from a robot that has no input and no output, which an
empty answer would hide.

.. code-block:: python
   :linenos:

    from pyniryo.nate.exceptions import ServerError

    try:
        states = nate.io.get_all()
    except ServerError as error:
        if error.status_code == 503:
            print(f"the robot is not serving its IOs yet: {error.reason}")
        else:
            raise
