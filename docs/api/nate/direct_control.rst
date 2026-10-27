Direct control
=====================================

The jog methods, the move methods, :meth:`~pyniryo.nate.components.Robot.execute_trajectory`
and :meth:`~pyniryo.nate.components.Robot.set_control_mode` are direct control: they move the
arm on demand instead of running a program. The robot accepts them only when its key selector
is on manual. On automatic the arm is left to the programs, and an operator who wants to move
it turns the key.

Refused on automatic
-------------------------------------

The robot answers 409 to a refused command, and the call raises
:class:`~pyniryo.nate.exceptions.ClientError`, whose message reads ``API error with status
code 409: direct control requires the robot to be in manual mode``.

.. code-block:: python
   :linenos:

    from pyniryo.nate import Nate
    from pyniryo.nate.exceptions import ClientError
    from pyniryo.nate.models import Joints

    nate = Nate(hostname="10.10.10.10", login=("user", "secret"))

    try:
        nate.robot.move_joints(Joints(0, 0, 0, 0, 0, 0)).wait()
    except ClientError as error:
        print(error)

The body carries ``"code": "manual_mode_required"`` next to that message, which
``error.code`` hands back for a caller that needs to tell this refusal from another 409, and
``error.response`` holds the body raw.

Held by a workstation
-------------------------------------

A Studio workstation takes a control session on the robot when it connects, and the robot then
refuses the driving commands of any other workstation with a 409 whose ``error.code`` reads
``robot_held``. ``error.details["client_id"]`` names the holder, the value
:meth:`~pyniryo.nate.components.Device.get_status` reports as ``client_id``.

pyniryo is not a workstation: it sends no ``Client-Id`` header and connects to the broker under
an id of its own shape, so it never takes the session and never holds the robot. Its commands
go through whatever the session says, which is to say they are refused while a workstation
holds the robot, and pass otherwise. A program the robot runs holds the arm the same way, with
``error.code`` reading ``robot_held_by_program``.

A command sent before the robot has received any safety state is refused the same way: it does
not assume a mode it has not read, whether that state is late or never comes. A program it runs
goes through all the same.

Moving from a program
-------------------------------------

A program the robot runs keeps moving the arm on automatic, where nothing else can. The moves,
:meth:`~pyniryo.nate.components.Robot.execute_trajectory` and
:meth:`~pyniryo.nate.components.Robot.set_control_mode` go through.

The robot recognizes the program by the ``Execution-Token`` header, which the client reads from
the ``NATE_EXECUTION_TOKEN`` environment variable. The program executor sets it, so a program
uploaded with :meth:`~pyniryo.nate.components.Programs.create` and started on the robot carries
the token without you writing anything. A script running on your own machine has no such
variable to read, so its moves are refused.

While a program is running, the robot checks the execution token before the mode, so an outside
script is turned away with a 401 whatever the key position. That 401 carries nothing to read:
its body is ``{"Channel":"","Err":{}}``, which ``error.reason`` hands back raw. The 409 above is
what an outside script gets when no program is running.

The jog is dropped without an answer
-------------------------------------

The jog methods publish over MQTT, where the robot sends nothing back. On automatic it drops the
command and the call returns as if it had been taken. Nothing is raised, nothing is logged on
your side, and the arm does not move.

A running program does not get through either: jogging is an operator watching the arm and
driving it step by step, which is what the automatic mode rules out.

.. warning::
    :meth:`~pyniryo.nate.components.Robot.jog_mode` only sets the control mode, so it is not
    the guard it looks like. From an outside script it raises on entry, like a move does, and a
    key turned to automatic during the block makes the exit raise too, leaving the robot in jog
    mode. From a program it opens without a word on automatic, and every jog inside the block is
    then dropped in silence.
