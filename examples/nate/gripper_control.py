import time

from pyniryo.nate import Nate
from pyniryo.nate.models import DigitalIO, IOLabel

# The states to write for a given behavior depend on how the gripper is configured: check
# how your gripper is wired and configured before reusing these values, see
# /api/nate/tool_io. tdo1 and tdo2 are always written together in a single update_states
# call: writing them one after the other would pass through an intermediate combined state
# that may not correspond to any behavior the gripper was configured for.
GRIPPER_OPEN = [
    DigitalIO(id=IOLabel.TDO1, value=False),
    DigitalIO(id=IOLabel.TDO2, value=False),
]
GRIPPER_CLOSE = [
    DigitalIO(id=IOLabel.TDO1, value=True),
    DigitalIO(id=IOLabel.TDO2, value=True),
]


def main():
    n = Nate()

    print('Example 1: Open and close, waiting a fixed delay for the gripper to move')
    """
    Without feedback from the gripper, the only way to know it has finished moving is to
    wait long enough. This is simple but fragile: too short a delay risks acting before the
    gripper is done, too long wastes time.
    """

    n.io.update_states(GRIPPER_OPEN)
    time.sleep(1)

    n.io.update_states(GRIPPER_CLOSE)
    time.sleep(1)

    n.io.update_states(GRIPPER_OPEN)
    time.sleep(1)


if __name__ == '__main__':
    main()
