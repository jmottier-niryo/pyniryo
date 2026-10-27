pyniryo
=========

.. contents:: **Table of Contents**
    :backlinks: none

Installation
------------

PyNiryo is distributed on `PyPI <https://pypi.org>`_ as a universal
wheel and is available on Linux/macOS and Windows and supports
Python 3.10+

.. code-block:: bash

    $ pip install pyniryo

Documentation
-----------------

PyNiryo allows to write simple script in Python in order to control Niryo's robots

.. code-block:: python

    from pyniryo import *

    robot = NiryoRobot("10.10.10.10")

    robot.calibrate_auto()

    robot.move(JointsPosition(0.2, -0.3, 0.1, 0.0, 0.5, -0.8))

To see more examples or learn more about the available functions,
full documentation is available `here <https://niryorobotics.github.io/pyniryo>`_.

Nate Robot API
--------------

PyNiryo also provides a client for controlling Niryo's Nate robots through their API.

.. code-block:: python

    from pyniryo.nate.client import Nate
    from pyniryo.nate.models import Joints

    # Initialize with hostname and credentials
    nate = Nate(hostname="10.10.10.10", login=("user", "secret"))

    # Or use environment variables
    nate = Nate()

    # Move the robot. The key selector must be on manual, see the Direct control page of
    # the documentation.
    nate.robot.move(Joints(0, 0, 0, 0, 0, 0)).wait()

Constructor Arguments
~~~~~~~~~~~~~~~~~~~~~

The ``Nate`` class constructor accepts the following parameters:

- ``hostname`` (str | None): The hostname or IP address of the Nate API. If ``None``, it will use the ``NATE_HOSTNAME`` environment variable, defaulting to ``'localhost'`` if not set.
- ``token`` (str | None): Authentication token for API access. If ``None``, it will use the ``NATE_TOKEN`` environment variable. If a token is not provided, username/password authentication will be used instead.
- ``login`` (tuple[str, str] | None): A tuple containing ``(username, password)`` for authentication. If ``None``, it will use the ``NATE_USERNAME`` and ``NATE_PASSWORD`` environment variables. This parameter is omitted when using token authentication.

Environment Variables
~~~~~~~~~~~~~~~~~~~~~

The following environment variables can be used to configure the Nate client:

**Authentication:**

- ``NATE_HOSTNAME`` - The hostname or IP address of the Nate API (default: ``'localhost'``)
- ``NATE_TOKEN`` - Authentication token for API access
- ``NATE_USERNAME`` - Username for login authentication (used when token is not provided)
- ``NATE_PASSWORD`` - Password for login authentication (used when token is not provided)

**Advanced Options:**

- ``NATE_HTTP_PORT`` - HTTPS port of the API (default: ``8443``)
- ``NATE_MQTT_PORT`` - MQTTS port of the broker (default: ``8883``)
- ``NATE_CA_FILE`` - A PEM file holding extra root certificates to trust, on top of the production root shipped with the package
- ``NATE_DEVICE_ID`` - The identifier of the expected robot; another robot at that address is refused before any credential is sent

Connection security
~~~~~~~~~~~~~~~~~~~

Every connection to the robot is encrypted, over HTTPS and MQTTS. The robot presents a certificate signed by
the Niryo certificate authority, whose production root ships with the package: no setup is needed to trust a
Niryo robot. A robot whose certificate is signed by anyone else is refused with a
``pyniryo.nate.exceptions.TlsError``.

The host name is not checked, as a robot is reached by an IP address or an mDNS name that change from one
network to the next. The robot identity is the device ID written in its certificate instead. Pass the
expected one as ``Nate(hostname, login, device_id=...)`` or ``NATE_DEVICE_ID`` and any other robot at that
address is refused before a credential is sent. Without it, the first handshake binds the client to the
device it reaches. Either way every later connection, MQTT included, must present the same device, and the
device ID reported by the API must match it too.

The package ships the Niryo production root only. A robot signed by another Niryo root is trusted by
pointing ``NATE_CA_FILE`` at a PEM file holding that root.
The programs run on a robot need nothing: the robot hands them the chain of its own certificate.

Generated code
--------------

The transport models, the API paths and the MQTT topics of the Nate client are generated
from the OpenAPI and AsyncAPI specifications published by the robot API. Point ``SPECS_DIR``
at a directory holding ``openapi.yaml`` and ``asyncapi.yaml`` to regenerate them:

.. code-block:: bash

    $ make gen SPECS_DIR=/path/to/specs

This writes ``pyniryo/nate/_internal/paths_gen.py``, ``pyniryo/nate/_internal/topics_gen.py``,
``pyniryo/nate/_internal/transport_models/models_gen.py`` and
``pyniryo/nate/_internal/transport_models/async_models_gen.py``, which are never edited by hand.

    .. note::

    If you need the documentation for an older version, you can build it locally by
    checking out the corresponding git tag and running:

    .. code-block:: bash

        pip install -r docs/requirements.txt
        sphinx-build docs/ _build/html


License
-------

PyNiryo is distributed under the terms of
`GNU General Public License v3.0 <https://choosealicense.com/licenses/gpl-3.0>`_
