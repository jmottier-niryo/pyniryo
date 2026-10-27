import logging
import time
from abc import ABC, abstractmethod
from typing_extensions import Self

from .._internal.http import HttpClient
from .._internal.mqtt import MqttClient

logger = logging.getLogger(__name__)


class BaseAPIComponent(ABC):
    """
    Base class for all API components.
    """

    def __init__(self, http_client: HttpClient, mqtt_client: MqttClient, correlation_id: str) -> None:
        self._http_client: HttpClient = http_client
        self._mqtt_client = mqtt_client
        self._correlation_id = correlation_id
        self._closed = False

    def close(self) -> None:
        """
        Close the component and release any resources if needed.
        """
        if self._closed:
            return
        self._close()
        self._closed = True

    def _close(self) -> None:
        """
        Internal method to be overridden by subclasses for cleanup logic.
        """
        pass


class BaseCommand(ABC):

    def __init__(self, command_id: str, timeout: float) -> None:
        self._command_id = command_id
        self._timeout = timeout
        self._started_at = time.monotonic()

    @property
    def command_id(self) -> str:
        """
        Get the command ID of the command.

        :return: The command ID.
        """
        return self._command_id

    @abstractmethod
    def wait(self) -> None:
        """
        Wait for the command to complete. A timeout of zero or less waits indefinitely. The timeout counts from
        the moment the command was sent, not from the call to this method.
        """
        pass

    def __enter__(self) -> 'Self':
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):  # type: ignore
        self.wait()
