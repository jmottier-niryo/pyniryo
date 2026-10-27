from typing import Callable
from strenum import StrEnum

from .._internal import transport_models

Unsubscribe = Callable[[], None]


class ExecutorStatus(StrEnum):
    """
    Enumeration of executor statuses for trajectories and programs.
    """
    IDLE = 'idle'
    PENDING = 'pending'
    RUNNING = 'running'
    COMPLETED = 'completed'
    FAILED = 'failed'
    PAUSED = 'paused'
    STOPPED = 'stopped'
    SAFETY_STOPPED = 'safety_stopped'
    SAFETY_PAUSED = 'safety_paused'

    @classmethod
    def from_transport_model(cls, model: transport_models.s.ExecutorStatus) -> 'ExecutorStatus':
        return cls(model.value)

    def to_transport_model(self) -> transport_models.s.ExecutorStatus:
        return transport_models.s.ExecutorStatus(self.value)

    def is_error(self) -> bool:
        """
        Check if this status represents an error condition.

        :return: True if the status indicates failure, False otherwise.
        """
        return self == ExecutorStatus.FAILED

    def is_stopped(self) -> bool:
        """
        Check if this status means the execution was stopped before completing, by a client or by the safety chain.

        :return: True if the status is stopped or safety_stopped, False otherwise.
        """
        return self in (ExecutorStatus.STOPPED, ExecutorStatus.SAFETY_STOPPED)

    def is_final(self) -> bool:
        """
        Check if this status represents a final state, after which the executor reports nothing more about the
        execution.

        :return: True if the status is completed, failed, stopped or safety_stopped, False otherwise.
        """
        return self == ExecutorStatus.COMPLETED or self.is_error() or self.is_stopped()
