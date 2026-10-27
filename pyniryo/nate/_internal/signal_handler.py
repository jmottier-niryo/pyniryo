from __future__ import annotations

import logging
import os
import socket
from dataclasses import dataclass
from threading import Lock, Thread
from typing import TYPE_CHECKING, Any, Protocol

import orjson
from strenum import StrEnum

if TYPE_CHECKING:
    from ..client import Nate

logger = logging.getLogger(__name__)


class Action(StrEnum):
    UNKNOWN = 'unknown'

    PAUSE = 'p'
    RESUME = 'r'
    STOP = 's'

    ACK_PAUSE = 'ack_p'
    ACK_RESUME = 'ack_r'
    ACK_STOP = 'ack_s'


@dataclass
class Signal:
    action: Action

    @classmethod
    def from_dict(cls, d: dict[str, Any]) -> Signal:
        return cls(action=Action(d['action']))


class IPCProtocol(Protocol):

    def start(self) -> None:
        ...

    def stop(self) -> None:
        ...

    def read_signal(self) -> Signal | None:
        """
        Read the next signal from the IPC. The call is expected to be blocking until the next message is received.
        :return: The next signal from the IPC. If None, it means the connection is closed.
        """
        ...

    def write_signal(self, sig: Signal) -> None:
        ...


class IPCError(Exception):
    """Base class for all Inter-Process Communication errors."""
    pass


class IPCCommunicationError(IPCError):
    """Raised when a network or transport operation (read/write) fails."""
    pass


class IPCMessageError(IPCError):
    """Raised when a message fails to serialize or deserialize."""

    def __init__(self, message: str, msg: Any) -> None:
        super().__init__(message)
        self.msg = msg


class SocketIPC:

    def __init__(self, ipc_socket_address: str = '') -> None:
        self._address: Any
        if os.path.exists(ipc_socket_address):
            self._address = ipc_socket_address
            self._socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        elif ':' in ipc_socket_address:
            self._address = ipc_socket_address.split(':')
            if len(self._address) != 2:
                raise ValueError(f'Invalid ipc socket address "{ipc_socket_address}". '
                                 f'Value with ":" should be in format "host:port"')
            self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        else:
            raise ValueError(f'Invalid ipc socket address "{ipc_socket_address}"')

    def stop(self) -> None:
        self._socket.close()

    def start(self) -> None:
        try:
            self._socket.connect(self._address)
        except socket.error as e:
            raise IPCCommunicationError(f'Failed to connect to IPC socket at {self._address}') from e

    def read_signal(self) -> Signal | None:
        msg = self._socket.recv(1024)
        if msg == b'':
            return None
        try:
            return Signal.from_dict(orjson.loads(msg))
        except (KeyError, orjson.JSONDecodeError) as e:
            raise IPCMessageError(f'Failed to parse signal from {self._address}', msg=msg) from e

    def write_signal(self, sig: Signal) -> None:
        try:
            self._socket.send(orjson.dumps(sig.__dict__, option=orjson.OPT_APPEND_NEWLINE))
        except orjson.JSONEncodeError as e:
            raise IPCMessageError(f'Failed to serialize signal to JSON', msg=sig) from e
        except socket.error as e:
            raise IPCCommunicationError(f'Failed to send signal to {self._address}') from e


class SocketSignalHandler:
    _instance: SocketSignalHandler | None = None
    _instance_lock = Lock()

    def __init__(self, ipc: IPCProtocol) -> None:
        self._ipc = ipc
        self._nates: list[Nate] = []
        self._nates_lock = Lock()

    @classmethod
    def register(cls, nate: Nate, ipc_socket: str) -> SocketSignalHandler:
        with cls._instance_lock:
            if cls._instance is None:
                cls._instance = cls(SocketIPC(ipc_socket))
                cls._instance.start()
            cls._instance._add(nate)
            return cls._instance

    def unregister(self, nate: Nate) -> None:
        with self._instance_lock:
            with self._nates_lock:
                if nate in self._nates:
                    self._nates.remove(nate)
                if len(self._nates) == 0:
                    self._ipc.stop()
                    SocketSignalHandler._instance = None

    def _add(self, nate: Nate) -> None:
        with self._nates_lock:
            self._nates.append(nate)

    def _snapshot(self) -> list[Nate]:
        with self._nates_lock:
            return list(self._nates)

    def start(self) -> None:
        self._ipc.start()
        Thread(target=self._input_loop, daemon=True).start()

    def _input_loop(self) -> None:
        while sig := self._ipc.read_signal():
            if sig.action == Action.PAUSE:
                handler = self._on_pause_sig
                action = Action.ACK_PAUSE
            elif sig.action == Action.RESUME:
                handler = self._on_resume_sig
                action = Action.ACK_RESUME
            elif sig.action == Action.STOP:
                handler = self._on_stop_sig
                action = Action.ACK_STOP
            else:
                logger.error(f'Unknown signal: "{sig}"')
                continue

            for nate in self._snapshot():
                try:
                    handler(nate)
                except Exception:
                    logger.exception(f'Failed to handle signal with handler "{handler}"')

            try:
                self._ipc.write_signal(Signal(action=action))
            except Exception:
                logger.exception(f'Failed to send ACK "{action}"')

    @staticmethod
    def _is_command_in_progress(nate: Nate) -> bool:
        cmds = nate.robot.get_commands()
        if len(cmds) == 0:
            return False
        cmd = cmds[-1]
        return not cmd.state.is_final()

    def _on_pause_sig(self, nate: Nate) -> None:
        nate.robot._freeze_commands()
        if not self._is_command_in_progress(nate):
            return
        nate.robot.pause()

    def _on_resume_sig(self, nate: Nate) -> None:
        if self._is_command_in_progress(nate):
            nate.robot.resume()
        nate.robot._unfreeze_commands()

    def _on_stop_sig(self, nate: Nate) -> None:
        nate.robot._freeze_commands()
        if not self._is_command_in_progress(nate):
            return
        nate.robot.stop()
