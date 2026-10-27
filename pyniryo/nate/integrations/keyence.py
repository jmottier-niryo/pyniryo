from enum import Enum
import socket

_DEFAULT_CAMERA_PORT = 8500
_DEFAULT_TIMEOUT = 5


class _Cmd(Enum):
    TRIGGER = 'T2'

    def ascii(self) -> bytes:
        return (self.value + '\r').encode('ascii')


class KeyenceError(Exception):
    pass


class UnknownCommandError(KeyenceError):
    pass


class UnprocessableCommandError(KeyenceError):
    pass


class InvalidCommandPayload(KeyenceError):
    pass


def _check_error(resp: list[str]) -> None:
    has_err = resp[0] == 'ER'
    if not has_err:
        return

    og_cmd = resp[1]
    code = resp[2]

    if code == '02':
        raise UnknownCommandError(f'Unknown command "{og_cmd}" sent to Keyence IV4')
    elif code == '03':
        raise UnprocessableCommandError(f'Command "{og_cmd}" cannot be processed currently')
    elif code == '22':
        raise InvalidCommandPayload(f'Invalid payload for command "{og_cmd}" sent to Keyence IV4')
    else:
        raise KeyenceError(f'Unknown error code: {code} (cmd {og_cmd})')


class IV4:

    def __init__(self, camera_hostname: str, camera_port: int = _DEFAULT_CAMERA_PORT, timeout: int = _DEFAULT_TIMEOUT):
        self._camera_hostname = camera_hostname
        self._camera_port = camera_port
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._socket.settimeout(timeout)
        self._socket.connect((self._camera_hostname, self._camera_port))

    def close(self) -> None:
        self._socket.close()

    def _send(self, cmd: _Cmd) -> list[str]:
        try:
            self._socket.sendall(cmd.ascii())
            resp = self._socket.recv(1024)
            return resp.decode('ascii').strip().split(',')
        except TimeoutError:
            raise TimeoutError('Timed out waiting for keyence iv4 response')
        except ConnectionRefusedError:
            raise ConnectionRefusedError(
                f'Connection to Keyence iv4 refused. ({self._camera_hostname}:{self._camera_port})')
        except Exception as e:
            raise RuntimeError(f'Unknown error while connecting to Keyence iv4. ({e})')

    def trigger(self) -> bool:
        resp = self._send(_Cmd.TRIGGER)
        _check_error(resp)

        if resp[0] != 'RT' or len(resp) < 3:
            raise ValueError(f'Unexpected trigger response: {resp}')

        if resp[2] == 'OK':
            return True
        elif resp[2] == 'NG':
            return False
        else:
            raise ValueError(f"Unexpected global state in response: {resp}")
