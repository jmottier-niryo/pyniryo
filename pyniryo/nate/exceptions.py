import json
import ssl
from typing import Any

import requests
from pydantic_core import ErrorDetails


class PyNiryoError(Exception):
    """
    Base exception class for all PyNiryo errors.
    """
    pass


class ApiError(PyNiryoError):
    """
    Exception raised when an API request fails.

    :param status_code: The HTTP status code of the response.
    :param response: The response body text.
    :param message: An optional custom error message.

    :ivar reason: The explanation the server gave, or the raw body when it gave none.
    :ivar code: The machine readable reason the server gave, e.g. ``robot_held`` or ``no_execution``, None when
        it gave none. A client that does not know the code it receives falls back to ``reason``.
    :ivar details: What the refusal is about, keyed by name. Which keys are present depends on ``code``.
    """
    status_code: int
    response: str
    reason: str
    code: str | None
    details: dict[str, Any]

    def __init__(self, status_code: int, response: str, message: str | None = None) -> None:
        self.status_code = status_code
        self.response = response
        body = _read_body(response)
        self.reason = _read_reason(response, body)
        self.code = body.get('code') if isinstance(body.get('code'), str) else None
        details = body.get('details')
        self.details = details if isinstance(details, dict) else {}
        if message is None:
            message = f"API error with status code {status_code}: {self.reason}"
        super().__init__(message)

    @classmethod
    def from_response(cls, response: requests.Response, *args: Any) -> "ApiError":
        """
        Create an ApiError from a requests Response object.
        
        :param response: The HTTP response object.
        :return: An ApiError instance.
        """
        return cls(response.status_code, response.text, *args)

    def has_code(self, code: str) -> bool:
        """
        :param code: A machine readable reason, as documented by the API.
        :return: Whether the server refused the request for this very reason.
        """
        return self.code == code


class ClientError(ApiError):
    """
    Exception raised when a client error (4xx status code) occurs.
    """
    pass


class ServerError(ApiError):
    """
    Exception raised when a server error (5xx status code) occurs.
    """
    pass


class TransportError(PyNiryoError):
    """
    Exception raised when an HTTP request could not complete: no connection, or no response in time.

    A request that timed out may still have been applied by the robot, a move for instance is
    accepted before its response is written.

    :param method: The HTTP method of the request.
    :param path: The path of the request.
    :param reason: What went wrong, as the transport reported it.
    """
    method: str
    path: str

    def __init__(self, method: str, path: str, reason: str | Exception) -> None:
        self.method = method
        self.path = path
        super().__init__(f"{method} {path} failed: {reason}")


class TlsError(PyNiryoError, ssl.CertificateError):
    """
    Exception raised when the robot could not be trusted: its certificate is not signed by a Niryo root, carries
    no device identity, or names another device than the one this client is bound to.

    :param reason: What went wrong.
    :param method: The HTTP method of the request that failed, when raised from an HTTP call.
    :param path: The path of the request that failed, when raised from an HTTP call.

    It is an :class:`ssl.CertificateError` too, so the transport libraries treat it as the certificate
    failure it is.
    """
    method: str | None
    path: str | None

    def __init__(self, reason: str | BaseException, method: str | None = None, path: str | None = None) -> None:
        self.method = method
        self.path = path
        prefix = f"{method} {path} failed: " if method and path else ""
        super().__init__(f"{prefix}{reason}")

    def __str__(self) -> str:
        # ssl.SSLError renders its args as a tuple when no strerror is set
        return str(self.args[0])


class DataValidationError(PyNiryoError):
    """
    Exception raised when data validation fails.
    """
    pass


class InternalError(PyNiryoError):
    """
    Exception raised when an internal error occurs.
    """
    pass


class ExecutionStoppedError(PyNiryoError):
    """
    Exception raised when a program execution was stopped before completing, by a client or by the safety chain.
    """
    pass


class PrepareTrajectoryError(PyNiryoError):
    """
    Exception raised when the robot fails to prepare a move, before any trajectory is generated: the request
    could not be turned into a motion problem, for instance a frame or a waypoint the robot cannot resolve.
    """
    pass


class GenerateTrajectoryError(PyNiryoError):
    """
    Exception raised when an error occurs during trajectory generation.
    
    This exception is raised when the motion planner fails to generate a valid
    trajectory from the provided waypoints, typically due to kinematic constraints,
    unreachable positions, or invalid planner configurations.
    """
    pass


class LoadTrajectoryError(PyNiryoError):
    """
    Exception raised when an error occurs during trajectory loading.
    
    This exception is raised when the robot controller fails to load a generated
    trajectory, usually due to invalid trajectory data or controller state issues.
    """
    pass


class ExecuteTrajectoryError(PyNiryoError):
    """
    Exception raised when an error occurs during trajectory execution.
    
    This exception is raised when the robot fails to execute a loaded trajectory,
    which may occur due to hardware faults, safety limits being exceeded, or
    external interruptions during movement.
    """
    pass


def get_msg_from_errors(errors: list[ErrorDetails]) -> str:
    """
    Get the error message from pydantic errors list.
    :param errors: The pydantic errors list.
    :return: The error message.
    """
    error_msgs = []
    for error in errors:
        if len(error['loc']) == 0:
            error_msgs.append(error['msg'])
        else:
            error_msgs.append(f'Error for field {error["loc"][0]}={repr(error["input"])}: {error["msg"]}')
    return ', '.join(error_msgs)


_REASON_KEYS = ('error', 'message', 'detail')


def _read_body(response: str) -> dict[str, Any]:
    """
    Parse an error body as the API error object, or as an empty object when it is not one: a proxy, a gateway or a
    crash answers something that is neither JSON nor the shape of an API error.

    :param response: The response body text.
    :return: The parsed object, or an empty one.
    """
    try:
        body = json.loads(response)
    except ValueError:
        return {}
    return body if isinstance(body, dict) else {}


def _read_reason(response: str, body: dict[str, Any]) -> str:
    """
    Extract the explanation the server gave from an error body, or the raw body when it carries none.

    :param response: The response body text.
    :param body: The body parsed by :func:`_read_body`.
    :return: The explanation, or the body itself.
    """
    return next((body[key] for key in _REASON_KEYS if isinstance(body.get(key), str) and body[key]), response)
