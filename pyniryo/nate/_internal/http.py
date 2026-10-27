import ssl
from typing import TypeVar, IO, cast, Any

from urllib3.util.retry import Retry

from pydantic import BaseModel, ValidationError
import requests
from requests.adapters import HTTPAdapter

from ..exceptions import get_msg_from_errors, ServerError, ClientError, InternalError, TransportError, TlsError

T = TypeVar("T", bound=BaseModel)


class _TlsAdapter(HTTPAdapter):
    """
    Hands the client's TLS context to urllib3. The host name is not checked at all, the context checks the
    device identity instead.
    """

    def __init__(self, ssl_context: ssl.SSLContext, max_retries: Retry) -> None:
        self._ssl_context = ssl_context
        super().__init__(max_retries=max_retries)

    def init_poolmanager(self, connections: int, maxsize: int, block: bool = False, **pool_kwargs: Any) -> None:
        pool_kwargs['ssl_context'] = self._ssl_context
        pool_kwargs['assert_hostname'] = False
        super().init_poolmanager(connections, maxsize, block, **pool_kwargs)

    def proxy_manager_for(self, proxy: str, **proxy_kwargs: Any) -> Any:
        proxy_kwargs['ssl_context'] = self._ssl_context
        proxy_kwargs['assert_hostname'] = False
        return super().proxy_manager_for(proxy, **proxy_kwargs)


def _innermost(error: BaseException) -> BaseException:
    """
    The exception at the bottom of the wrapping done by urllib3 and requests, where the certificate failure is.
    """
    while True:
        reason = getattr(error, 'reason', None)
        if isinstance(reason, BaseException):
            error = reason
        elif error.args and isinstance(error.args[0], BaseException):
            error = error.args[0]
        else:
            return error


def _transport_error(method: str, path: str, error: requests.RequestException) -> TransportError | TlsError:
    if isinstance(error, requests.exceptions.SSLError):
        return TlsError(_innermost(error), method, path)
    return TransportError(method, path, error)


class HttpClient:
    """
    A simple HTTP client wrapped around the requests library to suit the API behaviors.
    """

    def __init__(self,
                 hostname: str,
                 port: int,
                 ssl_context: ssl.SSLContext,
                 http_timeout: float = 0,
                 n_retry: int = 0,
                 retry_delay_factor: float = 1.) -> None:
        """
        Initialize the HTTP client.
        :param hostname: The hostname of the API.
        :param port: The port of the API.
        :param ssl_context: The TLS context every connection is made with, see :func:`..tls.build_context`.
        """
        if not hostname:
            raise ValueError(f'Invalid hostname "{hostname}"')
        if not (1 <= port <= 2**16 - 1):
            raise ValueError(f'Invalid port number "{port}"')

        self._hostname = hostname
        self._port = port
        self._scheme = 'https'
        self._http_timeout = http_timeout

        self._session = requests.Session()
        # only a connection that could not be made is retried: a request that timed out may have been applied,
        # and a certificate that was refused will be refused again
        retry = Retry(total=n_retry, connect=n_retry, read=0, status=0, other=0, backoff_factor=retry_delay_factor)
        self._session.mount(f'{self._scheme}://', _TlsAdapter(ssl_context, retry))

    def disconnect(self) -> None:
        """
        Disconnect the HTTP client and release any resources if needed.
        """
        self._session.close()

    def set_header(self, key: str, value: str) -> None:
        """
        Set a custom header.
        :param key: The key of the header.
        :param value: The value of the header.
        """
        self._session.headers[key] = value

    def set_token(self, token: str) -> None:
        """
        Set the authentication token.
        :param token: The token to set.
        """
        self.set_header('Authorization', f'Bearer {token}')

    @staticmethod
    def _resolve_status_code(response: requests.Response) -> None:
        """
        Resolve the status code of the response and raise an exception if needed.
        :param response: The response to resolve.
        """
        if response.status_code >= 500:
            raise ServerError.from_response(response)
        elif response.status_code >= 400:
            raise ClientError.from_response(response)

    def _url(self, path: str) -> str:
        """
        Generate the URL for the request.
        :param path: The path of the request.
        :return: The URL of the request.
        """
        return f"{self._scheme}://{self._hostname}:{self._port}{path}"

    def _request(self,
                 method: str,
                 path: str,
                 response_model: type[T],
                 json: BaseModel | None = None,
                 files: dict[str, IO[bytes]] | None = None,
                 optional: bool = False,
                 exclude: set[str] | None = None) -> T | None:
        """
        Make a request to the API.
        :param method: The method of the request.
        :param path: The path of the request.
        :param json: The data to send as serialized json with the request. If files is not None, this will be sent as form data.
        :param response_model: The model to use to parse the response.
        :param optional: If True, a response with no content (e.g. HTTP 204) is returned as None instead of being parsed.
        :param exclude: Fields of ``json`` to leave out of the request.
        :return: The response of the request.
        :rtype: response_model
        :raises TransportError: If no connection could be made or no response came within the timeout. A request that
            timed out may still have been applied by the robot.
        """
        if not issubclass(response_model, BaseModel):
            raise TypeError(f'Invalid type {response_model.__name__} for response model.')

        dict_json = None if json is None else json.model_dump(mode='json', exclude_none=True, exclude=exclude)
        try:
            response = self._session.request(method,
                                             self._url(path),
                                             json=dict_json if files is None else None,
                                             data=dict_json if files is not None else None,
                                             allow_redirects=True,
                                             files=files,
                                             timeout=self._http_timeout)
        except requests.RequestException as e:
            raise _transport_error(method, path, e) from e
        self._resolve_status_code(response)

        if optional and not response.content:
            return None

        try:
            response_data = response.json() if response.content else {}
            return response_model.model_validate(response_data)
        except requests.JSONDecodeError:
            raise InternalError(f'Failed to decode response as JSON: {response.text}')
        except ValidationError as e:
            raise InternalError(get_msg_from_errors(e.errors())) from e

    def get(self, path: str, response_model: type[T]) -> T:
        """
        Make a GET request to the API.
        :param path: The path of the request.
        :param response_model: The model to use to parse the response.
        :return: The response of the request.
        :rtype: response_model
        """
        return cast(T, self._request('GET', path, response_model))

    def get_optional(self, path: str, response_model: type[T]) -> T | None:
        """
        Make a GET request to the API, returning None if the server responds with no content.
        :param path: The path of the request.
        :param response_model: The model to use to parse the response.
        :return: The parsed response, or None if the response had no content.
        """
        return self._request('GET', path, response_model, optional=True)

    def post(self,
             path: str,
             response_model: type[T],
             data: BaseModel,
             files: dict[str, IO[bytes]] | None = None,
             exclude: set[str] | None = None) -> T:
        """
        Make a POST request to the API.
        :param path: The path of the request.
        :param response_model: The model to use to parse the response.
        :param data: The data to send with the request.
        :param files: Optional files to send with the request.
        :param exclude: Fields of ``data`` to leave out of the request.
        :return: The response of the request.
        :rtype: response_model
        """
        return cast(T, self._request('POST', path, response_model, data, files, exclude=exclude))

    def delete(self, path: str, response_model: type[T]) -> T:
        """
        Make a DELETE request to the API.
        :param path: The path of the request.
        :param response_model: The model to use to parse the response.
        :return: The response of the request.
        :rtype: response_model
        """
        return cast(T, self._request('DELETE', path, response_model))

    def patch(self,
              path: str,
              response_model: type[T],
              data: BaseModel,
              files: dict[str, IO[bytes]] | None = None,
              exclude: set[str] | None = None) -> T:
        """
        Make a PATCH request to the API.
        :param path: The path of the request.
        :param response_model: The model to use to parse the response.
        :param data: The data to send with the request.
        :param files: Optional files to send with the request.
        :param exclude: Fields of ``data`` to leave out of the request.
        :return: The response of the request.
        :rtype: response_model
        """
        return cast(T, self._request('PATCH', path, response_model, data, files, exclude=exclude))

    def put(self,
            path: str,
            response_model: type[T],
            data: BaseModel,
            files: dict[str, IO[bytes]] | None = None,
            exclude: set[str] | None = None) -> T:
        """
        Make a PUT request to the API.
        :param path: The path of the request.
        :param response_model: The model to use to parse the response.
        :param data: The data to send with the request.
        :param files: Optional files to send with the request.
        :param exclude: Fields of ``data`` to leave out of the request.
        :return: The response of the request.
        :rtype: response_model
        """
        return cast(T, self._request('PUT', path, response_model, data, files, exclude=exclude))

    def get_text(self, path: str, accept: str) -> str:
        """
        Make a GET request whose response is a text document rather than JSON.
        :param path: The path of the request.
        :param accept: The media type to ask for, e.g. ``text/html`` or ``application/yaml``.
        :return: The response body.
        """
        try:
            response = self._session.request('GET',
                                             self._url(path),
                                             headers={'Accept': accept},
                                             allow_redirects=True,
                                             timeout=self._http_timeout)
        except requests.RequestException as e:
            raise _transport_error('GET', path, e) from e
        self._resolve_status_code(response)
        return response.text

    def download(self, path: str, dst: IO[bytes]) -> None:
        """
        Download a file from the API.
        :param path: The path of the file to download.
        :param dst: A file-like object to write the content to.
        """
        try:
            with self._session.get(self._url(path), stream=True, timeout=self._http_timeout) as response:
                self._resolve_status_code(response)
                for chunk in response.iter_content(chunk_size=8192):
                    if chunk:  # filter out keep-alive chunks
                        dst.write(chunk)
        except requests.RequestException as e:
            raise _transport_error('GET', path, e) from e
