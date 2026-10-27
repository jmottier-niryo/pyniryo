import unittest
from unittest.mock import patch, Mock
import requests
from pydantic import BaseModel

from pyniryo.nate.exceptions import ServerError, ClientError, InternalError, PyNiryoError, TransportError, TlsError
from pyniryo.nate._internal.http import HttpClient
from pyniryo.nate._internal.tls import DeviceIdentity, build_context

from .fixtures import tls as fixtures


def new_client(port: int = 8000,
               identity: DeviceIdentity | None = None,
               roots: str | None = None,
               **kwargs) -> HttpClient:
    context = build_context(roots or fixtures.CA.read_text(), identity or DeviceIdentity())
    kwargs.setdefault("http_timeout", 5.)
    return HttpClient("127.0.0.1", port, ssl_context=context, **kwargs)


class ResponseModel(BaseModel):
    message: str


class TestHttpClient(unittest.TestCase):

    def setUp(self) -> None:
        self.client = new_client()
        self.client.set_token('<token>')

    def mock_response(self, status_code: int, json_data=None, text=""):
        mock_response = Mock()
        mock_response.status_code = status_code
        mock_response.json.return_value = json_data
        mock_response.text = text
        return mock_response

    @patch("requests.Session.request", autospec=True)
    def test_get_request_success(self, mock_request):
        mock_request.return_value = self.mock_response(200, {"message": "Success"})

        response = self.client.get("/test", ResponseModel)
        self.assertEqual(response.message, "Success")

    @patch("requests.Session.request", autospec=True)
    def test_post_request_success(self, mock_request):
        mock_request.return_value = self.mock_response(201, {"message": "Created"})

        data = ResponseModel(message="Hello")
        response = self.client.post("/test", ResponseModel, data)
        self.assertEqual(response.message, "Created")

    @patch("requests.Session.request", autospec=True)
    def test_delete_request_success(self, mock_request):
        mock_request.return_value = self.mock_response(204, {"message": "Deleted"})

        response = self.client.delete("/test", ResponseModel)
        self.assertEqual(response.message, "Deleted")

    @patch("requests.Session.request", autospec=True)
    def test_server_error(self, mock_request):
        mock_request.return_value = self.mock_response(500, text="Internal Server Error")

        with self.assertRaises(ServerError):
            self.client.get("/test", ResponseModel)

    @patch("requests.Session.request", autospec=True)
    def test_client_error(self, mock_request):
        mock_request.return_value = self.mock_response(404, text="Not Found")

        with self.assertRaises(ClientError):
            self.client.get("/test", ResponseModel)

    @patch("requests.Session.request", autospec=True)
    def test_manual_mode_refusal_message(self, mock_request):
        body = '{"error": "direct control requires the robot to be in manual mode", "code": "manual_mode_required"}'
        mock_request.return_value = self.mock_response(409, text=body)

        with self.assertRaises(ClientError) as refusal:
            self.client.get("/test", ResponseModel)

        self.assertEqual(str(refusal.exception),
                         'API error with status code 409: '
                         'direct control requires the robot to be in manual mode')
        self.assertEqual(409, refusal.exception.status_code)
        self.assertIn('manual_mode_required', refusal.exception.response)

    @patch("requests.Session.request", autospec=True)
    def test_read_timeout_is_a_transport_error(self, mock_request):
        """A response that never comes surfaces as a PyNiryoError, so programs catching it can stop the arm."""
        mock_request.side_effect = requests.exceptions.ReadTimeout("Read timed out. (read timeout=5.0)")

        with self.assertRaises(TransportError) as failure:
            self.client.post("/api/robot/waypoints", ResponseModel, ResponseModel(message="go"))

        self.assertIsInstance(failure.exception, PyNiryoError)
        self.assertIsInstance(failure.exception.__cause__, requests.exceptions.ReadTimeout)
        self.assertEqual("POST", failure.exception.method)
        self.assertEqual("/api/robot/waypoints", failure.exception.path)
        self.assertIn("POST /api/robot/waypoints", str(failure.exception))
        self.assertIn("Read timed out", str(failure.exception))

    @patch("requests.Session.request", autospec=True)
    def test_connection_error_is_a_transport_error(self, mock_request):
        mock_request.side_effect = requests.exceptions.ConnectionError("Connection refused")

        with self.assertRaises(TransportError):
            self.client.get("/test", ResponseModel)

    @patch("requests.Session.request", autospec=True)
    def test_model_validation_error(self, mock_request):
        mock_request.return_value = self.mock_response(200, {"message": 123})

        with self.assertRaises(InternalError):
            self.client.get("/test", ResponseModel)

    @patch("requests.Session.request", autospec=True)
    def test_internal_error_on_invalid_json(self, mock_request):
        mock_request.return_value = self.mock_response(200, "Invalid JSON")

        with self.assertRaises(InternalError):
            self.client.get("/test", ResponseModel)


class TestApiErrorReason(unittest.TestCase):
    """Test the explanation an ApiError exposes from the body of an error response."""

    def test_reason_of_a_json_body(self):
        """Test that the explanation of a JSON body is read out of its envelope."""
        for body in ['{"error": "Cannot set input states"}', '{"message": "Cannot set input states"}']:
            with self.subTest(body=body):
                error = ClientError(400, body)
                self.assertEqual(error.reason, "Cannot set input states")
                self.assertNotIn('{', str(error))

    def test_reason_of_a_body_that_is_not_json(self):
        """Test that a body no envelope can be read from is served as it is."""
        error = ServerError(502, "<html>Bad Gateway</html>")
        self.assertEqual(error.reason, "<html>Bad Gateway</html>")

    def test_reason_of_a_json_body_that_is_not_an_object(self):
        """Test that a JSON body with no envelope to read from is served as it is."""
        error = ServerError(500, '"boom"')
        self.assertEqual(error.reason, '"boom"')

    def test_reason_of_a_json_body_with_no_known_key(self):
        """Test that a JSON body carrying no explanation is served as it is."""
        error = ClientError(400, '{"unexpected": "shape"}')
        self.assertEqual(error.reason, '{"unexpected": "shape"}')

    def test_status_code_is_kept_in_the_message(self):
        """Test that reading the explanation does not lose the status code."""
        error = ClientError(400, '{"error": "Cannot set input states"}')
        self.assertIn('400', str(error))


if __name__ == "__main__":
    unittest.main()


class OptionalModel(BaseModel):
    name: str
    description: str | None = None


class TestHttpClientWireFormat(unittest.TestCase):

    def setUp(self) -> None:
        self.client = new_client()

    def mock_response(self, status_code: int, json_data=None, text=""):
        mock_response = Mock()
        mock_response.status_code = status_code
        mock_response.json.return_value = json_data
        mock_response.text = text
        mock_response.content = text.encode()
        return mock_response

    @patch("requests.Session.request", autospec=True)
    def test_unset_optional_fields_are_omitted_from_json_body(self, mock_request):
        mock_request.return_value = self.mock_response(200, {"message": "ok"}, '{"message": "ok"}')

        self.client.post("/test", ResponseModel, OptionalModel(name="n"))

        self.assertEqual(mock_request.call_args.kwargs["json"], {"name": "n"})

    @patch("requests.Session.request", autospec=True)
    def test_excluded_fields_are_omitted_from_form_data(self, mock_request):
        mock_request.return_value = self.mock_response(200, {"message": "ok"}, '{"message": "ok"}')

        self.client.post("/test", ResponseModel, OptionalModel(name="n", description="d"),
                         files={"file": Mock()}, exclude={"description"})

        self.assertEqual(mock_request.call_args.kwargs["data"], {"name": "n"})
        self.assertIsNone(mock_request.call_args.kwargs["json"])

    @patch("requests.Session.request", autospec=True)
    def test_get_text_returns_the_body_with_the_accept_header(self, mock_request):
        response = self.mock_response(200, text="openapi: 3.0.3")
        mock_request.return_value = response

        text = self.client.get_text("/api/docs/openapi/spec", accept="application/yaml")

        self.assertEqual(text, "openapi: 3.0.3")
        self.assertEqual(mock_request.call_args.kwargs["headers"], {"Accept": "application/yaml"})

    @patch("requests.Session.request", autospec=True)
    def test_get_text_raises_on_error_status(self, mock_request):
        mock_request.return_value = self.mock_response(503, text='{"error": "down"}')

        with self.assertRaises(ServerError):
            self.client.get_text("/api/docs/openapi/spec", accept="application/yaml")

    @patch("requests.Session.get", autospec=True)
    def test_download_uses_the_http_timeout(self, mock_get):
        client = new_client(http_timeout=7.5)
        response = Mock()
        response.status_code = 200
        response.iter_content.return_value = [b"abc"]
        response.__enter__ = lambda s: s
        response.__exit__ = lambda s, *a: None
        mock_get.return_value = response
        from io import BytesIO
        dst = BytesIO()

        client.download("/file", dst)

        self.assertEqual(mock_get.call_args.kwargs["timeout"], 7.5)
        self.assertEqual(dst.getvalue(), b"abc")


class TestApiErrorCode(unittest.TestCase):

    def test_code_and_details_are_read_from_the_body(self):
        body = '{"error": "the robot is held by another workstation", "code": "robot_held", "details": {"client_id": "studio-1"}}'

        error = ClientError(409, body)

        self.assertEqual(error.code, "robot_held")
        self.assertEqual(error.details, {"client_id": "studio-1"})
        self.assertTrue(error.has_code("robot_held"))
        self.assertFalse(error.has_code("no_execution"))

    def test_code_is_none_when_the_body_carries_none(self):
        error = ClientError(400, '{"error": "malformed"}')

        self.assertIsNone(error.code)
        self.assertEqual(error.details, {})

    def test_code_is_none_when_the_body_is_not_json(self):
        error = ServerError(502, 'Bad Gateway')

        self.assertIsNone(error.code)
        self.assertEqual(error.details, {})


class TestHttpClientRetries(unittest.TestCase):

    def test_only_connection_failures_are_retried(self):
        client = new_client(n_retry=3)

        retry = client._session.get_adapter('https://localhost').max_retries

        self.assertEqual(retry.connect, 3)
        self.assertEqual(retry.total, 3)
        self.assertEqual(retry.read, 0)
        self.assertEqual(retry.status, 0)
        self.assertEqual(retry.other, 0)


class TestHttpClientTls(unittest.TestCase):

    def test_always_speaks_https(self):
        self.assertEqual(new_client()._url('/p'), 'https://127.0.0.1:8000/p')

    def test_talks_to_a_robot_signed_by_a_trusted_root_whatever_its_address(self):
        identity = DeviceIdentity()
        with fixtures.https_server(fixtures.ROBOT) as server:
            client = new_client(server.server_port, identity)

            response = client.get('/test', ResponseModel)

        self.assertEqual(response.message, 'ok')
        self.assertEqual(identity.expected, fixtures.DEVICE_ID)

    def test_refuses_a_robot_signed_by_another_root(self):
        with fixtures.https_server(fixtures.OTHER_CA_ROBOT) as server:
            client = new_client(server.server_port)

            with self.assertRaises(TlsError) as ctx:
                client.get('/test', ResponseModel)

        self.assertEqual(ctx.exception.method, 'GET')
        self.assertEqual(ctx.exception.path, '/test')
        self.assertIn('certificate verify failed', str(ctx.exception))

    def test_refuses_a_robot_without_device_identity(self):
        with fixtures.https_server(fixtures.NO_URN) as server:
            client = new_client(server.server_port)

            with self.assertRaises(TlsError) as ctx:
                client.get('/test', ResponseModel)

        self.assertIn('exactly one device', str(ctx.exception))
        self.assertNotIn('Max retries', str(ctx.exception))

    def test_refuses_another_device_once_bound(self):
        with fixtures.https_server(fixtures.OTHER_DEVICE) as server:
            client = new_client(server.server_port, DeviceIdentity(expected=fixtures.DEVICE_ID))

            with self.assertRaises(TlsError) as ctx:
                client.get('/test', ResponseModel)

        self.assertIn(fixtures.OTHER_DEVICE_ID, str(ctx.exception))

    def test_a_refused_certificate_is_not_retried(self):
        with fixtures.https_server(fixtures.OTHER_CA_ROBOT) as server:
            client = new_client(server.server_port, n_retry=3)
            with patch('time.sleep') as sleep:
                with self.assertRaises(TlsError):
                    client.get('/test', ResponseModel)

            self.assertEqual(server.connections, 1)
        sleep.assert_not_called()

    def test_download_and_get_text_report_tls_failures_the_same_way(self):
        from io import BytesIO
        with fixtures.https_server(fixtures.NO_URN) as server:
            client = new_client(server.server_port)

            with self.assertRaises(TlsError):
                client.get_text('/doc', accept='text/plain')
            with self.assertRaises(TlsError):
                client.download('/file', BytesIO())
