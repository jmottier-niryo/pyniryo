import json
import socket
import ssl
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from contextlib import contextmanager
from pathlib import Path
from typing import Iterator

DIR = Path(__file__).parent

CA = DIR / 'ca.pem'
OTHER_CA = DIR / 'other-ca.pem'
ROBOT_KEY = DIR / 'robot-key.pem'
ROBOT = DIR / 'robot.pem'
OTHER_DEVICE = DIR / 'other-device.pem'
NO_URN = DIR / 'no-urn.pem'
TWO_URNS = DIR / 'two-urns.pem'
OTHER_CA_ROBOT = DIR / 'other-ca-robot.pem'

DEVICE_ID = 'test-device-123'
OTHER_DEVICE_ID = 'other-device-456'


def server_context(certfile: Path) -> ssl.SSLContext:
    context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    context.load_cert_chain(certfile, ROBOT_KEY)
    return context


@contextmanager
def raw_tls_server(certfile: Path, accepts: int = 1) -> Iterator[tuple[str, int]]:
    """
    Accepts ``accepts`` connections, runs the TLS handshake on each and closes them. A failed handshake is
    swallowed: the client side is what the tests observe.
    """
    context = server_context(certfile)
    listener = socket.socket()
    listener.bind(('127.0.0.1', 0))
    listener.listen(accepts)
    listener.settimeout(5)

    def serve() -> None:
        for _ in range(accepts):
            try:
                conn, _ = listener.accept()
            except OSError:
                return
            try:
                with context.wrap_socket(conn, server_side=True) as tls:
                    tls.recv(1)
            except (OSError, ssl.SSLError):
                pass

    thread = threading.Thread(target=serve, daemon=True)
    thread.start()
    try:
        yield listener.getsockname()
    finally:
        listener.close()
        thread.join(timeout=5)


class _JsonHandler(BaseHTTPRequestHandler):

    def do_GET(self) -> None:
        body = json.dumps({'message': 'ok', 'path': self.path}).encode()
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, *_args: object) -> None:
        pass


class _CountingServer(ThreadingHTTPServer):
    connections = 0

    def get_request(self):  # type: ignore[no-untyped-def]
        # the handshake runs inside accept(), count before it can fail
        self.connections += 1
        return super().get_request()

    def handle_error(self, request, client_address) -> None:  # type: ignore[no-untyped-def]
        pass


@contextmanager
def https_server(certfile: Path) -> Iterator[_CountingServer]:
    """
    Serves ``{"message": "ok"}`` on every GET over TLS with the given leaf. ``connections`` counts the TCP
    connections accepted, handshake failures included.
    """
    server = _CountingServer(('127.0.0.1', 0), _JsonHandler)
    server.socket = server_context(certfile).wrap_socket(server.socket, server_side=True)
    thread = threading.Thread(target=server.serve_forever, kwargs={'poll_interval': 0.05}, daemon=True)
    thread.start()
    try:
        yield server
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=5)
