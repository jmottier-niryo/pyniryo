import ssl
import threading
from importlib import resources
from typing import Any, Mapping

from ..exceptions import TlsError

DEVICE_URN_PREFIX = 'urn:niryo:device:'
_ROOTS_RESOURCE = 'roots.pem'


def _bundled_roots() -> str | None:
    """
    The Niryo roots shipped with the package, or None when the bundle holds no certificate.
    """
    resource = resources.files(__package__) / _ROOTS_RESOURCE
    if not resource.is_file():
        return None
    roots: str = resource.read_text()
    if 'BEGIN CERTIFICATE' not in roots:
        return None
    return roots


def load_roots(extra_ca_file: str | None = None) -> str:
    """
    The roots this client trusts, as PEM: the Niryo roots shipped with the package, plus the content of
    ``extra_ca_file`` when given.
    :raises TlsError: If the package was built without roots and no extra file is given, or the extra file
        cannot be read.
    """
    roots = _bundled_roots()
    extra = None
    if extra_ca_file is not None:
        try:
            with open(extra_ca_file) as f:
                extra = f.read()
        except OSError as e:
            raise TlsError(f'Unable to read the CA file {extra_ca_file}: {e}') from e
    if roots is None and extra is None:
        raise TlsError('This PyNiryo ships no Niryo root certificate: point NATE_CA_FILE at a PEM file holding '
                       'the root to trust.')
    return (roots or '') + (extra or '')


class _IdentitySocket(ssl.SSLSocket):
    """
    A socket whose handshake also checks the device named by the robot certificate, so that no application
    byte leaves before the robot is known to be the right one.
    """

    def do_handshake(self, block: bool = False) -> None:
        super().do_handshake(block)
        context = self.context
        assert isinstance(context, _IdentityContext)
        try:
            context.identity.check(self.getpeercert() or {})
        except TlsError:
            self.close()
            raise


class _IdentityContext(ssl.SSLContext):
    sslsocket_class = _IdentitySocket
    identity: 'DeviceIdentity'


def build_context(roots_pem: str, identity: 'DeviceIdentity') -> ssl.SSLContext:
    """
    A client context trusting only ``roots_pem``, whose handshakes check the robot against ``identity``.
    The host name is not checked: a robot is reached by an address that changes from one network to the next,
    its identity is the device URN in its certificate.
    """
    context = _IdentityContext(ssl.PROTOCOL_TLS_CLIENT)
    context.identity = identity
    context.minimum_version = ssl.TLSVersion.TLSv1_2
    context.check_hostname = False
    context.verify_mode = ssl.CERT_REQUIRED
    context.load_verify_locations(cadata=roots_pem)
    return context


class DeviceIdentity:
    """
    The device this client is bound to. The first certificate checked pins the device ID; every certificate
    checked afterwards must name the same device.
    """

    def __init__(self, expected: str | None = None) -> None:
        self._expected = expected
        self._lock = threading.Lock()

    @property
    def expected(self) -> str | None:
        return self._expected

    @staticmethod
    def device_id_of(peercert: Mapping[str, Any]) -> str:
        """
        :param peercert: A certificate as returned by :meth:`ssl.SSLSocket.getpeercert`.
        :return: The device ID carried by its ``urn:niryo:device:<id>`` URI SAN.
        :raises TlsError: If the certificate carries no such URN, or more than one.
        """
        urns = [
            value for kind, value in peercert.get('subjectAltName', ())
            if kind == 'URI' and value.startswith(DEVICE_URN_PREFIX)
        ]
        if len(urns) != 1:
            raise TlsError(f'The robot certificate must name exactly one device, it names {urns or "none"}.')
        device_id: str = urns[0][len(DEVICE_URN_PREFIX):]
        if not device_id:
            raise TlsError(f'The robot certificate names an empty device: {urns[0]}.')
        return device_id

    def bind(self, device_id: str) -> None:
        """
        Bind the client to a device, or check the device it is already bound to.
        :raises TlsError: If the client is bound to another device.
        """
        with self._lock:
            if self._expected is None:
                self._expected = device_id
            elif self._expected != device_id:
                raise TlsError(f'The robot certificate names device {device_id}, this client is bound to '
                               f'device {self._expected}.')

    def check(self, peercert: Mapping[str, Any]) -> str:
        """
        Check the certificate against the bound device, binding it when none is yet.
        :param peercert: A certificate as returned by :meth:`ssl.SSLSocket.getpeercert`.
        :return: The device ID of the certificate.
        :raises TlsError: If the certificate carries no device, or another device than the bound one.
        """
        device_id = self.device_id_of(peercert)
        self.bind(device_id)
        return device_id
