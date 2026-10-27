import socket
import ssl
import unittest
from unittest.mock import patch

from pyniryo.nate._internal.tls import DeviceIdentity, build_context, load_roots, _bundled_roots
from pyniryo.nate.exceptions import TlsError

from .fixtures import tls as fixtures


def peercert_with(*sans: tuple[str, str]) -> dict:
    return {
        'subject': ((('commonName', 'robot'), ), ), 'subjectAltName': sans
    }


class TestDeviceIdentity(unittest.TestCase):

    def test_first_check_pins_the_device_id_of_the_certificate(self):
        identity = DeviceIdentity()

        device_id = identity.check(peercert_with(('URI', 'urn:niryo:device:abc')))

        self.assertEqual(device_id, 'abc')
        self.assertEqual(identity.expected, 'abc')

    def test_same_device_passes_again(self):
        identity = DeviceIdentity()
        identity.check(peercert_with(('URI', 'urn:niryo:device:abc')))

        self.assertEqual(identity.check(peercert_with(('URI', 'urn:niryo:device:abc'))), 'abc')

    def test_another_device_is_refused_once_pinned(self):
        identity = DeviceIdentity(expected='abc')

        with self.assertRaises(TlsError) as ctx:
            identity.check(peercert_with(('URI', 'urn:niryo:device:xyz')))
        self.assertIn('xyz', str(ctx.exception))
        self.assertIn('abc', str(ctx.exception))

    def test_certificate_without_device_urn_is_refused(self):
        identity = DeviceIdentity()

        with self.assertRaises(TlsError):
            identity.check(peercert_with(('DNS', 'localhost'), ('IP Address', '127.0.0.1')))
        self.assertIsNone(identity.expected)

    def test_certificate_without_san_is_refused(self):
        with self.assertRaises(TlsError):
            DeviceIdentity().check({'subject': ()})

    def test_uri_of_another_scheme_is_not_a_device(self):
        with self.assertRaises(TlsError):
            DeviceIdentity().check(peercert_with(('URI', 'https://example.com/device/abc')))

    def test_empty_device_id_is_refused(self):
        with self.assertRaises(TlsError):
            DeviceIdentity().check(peercert_with(('URI', 'urn:niryo:device:')))

    def test_two_device_urns_are_refused(self):
        with self.assertRaises(TlsError):
            DeviceIdentity().check(peercert_with(('URI', 'urn:niryo:device:abc'), ('URI', 'urn:niryo:device:xyz')))


class TestBuildContext(unittest.TestCase):

    def handshake(self, context: ssl.SSLContext, certfile) -> dict:
        with fixtures.raw_tls_server(certfile) as (host, port):
            with socket.create_connection((host, port), timeout=5) as sock:
                with context.wrap_socket(sock, server_hostname=host) as tls:
                    return tls.getpeercert()

    def test_accepts_a_leaf_signed_by_a_given_root_whatever_the_host(self):
        context = build_context(fixtures.CA.read_text(), DeviceIdentity())

        peercert = self.handshake(context, fixtures.ROBOT)

        self.assertIn(('URI', 'urn:niryo:device:test-device-123'), peercert['subjectAltName'])

    def test_refuses_a_leaf_signed_by_another_root(self):
        context = build_context(fixtures.CA.read_text(), DeviceIdentity())

        with self.assertRaises(ssl.SSLCertVerificationError):
            self.handshake(context, fixtures.OTHER_CA_ROBOT)

    def test_trusts_every_root_of_the_bundle(self):
        context = build_context(fixtures.CA.read_text() + fixtures.OTHER_CA.read_text(), DeviceIdentity())

        self.handshake(context, fixtures.ROBOT)
        self.handshake(context, fixtures.OTHER_CA_ROBOT)

    def test_does_not_trust_the_system_roots(self):
        context = build_context(fixtures.OTHER_CA.read_text(), DeviceIdentity())

        self.assertEqual(context.cert_store_stats()['x509_ca'], 1)

    def test_requires_tls_1_2_at_least(self):
        context = build_context(fixtures.CA.read_text(), DeviceIdentity())

        self.assertEqual(context.minimum_version, ssl.TLSVersion.TLSv1_2)
        self.assertEqual(context.verify_mode, ssl.CERT_REQUIRED)
        self.assertFalse(context.check_hostname)

    def test_binds_the_device_of_the_first_handshake(self):
        identity = DeviceIdentity()
        context = build_context(fixtures.CA.read_text(), identity)

        self.handshake(context, fixtures.ROBOT)

        self.assertEqual(identity.expected, fixtures.DEVICE_ID)

    def test_refuses_a_leaf_without_device_urn(self):
        identity = DeviceIdentity()
        context = build_context(fixtures.CA.read_text(), identity)

        with self.assertRaises(TlsError):
            self.handshake(context, fixtures.NO_URN)
        self.assertIsNone(identity.expected)

    def test_refuses_another_device_once_bound(self):
        identity = DeviceIdentity()
        context = build_context(fixtures.CA.read_text(), identity)
        self.handshake(context, fixtures.ROBOT)

        with self.assertRaises(TlsError) as ctx:
            self.handshake(context, fixtures.OTHER_DEVICE)
        self.assertIn(fixtures.OTHER_DEVICE_ID, str(ctx.exception))

    def test_identity_error_is_a_certificate_error_for_the_libraries_above(self):
        self.assertIsInstance(TlsError('x'), ssl.CertificateError)


class TestLoadRoots(unittest.TestCase):

    def test_returns_the_bundled_roots(self):
        with patch('pyniryo.nate._internal.tls._bundled_roots', return_value='BUNDLED'):
            self.assertEqual(load_roots(), 'BUNDLED')

    def test_appends_the_extra_ca_file(self):
        with patch('pyniryo.nate._internal.tls._bundled_roots', return_value='BUNDLED\n'):
            roots = load_roots(str(fixtures.OTHER_CA))

        self.assertTrue(roots.startswith('BUNDLED\n'))
        self.assertIn(fixtures.OTHER_CA.read_text(), roots)

    def test_fails_when_the_package_was_built_without_roots(self):
        with patch('pyniryo.nate._internal.tls._bundled_roots', return_value=None):
            with self.assertRaises(TlsError) as ctx:
                load_roots()
        self.assertIn('NATE_CA_FILE', str(ctx.exception))

    def test_extra_ca_file_alone_is_enough(self):
        with patch('pyniryo.nate._internal.tls._bundled_roots', return_value=None):
            self.assertEqual(load_roots(str(fixtures.CA)), fixtures.CA.read_text())

    def test_missing_extra_ca_file_is_an_error(self):
        with patch('pyniryo.nate._internal.tls._bundled_roots', return_value='BUNDLED'):
            with self.assertRaises(TlsError):
                load_roots('/nonexistent/ca.pem')

    @unittest.skipUnless(_bundled_roots(), 'roots.pem holds no certificate yet')
    def test_reads_the_real_bundle_when_present(self):
        roots = load_roots()
        self.assertIn('BEGIN CERTIFICATE', roots)

    def test_a_bundle_without_certificate_counts_as_no_bundle(self):
        with patch('pyniryo.nate._internal.tls.resources') as resources:
            resource = resources.files.return_value.__truediv__.return_value
            resource.is_file.return_value = True
            resource.read_text.return_value = '# no root yet\n'
            self.assertIsNone(_bundled_roots())
