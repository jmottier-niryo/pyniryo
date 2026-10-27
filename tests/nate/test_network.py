import unittest
from unittest.mock import MagicMock

from pyniryo.nate._internal import transport_models, paths_gen, topics_gen
from pyniryo.nate.components.network import Network
from pyniryo.nate.models.network import (InterfaceState, InterfaceConfig, InterfaceType, InterfaceStatus, Ipv4State,
                                         Ipv4Config, EthernetState, WiFiState)

from .base import BaseTestComponent


def _transport_interface_state(**overrides):
    fields = dict(
        id='eth0',
        type=transport_models.s.InterfaceType.ETHERNET,
        mac_address='00:11:22:33:44:55',
        has_carrier=True,
        status=transport_models.s.InterfaceStatus.CONNECTED,
        ipv4=transport_models.s.Ipv4State(address='192.168.1.10',
                                          netmask='255.255.255.0',
                                          gateway='192.168.1.1',
                                          dns=['8.8.8.8']),
        ethernet=transport_models.s.EthernetState(speed=1000),
        wifi=None,
    )
    fields.update(overrides)
    return transport_models.s.InterfaceState(**fields)


class TestNetwork(BaseTestComponent):

    def setUp(self):
        super().setUp()
        self.network = Network(http_client=self.http_client,
                               mqtt_client=self.mqtt_client,
                               correlation_id=self.correlation_id)

    def tearDown(self):
        del self.network

    def test_get_all_interfaces(self):
        self.http_client.get.return_value = transport_models.InterfaceStateList(root=[_transport_interface_state()])

        interfaces = self.network.get_all_interfaces()

        self.http_client.get.assert_called_once_with(paths_gen.Network.GET_NETWORK_INTERFACES,
                                                     transport_models.InterfaceStateList)
        self.assertEqual(len(interfaces), 1)
        self.assertIsInstance(interfaces[0], InterfaceState)
        self.assertEqual(interfaces[0].id, 'eth0')
        self.assertEqual(interfaces[0].type, InterfaceType.ETHERNET)
        self.assertEqual(interfaces[0].status, InterfaceStatus.CONNECTED)
        self.assertIsInstance(interfaces[0].ipv4, Ipv4State)
        self.assertEqual(interfaces[0].ipv4.address, '192.168.1.10')
        self.assertIsInstance(interfaces[0].ethernet, EthernetState)
        self.assertIsNone(interfaces[0].wifi)

    def test_get_interface_state(self):
        self.http_client.get.return_value = _transport_interface_state(id='wlan0',
                                                                        type=transport_models.s.InterfaceType.WIFI,
                                                                        ethernet=None,
                                                                        wifi=transport_models.s.WiFiState(
                                                                            ssid='my-network',
                                                                            signal_strength=80))

        state = self.network.get_interface_state('wlan0')

        self.http_client.get.assert_called_once_with(
            paths_gen.Network.GET_NETWORK_INTERFACE_STATE.format(interface_id='wlan0'),
            transport_models.s.InterfaceState)
        self.assertEqual(state.id, 'wlan0')
        self.assertEqual(state.type, InterfaceType.WIFI)
        self.assertIsInstance(state.wifi, WiFiState)
        self.assertEqual(state.wifi.ssid, 'my-network')
        self.assertIsNone(state.ethernet)

    def test_get_interface_config(self):
        self.http_client.get.return_value = transport_models.s.InterfaceConfig(
            enabled=True,
            ipv4=transport_models.s.Ipv4Config(ip='10.0.0.5', netmask='255.255.255.0', dhcp=False),
            ethernet=transport_models.s.EthernetConfig(link_local=True),
            wifi=None,
        )

        config = self.network.get_interface_config('eth0')

        self.http_client.get.assert_called_once_with(
            paths_gen.Network.GET_NETWORK_INTERFACE_CONFIG.format(interface_id='eth0'),
            transport_models.s.InterfaceConfig)
        self.assertIsInstance(config, InterfaceConfig)
        self.assertTrue(config.enabled)
        self.assertIsInstance(config.ipv4, Ipv4Config)
        self.assertEqual(config.ipv4.ip, '10.0.0.5')
        self.assertFalse(config.ipv4.dhcp)
        self.assertTrue(config.ethernet.link_local)

    def test_set_interface_config(self):
        updated = transport_models.s.InterfaceConfig(
            enabled=True,
            ipv4=transport_models.s.Ipv4Config(ip='10.0.0.6', netmask='255.255.255.0', dhcp=False),
            ethernet=None,
            wifi=None,
        )
        self.http_client.patch.return_value = updated
        config = InterfaceConfig(enabled=True, ipv4=Ipv4Config(ip='10.0.0.6', netmask='255.255.255.0', dhcp=False))

        result = self.network.set_interface_config('eth0', config)

        self.http_client.patch.assert_called_once()
        call_args = self.http_client.patch.call_args[0]
        self.assertEqual(call_args[0], paths_gen.Network.UPDATE_NETWORK_INTERFACE_CONFIG.format(interface_id='eth0'))
        self.assertEqual(call_args[1], transport_models.s.InterfaceConfig)
        self.assertEqual(call_args[2], config.to_transport_model())
        self.assertIsInstance(result, InterfaceConfig)
        self.assertEqual(result.ipv4.ip, '10.0.0.6')

    def test_reset_all(self):
        self.http_client.post.return_value = transport_models.EmptyPayload()

        self.network.reset()

        self.http_client.post.assert_called_once_with(paths_gen.Network.RESET_NETWORK_SETTINGS,
                                                      transport_models.EmptyPayload,
                                                      transport_models.s.InterfaceList(interfaces=None))

    def test_reset_specific_interfaces(self):
        self.http_client.post.return_value = transport_models.EmptyPayload()

        self.network.reset(['eth0', 'wlan0'])

        self.http_client.post.assert_called_once_with(
            paths_gen.Network.RESET_NETWORK_SETTINGS, transport_models.EmptyPayload,
            transport_models.s.InterfaceList(interfaces=['eth0', 'wlan0']))

    def test_on_interface_state_subscription(self):
        callback = MagicMock()

        self.network.on_interface_state('eth0', callback)

        self.mqtt_client.subscribe.assert_called_once()
        call_args = self.mqtt_client.subscribe.call_args[0]
        expected_topic = self.mqtt_client.format(topics_gen.Network.INTERFACE_STATE, interface_id='eth0')
        self.assertEqual(call_args[0], expected_topic)
        self.assertEqual(call_args[2], transport_models.a.NetworkInterfaceState)

    def test_on_interface_state_callback_execution(self):
        callback = MagicMock()

        self.network.on_interface_state('eth0', callback)
        internal_callback = self.mqtt_client.subscribe.call_args[0][1]

        internal_callback(
            None,
            transport_models.a.NetworkInterfaceState(
                id='eth0',
                type=transport_models.a.NetworkInterfaceStateType.ETHERNET,
                mac_address='00:11:22:33:44:55',
                has_carrier=True,
                status=transport_models.a.NetworkInterfaceStateStatus.CONNECTED,
                ipv4={
                    'address': '192.168.1.10',
                    'netmask': '255.255.255.0',
                    'gateway': '192.168.1.1',
                    'dns': None,
                },
                ethernet={'speed': 1000},
                wifi=None,
            ))

        callback.assert_called_once()
        state = callback.call_args[0][0]
        self.assertIsInstance(state, InterfaceState)
        self.assertEqual(state.id, 'eth0')
        self.assertEqual(state.ipv4.address, '192.168.1.10')
        self.assertEqual(state.ethernet.speed, 1000)


if __name__ == "__main__":
    unittest.main()
