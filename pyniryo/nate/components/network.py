from typing import Callable

from .base_api_component import BaseAPIComponent
from .._internal import paths_gen, transport_models, topics_gen
from ..models import InterfaceState, InterfaceConfig, Unsubscribe

InterfaceStateCallback = Callable[[InterfaceState], None]


class Network(BaseAPIComponent):
    """
    Network component for inspecting and configuring the robot's network interfaces.
    """

    def get_all_interfaces(self) -> list[InterfaceState]:
        """
        Get the actual state of all network interfaces.

        :return: The list of all network interfaces' states.
        """
        resp = self._http_client.get(paths_gen.Network.GET_NETWORK_INTERFACES, transport_models.InterfaceStateList)
        return [InterfaceState.from_transport_model(i) for i in resp.root]

    def get_interface_state(self, interface_id: str) -> InterfaceState:
        """
        Get the actual state of a network interface.

        :param interface_id: The ID of the network interface.
        :return: The current state of the network interface.
        """
        resp = self._http_client.get(paths_gen.Network.GET_NETWORK_INTERFACE_STATE.format(interface_id=interface_id),
                                     transport_models.s.InterfaceState)
        return InterfaceState.from_transport_model(resp)

    def get_interface_config(self, interface_id: str) -> InterfaceConfig:
        """
        Get the configuration of a network interface.

        :param interface_id: The ID of the network interface.
        :return: The current configuration of the network interface.
        """
        resp = self._http_client.get(
            paths_gen.Network.GET_NETWORK_INTERFACE_CONFIG.format(interface_id=interface_id),
            transport_models.s.InterfaceConfig)
        return InterfaceConfig.from_transport_model(resp)

    def set_interface_config(self, interface_id: str, config: InterfaceConfig) -> InterfaceConfig:
        """
        Update the configuration of a network interface.

        :param interface_id: The ID of the network interface.
        :param config: The configuration to apply.
        :return: The updated configuration of the network interface.
        """
        resp = self._http_client.patch(
            paths_gen.Network.UPDATE_NETWORK_INTERFACE_CONFIG.format(interface_id=interface_id),
            transport_models.s.InterfaceConfig, config.to_transport_model())
        return InterfaceConfig.from_transport_model(resp)

    def reset(self, interfaces: list[str] | None = None) -> None:
        """
        Reset network settings to their default values.

        :param interfaces: The IDs of the network interfaces to reset. If None, reset all
            the interfaces.
        """
        self._http_client.post(paths_gen.Network.RESET_NETWORK_SETTINGS, transport_models.EmptyPayload,
                               transport_models.s.InterfaceList(interfaces=interfaces))

    def on_interface_state(self, interface_id: str, callback: InterfaceStateCallback) -> Unsubscribe:
        """
        Set a callback to be called when the state of a network interface changes.

        :param interface_id: The ID of the network interface to subscribe to.
        :param callback: The callback function that takes an InterfaceState parameter.
        """

        def internal_callback(_: str, state: transport_models.a.NetworkInterfaceState) -> None:
            callback(InterfaceState.from_a_transport_model(state))

        topic = self._mqtt_client.format(topics_gen.Network.INTERFACE_STATE, interface_id=interface_id)
        return self._mqtt_client.subscribe(topic, internal_callback, transport_models.a.NetworkInterfaceState)
