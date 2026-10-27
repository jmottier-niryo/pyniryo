from dataclasses import dataclass
from typing import Any

from strenum import StrEnum

from .._internal import transport_models

_TransportInterfaceType = transport_models.s.InterfaceType | transport_models.a.NetworkInterfaceStateType
_TransportInterfaceStatus = transport_models.s.InterfaceStatus | transport_models.a.NetworkInterfaceStateStatus


class InterfaceType(StrEnum):
    """
    Type of a network interface.
    """
    ETHERNET = 'ethernet'
    WIFI = 'wifi'

    @classmethod
    def from_transport_model(cls, model: _TransportInterfaceType) -> 'InterfaceType':
        return cls(model.value)


class InterfaceStatus(StrEnum):
    """
    Status of a network interface.

    - DISABLED: The network interface is disabled and not operational.
    - ENABLED: The network interface is in the process of being connected or is not yet
      fully operational.
    - CONNECTED: The network interface is connected to a network and operational.
    """
    DISABLED = 'disabled'
    ENABLED = 'enabled'
    CONNECTED = 'connected'

    @classmethod
    def from_transport_model(cls, model: _TransportInterfaceStatus) -> 'InterfaceStatus':
        return cls(model.value)


@dataclass
class Ipv4State:
    """
    :param address: Static IP address assigned to the device.
    :param netmask: Subnet mask for the network.
    :param gateway: Default gateway IP address.
    :param dns: List of DNS server IP addresses.
    """
    address: str
    netmask: str
    gateway: str
    dns: list[str] | None

    @classmethod
    def from_transport_model(cls, model: transport_models.s.Ipv4State) -> 'Ipv4State':
        return cls(
            address=str(model.address),
            netmask=str(model.netmask),
            gateway=str(model.gateway),
            dns=[str(d) for d in model.dns] if model.dns is not None else None,
        )

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> 'Ipv4State':
        return cls(
            address=data['address'],
            netmask=data['netmask'],
            gateway=data['gateway'],
            dns=data.get('dns'),
        )


@dataclass
class EthernetState:
    """
    :param speed: Speed in Mbps.
    """
    speed: int

    @classmethod
    def from_transport_model(cls, model: transport_models.s.EthernetState) -> 'EthernetState':
        return cls(speed=model.speed)

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> 'EthernetState':
        return cls(speed=data['speed'])


@dataclass
class WiFiState:
    """
    :param ssid: The network name the interface is connected to.
    :param signal_strength: Signal strength in percentage (0-100).
    """
    ssid: str
    signal_strength: int

    @classmethod
    def from_transport_model(cls, model: transport_models.s.WiFiState) -> 'WiFiState':
        return cls(ssid=model.ssid, signal_strength=model.signal_strength)

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> 'WiFiState':
        return cls(ssid=data['ssid'], signal_strength=data['signal_strength'])


@dataclass
class InterfaceState:
    """
    Actual, live state of a network interface.

    :param id: Unique identifier for the network interface.
    :param type: Type of the network interface.
    :param mac_address: MAC address of the network interface.
    :param has_carrier: Whether the network interface has a carrier (e.g. for an ethernet
        interface, a cable is plugged-in).
    :param status: Status of the network interface.
    :param ipv4: IPv4 state of the network interface.
    :param ethernet: Ethernet-specific state, set only if the interface is an ethernet one.
    :param wifi: WiFi-specific state, set only if the interface is a WiFi one.
    """
    id: str
    type: InterfaceType
    mac_address: str
    has_carrier: bool
    status: InterfaceStatus
    ipv4: Ipv4State
    ethernet: EthernetState | None
    wifi: WiFiState | None

    @classmethod
    def from_transport_model(cls, model: transport_models.s.InterfaceState) -> 'InterfaceState':
        return cls(
            id=model.id,
            type=InterfaceType.from_transport_model(model.type),
            mac_address=model.mac_address,
            has_carrier=model.has_carrier,
            status=InterfaceStatus.from_transport_model(model.status),
            ipv4=Ipv4State.from_transport_model(model.ipv4),
            ethernet=EthernetState.from_transport_model(model.ethernet) if model.ethernet else None,
            wifi=WiFiState.from_transport_model(model.wifi) if model.wifi else None,
        )

    @classmethod
    def from_a_transport_model(cls, model: transport_models.a.NetworkInterfaceState) -> 'InterfaceState':
        return cls(
            id=model.id,
            type=InterfaceType.from_transport_model(model.type),
            mac_address=model.mac_address,
            has_carrier=model.has_carrier,
            status=InterfaceStatus.from_transport_model(model.status),
            ipv4=Ipv4State.from_dict(model.ipv4),
            ethernet=EthernetState.from_dict(model.ethernet) if model.ethernet else None,
            wifi=WiFiState.from_dict(model.wifi) if model.wifi else None,
        )


@dataclass
class Ipv4Config:
    """
    :param ip: Static IP address to assign to the device.
    :param netmask: Subnet mask for the network.
    :param gateway: Default gateway IP address.
    :param dns: List of DNS server IP addresses.
    :param dhcp: Whether DHCP is enabled. If True, ``ip``, ``netmask``, ``gateway`` and
        ``dns`` may be ignored.
    """
    ip: str
    netmask: str
    dhcp: bool
    gateway: str | None = None
    dns: list[str] | None = None

    @classmethod
    def from_transport_model(cls, model: transport_models.s.Ipv4Config) -> 'Ipv4Config':
        return cls(
            ip=str(model.ip),
            netmask=str(model.netmask),
            gateway=str(model.gateway) if model.gateway is not None else None,
            dns=[str(d) for d in model.dns] if model.dns is not None else None,
            dhcp=model.dhcp,
        )

    def to_transport_model(self) -> transport_models.s.Ipv4Config:
        return transport_models.s.Ipv4Config(
            ip=self.ip,
            netmask=self.netmask,
            gateway=self.gateway,
            dns=self.dns,
            dhcp=self.dhcp,
        )


@dataclass
class EthernetConfig:
    """
    :param link_local: Whether link-local addressing (169.254.x.x) is enabled as a fallback
        for the ethernet interface.
    """
    link_local: bool

    @classmethod
    def from_transport_model(cls, model: transport_models.s.EthernetConfig) -> 'EthernetConfig':
        return cls(link_local=model.link_local)

    def to_transport_model(self) -> transport_models.s.EthernetConfig:
        return transport_models.s.EthernetConfig(link_local=self.link_local)


@dataclass
class WpaPersonal:
    """
    WPA2/WPA3 personal network credentials.

    :param psk: The network passphrase/PSK.
    """
    psk: str

    @classmethod
    def from_transport_model(cls, model: transport_models.s.WpaPersonal) -> 'WpaPersonal':
        return cls(psk=model.psk)

    def to_transport_model(self) -> transport_models.s.WpaPersonal:
        return transport_models.s.WpaPersonal(psk=self.psk)


@dataclass
class WpaEnterprise:
    """
    802.1X / EAP enterprise network credentials.

    :param identity: The username or machine identity.
    :param password: The user password (for PEAP/TTLS) or private key password (for
        EAP-TLS).
    """
    identity: str
    password: str

    @classmethod
    def from_transport_model(cls, model: transport_models.s.WpaEnterprise) -> 'WpaEnterprise':
        return cls(identity=model.identity, password=model.password)

    def to_transport_model(self) -> transport_models.s.WpaEnterprise:
        return transport_models.s.WpaEnterprise(identity=self.identity, password=self.password)


@dataclass
class WiFiConfig:
    """
    :param ssid: The network name to connect to.
    :param personal: WPA2/WPA3 personal credentials, if the network uses personal auth.
    :param enterprise: 802.1X/EAP enterprise credentials, if the network uses enterprise auth.
    """
    ssid: str
    personal: WpaPersonal | None = None
    enterprise: WpaEnterprise | None = None

    @classmethod
    def from_transport_model(cls, model: transport_models.s.WiFiConfig) -> 'WiFiConfig':
        return cls(
            ssid=model.ssid,
            personal=WpaPersonal.from_transport_model(model.personal) if model.personal else None,
            enterprise=WpaEnterprise.from_transport_model(model.enterprise) if model.enterprise else None,
        )

    def to_transport_model(self) -> transport_models.s.WiFiConfig:
        return transport_models.s.WiFiConfig(
            ssid=self.ssid,
            personal=self.personal.to_transport_model() if self.personal else None,
            enterprise=self.enterprise.to_transport_model() if self.enterprise else None,
        )


@dataclass
class InterfaceConfig:
    """
    Desired configuration of a network interface.

    :param enabled: Whether the network interface is enabled.
    :param ipv4: IPv4 configuration.
    :param ethernet: Ethernet-specific configuration, applicable only to ethernet interfaces.
    :param wifi: WiFi-specific configuration, applicable only to WiFi interfaces.
    """
    enabled: bool
    ipv4: Ipv4Config
    ethernet: EthernetConfig | None = None
    wifi: WiFiConfig | None = None

    @classmethod
    def from_transport_model(cls, model: transport_models.s.InterfaceConfig) -> 'InterfaceConfig':
        return cls(
            enabled=model.enabled,
            ipv4=Ipv4Config.from_transport_model(model.ipv4),
            ethernet=EthernetConfig.from_transport_model(model.ethernet) if model.ethernet else None,
            wifi=WiFiConfig.from_transport_model(model.wifi) if model.wifi else None,
        )

    def to_transport_model(self) -> transport_models.s.InterfaceConfig:
        return transport_models.s.InterfaceConfig(
            enabled=self.enabled,
            ipv4=self.ipv4.to_transport_model(),
            ethernet=self.ethernet.to_transport_model() if self.ethernet else None,
            wifi=self.wifi.to_transport_model() if self.wifi else None,
        )
