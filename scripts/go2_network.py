#!/usr/bin/env python3
"""Runtime Wi-Fi/AP setup, LAN discovery and the shared Go2 web hostname."""

from __future__ import annotations

import fcntl
import ipaddress
import json
import re
import socket
import subprocess
import threading
import time
from typing import Callable

import dbus

APPLICATION_ID = "unitree-go2-sdk"
DISCOVERY_PROTOCOL = "go2-sdk-web-1"
DISCOVERY_PORT = 38082
DISCOVERY_GROUP = "239.255.77.3"
PEER_TIMEOUT = 3.0


def valid_robot_id(robot: object) -> bool:
    return isinstance(robot, str) and re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]{0,63}", robot) is not None


def command(*args: str) -> str:
    result = subprocess.run(args, capture_output=True, text=True, timeout=30, check=False)
    if result.returncode:
        raise RuntimeError(result.stderr.strip() or result.stdout.strip() or "command failed")
    return result.stdout.strip()


class RobotNetwork:
    def __init__(
        self, robot: str, mode: str, port: int, ap_address: str,
        wifi_connection: str, ap_password: str, ready: Callable[[], bool], logger,
        ap_band: str = "a", ap_channel: int = 0, robot_name: str = "", ap_ssid: str = "",
    ) -> None:
        if not valid_robot_id(robot) or mode not in ("wifi", "ap"):
            raise ValueError("invalid robot or network_mode")
        if not 8 <= len(ap_password) <= 63:
            raise ValueError("ap_password must contain 8 to 63 characters")
        if ap_band not in ("a", "bg") or not 0 <= ap_channel <= 196:
            raise ValueError("invalid AP band or channel")
        ap_channel = ap_channel or (149 if ap_band == "a" else 6)
        if (ap_band == "bg" and ap_channel > 14) or (ap_band == "a" and ap_channel < 32):
            raise ValueError("AP channel does not match band")
        self.robot, self.mode, self.port = robot, mode, port
        self.name = robot_name or robot
        ap_ssid = ap_ssid or f"Go2-{robot}"[:32]
        if not 1 <= len(ap_ssid.encode("utf-8")) <= 32:
            raise ValueError("ap_ssid must contain 1 to 32 bytes")
        self.ap_address = ipaddress.IPv4Interface(ap_address)
        self.hostname = "go2-sdk.local"
        self.ready, self.logger = ready, logger
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._peers: dict[str, dict] = {}
        self._group = None
        self._started_at = time.monotonic()
        self._last_claim = 0.0
        self._thread = None
        self._socket = None
        self._bus = None
        self._wifi_device = None
        self._wifi_connected = True
        self._wifi_refresh_at = 0.0
        self._restore_connection = ""
        self._guard = open("/run/go2-web-network.lock", "a", encoding="utf-8")
        try:
            fcntl.flock(self._guard, fcntl.LOCK_EX | fcntl.LOCK_NB)
            devices = command("nmcli", "-t", "-f", "DEVICE,TYPE", "device").splitlines()
            self.interface = next(line.rsplit(":", 1)[0] for line in devices if line.endswith(":wifi"))
            self._bus = dbus.SystemBus(private=True)
            self._server = dbus.Interface(
                self._bus.get_object("org.freedesktop.Avahi", "/"), "org.freedesktop.Avahi.Server"
            )
            active = command("nmcli", "-g", "GENERAL.CON-UUID", "device", "show", self.interface)
            if mode == "wifi":
                current = command("nmcli", "-g", "GENERAL.CONNECTION", "device", "show", self.interface)
                try:
                    if wifi_connection:
                        command("nmcli", "connection", "modify", wifi_connection, "802-11-wireless.powersave", "2")
                    if wifi_connection and current != wifi_connection:
                        command("nmcli", "--wait", "20", "connection", "up", wifi_connection,
                                "ifname", self.interface)
                except (RuntimeError, subprocess.SubprocessError) as exc:
                    self.logger.warning(f"Preferred Wi-Fi {wifi_connection} unavailable: {exc}")
                    active = command("nmcli", "-g", "GENERAL.CON-UUID", "device", "show", self.interface)
                    if not active or active == "--":
                        self._connect_fallback(wifi_connection)
                active = command("nmcli", "-g", "GENERAL.CON-UUID", "device", "show", self.interface)
                if not active or active == "--":
                    self._connect_fallback(wifi_connection)
                    active = command("nmcli", "-g", "GENERAL.CON-UUID", "device", "show", self.interface)
                self.ssid = command("nmcli", "-g", "802-11-wireless.ssid", "connection", "show", active)
                manager = dbus.Interface(
                    self._bus.get_object("org.freedesktop.NetworkManager", "/org/freedesktop/NetworkManager"),
                    "org.freedesktop.NetworkManager",
                )
                self._wifi_device = dbus.Interface(
                    self._bus.get_object("org.freedesktop.NetworkManager",
                                         manager.GetDeviceByIpIface(self.interface)),
                    "org.freedesktop.DBus.Properties",
                )
            else:
                self.ssid = ap_ssid
                profile = f"go2-sdk-ap-{robot}"
                self._restore_connection = active if active and active != "--" else ""
                profiles = command("nmcli", "-g", "NAME", "connection", "show").splitlines()
                if profile in profiles:
                    profile_uuid = command("nmcli", "-g", "connection.uuid", "connection", "show", profile)
                    if self._restore_connection == profile_uuid:
                        self._restore_connection = command(
                            "nmcli", "-g", "connection.uuid", "connection", "show", wifi_connection
                        ) if wifi_connection else ""
                if profile not in profiles:
                    command("nmcli", "connection", "add", "type", "wifi", "ifname", self.interface,
                            "con-name", profile, "autoconnect", "no", "ssid", self.ssid)
                command("nmcli", "connection", "modify", profile,
                        "802-11-wireless.ssid", self.ssid,
                        "802-11-wireless.mode", "ap", "802-11-wireless.band", ap_band,
                        "802-11-wireless.channel", str(ap_channel), "802-11-wireless.powersave", "2",
                        "802-11-wireless-security.key-mgmt", "wpa-psk",
                        "802-11-wireless-security.proto", "rsn",
                        "802-11-wireless-security.pairwise", "ccmp",
                        "802-11-wireless-security.group", "ccmp",
                        "802-11-wireless-security.psk", ap_password,
                        "ipv4.method", "shared", "ipv4.addresses", str(self.ap_address),
                        "ipv6.method", "disabled", "connection.autoconnect", "no")
                command("nmcli", "connection", "up", profile, "ifname", self.interface)
            addresses = json.loads(command("ip", "-j", "-4", "address", "show", "dev", self.interface))[0]
            primary = next(item for item in addresses["addr_info"] if item["scope"] == "global")
            self.address = primary["local"]
            self.subnet = ipaddress.IPv4Interface(f"{self.address}/{primary['prefixlen']}").network
            self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_BINDTODEVICE, self.interface.encode() + b"\0")
            if mode == "wifi":
                self._socket.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_IF, socket.inet_aton(self.address))
                self._socket.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 1)
                self._socket.setsockopt(
                    socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP,
                    socket.inet_aton(DISCOVERY_GROUP) + socket.inet_aton(self.address),
                )
            self._socket.bind(("0.0.0.0", DISCOVERY_PORT))
            self._socket.settimeout(0.05)
            self.logger.info(f"Go2 {mode}: {self.ssid}; shared URL http://{self.hostname}:{port}")
        except BaseException:
            self.close()
            raise

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, name="go2-network", daemon=True)
        self._thread.start()

    def _connect_fallback(self, preferred: str) -> None:
        settings = dbus.Interface(
            self._bus.get_object("org.freedesktop.NetworkManager", "/org/freedesktop/NetworkManager/Settings"),
            "org.freedesktop.NetworkManager.Settings",
        )
        candidates = []
        for path in settings.ListConnections():
            profile = dbus.Interface(self._bus.get_object("org.freedesktop.NetworkManager", path),
                                     "org.freedesktop.NetworkManager.Settings.Connection").GetSettings()
            connection = profile["connection"]
            wireless = profile.get("802-11-wireless", {})
            if (connection["type"] != "802-11-wireless"
                    or wireless.get("mode", "infrastructure") != "infrastructure"
                    or not connection.get("autoconnect", True)
                    or preferred in (connection["id"], connection["uuid"])):
                continue
            candidates.append((int(connection.get("autoconnect-priority", 0)), str(connection["uuid"])))
        for _, uuid in sorted(candidates, reverse=True):
            try:
                command("nmcli", "--wait", "20", "connection", "up", "uuid", uuid, "ifname", self.interface)
                return
            except RuntimeError as exc:
                self.logger.warning(f"Fallback Wi-Fi {uuid} unavailable: {exc}")
        raise RuntimeError("No saved automatic Wi-Fi network is available")

    def _release(self) -> None:
        if self._group is not None:
            try:
                self._group.Free()
            except dbus.DBusException:
                pass
            self._group = None

    def _elect(self) -> None:
        candidates = [self.robot]
        with self._lock:
            candidates.extend(self._peers)
        leader = min(candidates)
        now = time.monotonic()
        if leader != self.robot:
            self._release()
        elif now - self._started_at > 2.0 and now - self._last_claim > 1.0:
            self._last_claim = now
            if self._group is not None and self._group.GetState() in (3, 4):
                self._release()  # Another owner is withdrawing its record.
            if self._group is None:
                group = dbus.Interface(
                    self._bus.get_object("org.freedesktop.Avahi", self._server.EntryGroupNew()),
                    "org.freedesktop.Avahi.EntryGroup",
                )
                self._group = group
                # Answer IPv4 address queries over both mDNS transports;
                # mobile resolvers can send their queries over IPv6. Scope
                # the record to Wi-Fi and use the standard host TTL. Avahi
                # withdraws it on shutdown and flushes it on owner changes.
                group.AddRecord(
                    socket.if_nametoindex(self.interface), -1, dbus.UInt32(1), self.hostname,
                    dbus.UInt16(1), dbus.UInt16(1), dbus.UInt32(120),
                    dbus.ByteArray(socket.inet_aton(self.address)),
                )
                group.Commit()
                self.logger.info(f"Shared Go2 URL published by {self.robot}")

    def _receive(self, payload: bytes, source: str) -> None:
        try:
            data = json.loads(payload)
            robot = data["robot"]
            port = data["port"]
            if (
                data.get("protocol") != DISCOVERY_PROTOCOL or not valid_robot_id(robot) or robot == self.robot
                or data.get("network_mode") != "wifi" or self.mode != "wifi"
                or data.get("hostname") != self.hostname
                or ipaddress.IPv4Address(source) not in self.subnet
                or isinstance(port, bool) or not isinstance(port, int) or not 1 <= port <= 65535
                or not isinstance(data.get("connected"), bool)
            ):
                return
            with self._lock:
                self._peers[robot] = {
                    "address": source, "port": port, "connected": data["connected"],
                    "name": data.get("name") if isinstance(data.get("name"), str) else robot,
                    "network_mode": "wifi", "ssid": data.get("ssid", ""), "seen": time.monotonic(),
                }
        except (ValueError, KeyError, TypeError):
            return

    def _refresh_wifi(self) -> None:
        config = self._wifi_device.Get("org.freedesktop.NetworkManager.Device", "Ip4Config")
        access_point = self._wifi_device.Get("org.freedesktop.NetworkManager.Device.Wireless", "ActiveAccessPoint")
        if config == "/" or access_point == "/":
            self._wifi_connected = False
            self._release()
            with self._lock:
                self._peers.clear()
            return
        properties = dbus.Interface(self._bus.get_object("org.freedesktop.NetworkManager", config),
                                    "org.freedesktop.DBus.Properties")
        addresses = properties.Get("org.freedesktop.NetworkManager.IP4Config", "AddressData")
        if not addresses:
            self._wifi_connected = False
            self._release()
            return
        primary = addresses[0]
        address = str(primary["address"])
        subnet = ipaddress.IPv4Interface(f"{address}/{int(primary['prefix'])}").network
        properties = dbus.Interface(self._bus.get_object("org.freedesktop.NetworkManager", access_point),
                                    "org.freedesktop.DBus.Properties")
        self.ssid = bytes(properties.Get("org.freedesktop.NetworkManager.AccessPoint", "Ssid")).decode(
            "utf-8", errors="replace"
        )
        if address != self.address or subnet != self.subnet or not self._wifi_connected:
            self._release()
            with self._lock:
                self._peers.clear()
            try:
                self._socket.setsockopt(socket.IPPROTO_IP, socket.IP_DROP_MEMBERSHIP,
                                       socket.inet_aton(DISCOVERY_GROUP) + socket.inet_aton(self.address))
            except OSError:
                pass  # The old interface address may already have been removed.
            self._socket.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_IF, socket.inet_aton(address))
            self._socket.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP,
                                   socket.inet_aton(DISCOVERY_GROUP) + socket.inet_aton(address))
            self.address, self.subnet = address, subnet
            self.logger.info(f"Go2 Wi-Fi: {self.ssid}; address {self.address}")
        self._wifi_connected = True

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                now = time.monotonic()
                if self.mode == "wifi":
                    if now >= self._wifi_refresh_at:
                        self._wifi_refresh_at = now + 2.0
                        self._refresh_wifi()
                    if not self._wifi_connected:
                        self._stop.wait(0.5)
                        continue
                with self._lock:
                    self._peers = {key: peer for key, peer in self._peers.items()
                                   if now - peer["seen"] < PEER_TIMEOUT}
                if self.mode == "wifi":
                    message = json.dumps({
                        "protocol": DISCOVERY_PROTOCOL, "robot": self.robot,
                        "name": self.name, "port": self.port,
                        "network_mode": self.mode, "hostname": self.hostname,
                        "connected": bool(self.ready()), "ssid": self.ssid,
                    }).encode()
                    self._socket.sendto(message, (str(self.subnet.broadcast_address), DISCOVERY_PORT))
                    self._socket.sendto(message, (DISCOVERY_GROUP, DISCOVERY_PORT))
                    until = time.monotonic() + 0.35
                    while time.monotonic() < until and not self._stop.is_set():
                        try:
                            payload, source = self._socket.recvfrom(2048)
                        except socket.timeout:
                            continue
                        self._receive(payload, source[0])
                self._elect()
            except (OSError, RuntimeError, subprocess.SubprocessError, dbus.DBusException) as exc:
                if isinstance(exc, dbus.DBusException):
                    self._release()
                self.logger.warning(f"Go2 network: {exc}")
            self._stop.wait(0.15 if self.mode == "wifi" else 0.5)

    def devices(self) -> list[dict]:
        with self._lock:
            peers = dict(self._peers)
        result = []
        for robot in sorted({self.robot, *peers}):
            peer = peers.get(robot)
            online = bool(peer and time.monotonic() - peer["seen"] < PEER_TIMEOUT and peer["connected"])
            local = robot == self.robot
            result.append({
                "robot": robot, "name": self.name if local else peer.get("name", robot),
                "connected": bool(self.ready()) if local else online,
                "network_mode": self.mode if local else (peer or {}).get("network_mode"),
                "ssid": self.ssid if local else (peer or {}).get("ssid", ""),
            })
        return result

    def endpoint(self, robot: str) -> tuple[str, int] | None:
        with self._lock:
            peer = self._peers.get(robot)
            if peer and peer["connected"] and time.monotonic() - peer["seen"] < PEER_TIMEOUT:
                return peer["address"], peer["port"]
        return None

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=7)
        self._release()
        if self._socket is not None:
            self._socket.close()
        if self._bus is not None:
            self._bus.close()
        if self._restore_connection:
            try:
                command("nmcli", "connection", "up", "uuid", self._restore_connection)
            except (RuntimeError, subprocess.SubprocessError) as exc:
                self.logger.error(f"Cannot restore Wi-Fi: {exc}")
        self._guard.close()
