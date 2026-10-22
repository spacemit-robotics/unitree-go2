package com.spacemit.go2web;

import android.net.LinkAddress;
import android.net.LinkProperties;
import android.net.Network;
import android.net.RouteInfo;

import org.json.JSONObject;

import java.io.ByteArrayOutputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.net.DatagramPacket;
import java.net.HttpURLConnection;
import java.net.Inet4Address;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.MulticastSocket;
import java.net.NetworkInterface;
import java.net.Proxy;
import java.net.SocketTimeoutException;
import java.net.URL;
import java.nio.charset.StandardCharsets;
import java.util.LinkedHashSet;
import java.util.Set;

/** Resolves the existing Go2 page on Wi-Fi without relying on a browser DNS cache. */
final class LocalDiscovery {
    static final String DEFAULT_URL = "http://go2-sdk.local:8082/";

    private LocalDiscovery() {}

    static String find(Network network, LinkProperties links, String configured, String last) {
        Set<String> candidates = new LinkedHashSet<>();
        if (!DEFAULT_URL.equals(configured)) candidates.add(configured);
        if (last != null && !last.isEmpty()) candidates.add(last);
        for (String candidate : candidates) {
            if (isServer(network, candidate)) return candidate;
        }
        candidates.clear();
        try {
            for (String address : mdns(network)) candidates.add("http://" + address + ":8082/");
        } catch (IOException ignored) { /* Discovery broadcasts or direct addresses can still work. */ }
        for (String candidate : candidates) {
            if (isServer(network, candidate)) return candidate;
        }
        candidates.clear();
        try {
            candidates.addAll(peers(network, links));
        } catch (IOException ignored) { /* Some access points filter multicast. */ }
        // A hotspot's gateway is the board.
        if (links != null) {
            for (RouteInfo route : links.getRoutes()) {
                if (route.isDefaultRoute() && route.getGateway() instanceof Inet4Address) {
                    candidates.add("http://" + route.getGateway().getHostAddress() + ":8082/");
                }
            }
        }
        for (String candidate : candidates) {
            if (isServer(network, candidate)) return candidate;
        }
        return null;
    }

    static boolean isServer(Network network, String base) {
        HttpURLConnection connection = null;
        try {
            connection = (HttpURLConnection) network.openConnection(new URL(base + "api/robots"), Proxy.NO_PROXY);
            connection.setConnectTimeout(700);
            connection.setReadTimeout(700);
            connection.setInstanceFollowRedirects(false);
            if (connection.getResponseCode() != 200) return false;
            try (InputStream input = connection.getInputStream()) {
                ByteArrayOutputStream data = new ByteArrayOutputStream();
                byte[] buffer = new byte[2048];
                int length;
                while ((length = input.read(buffer)) >= 0) {
                    data.write(buffer, 0, length);
                    if (data.size() > 16384) return false;
                }
                JSONObject body = new JSONObject(data.toString(StandardCharsets.UTF_8.name()));
                return "unitree-go2-sdk".equals(body.optString("application"))
                        && validRobotId(body.optString("local_robot")) && body.optJSONArray("robots") != null;
            }
        } catch (Exception ignored) {
            return false;
        } finally {
            if (connection != null) connection.disconnect();
        }
    }

    private static boolean validRobotId(String robot) {
        return robot.matches("[A-Za-z0-9][A-Za-z0-9_-]{0,63}");
    }

    private static Set<String> peers(Network network, LinkProperties links) throws IOException {
        Set<String> result = new LinkedHashSet<>();
        if (links == null || links.getInterfaceName() == null) return result;
        NetworkInterface iface = NetworkInterface.getByName(links.getInterfaceName());
        try (MulticastSocket socket = new MulticastSocket(null)) {
            socket.setReuseAddress(true);
            socket.bind(new InetSocketAddress(38082));
            network.bindSocket(socket);
            socket.joinGroup(new InetSocketAddress("239.255.77.3", 38082), iface);
            socket.setSoTimeout(200);
            long until = System.nanoTime() + 1_300_000_000L;
            while (System.nanoTime() < until) {
                byte[] bytes = new byte[2048];
                DatagramPacket packet = new DatagramPacket(bytes, bytes.length);
                try {
                    socket.receive(packet);
                } catch (SocketTimeoutException ignored) {
                    continue;
                }
                if (!(packet.getAddress() instanceof Inet4Address) || !localAddress(packet.getAddress(), links)) continue;
                try {
                    JSONObject body = new JSONObject(new String(bytes, 0, packet.getLength(), StandardCharsets.UTF_8));
                    int port = body.optInt("port", 0);
                    if (body.optString("protocol").equals("go2-sdk-web-1") && validRobotId(body.optString("robot"))
                            && body.optBoolean("connected") && port > 0 && port <= 65535) {
                        result.add("http://" + packet.getAddress().getHostAddress() + ":" + port + "/");
                    }
                } catch (Exception ignored) { /* Ignore unrelated LAN datagrams. */ }
            }
        }
        return result;
    }

    private static boolean localAddress(InetAddress address, LinkProperties links) {
        for (LinkAddress local : links.getLinkAddresses()) {
            if (!(local.getAddress() instanceof Inet4Address)) continue;
            byte[] a = address.getAddress(), b = local.getAddress().getAddress();
            int remaining = local.getPrefixLength();
            boolean match = true;
            for (int i = 0; i < 4; i++) {
                int mask = 0xff << (8 - Math.min(8, remaining)) & 0xff;
                if ((a[i] & mask) != (b[i] & mask)) match = false;
                remaining = Math.max(0, remaining - 8);
            }
            if (match) return true;
        }
        return false;
    }

    private static Set<String> mdns(Network network) throws IOException {
        Set<String> addresses = new LinkedHashSet<>();
        ByteArrayOutputStream bytes = new ByteArrayOutputStream();
        try (DataOutputStream query = new DataOutputStream(bytes)) {
            query.writeShort(0x6720); query.writeShort(0); query.writeShort(1);
            query.writeShort(0); query.writeShort(0); query.writeShort(0);
            query.writeByte(7); query.writeBytes("go2-sdk"); query.writeByte(5); query.writeBytes("local");
            query.writeByte(0); query.writeShort(1); query.writeShort(1);
        }
        try (MulticastSocket socket = new MulticastSocket(null)) {
            network.bindSocket(socket);
            socket.bind(new InetSocketAddress(0));
            socket.setTimeToLive(255);
            socket.setSoTimeout(200);
            byte[] query = bytes.toByteArray();
            socket.send(new DatagramPacket(query, query.length, InetAddress.getByName("224.0.0.251"), 5353));
            long until = System.nanoTime() + 900_000_000L;
            while (System.nanoTime() < until) {
                byte[] response = new byte[4096];
                DatagramPacket packet = new DatagramPacket(response, response.length);
                try {
                    socket.receive(packet);
                } catch (SocketTimeoutException ignored) {
                    continue;
                }
                addresses.addAll(parseMdns(response, packet.getLength()));
                if (!addresses.isEmpty()) break;
            }
        }
        return addresses;
    }

    static Set<String> parseMdns(byte[] data, int length) {
        Set<String> result = new LinkedHashSet<>();
        if (length < 12 || length > data.length || unsigned(data, 0) != 0x6720 || (data[2] & 0x80) == 0) return result;
        try {
            int[] offset = {12};
            for (int i = 0; i < unsigned(data, 4); i++) { readName(data, length, offset); offset[0] += 4; }
            int count = unsigned(data, 6) + unsigned(data, 8) + unsigned(data, 10);
            for (int i = 0; i < count; i++) {
                String name = readName(data, length, offset);
                int start = offset[0];
                if (start + 10 > length) break;
                int type = unsigned(data, start), size = unsigned(data, start + 8);
                offset[0] += 10;
                if (offset[0] + size > length) break;
                if (name.equalsIgnoreCase("go2-sdk.local") && type == 1 && size == 4) {
                    byte[] address = java.util.Arrays.copyOfRange(data, offset[0], offset[0] + 4);
                    result.add(InetAddress.getByAddress(address).getHostAddress());
                }
                offset[0] += size;
            }
        } catch (IOException | IndexOutOfBoundsException ignored) { /* Reject malformed packets. */ }
        return result;
    }

    private static int unsigned(byte[] data, int offset) {
        return (data[offset] & 255) << 8 | data[offset + 1] & 255;
    }

    private static String readName(byte[] data, int length, int[] position) throws IOException {
        StringBuilder name = new StringBuilder();
        int offset = position[0];
        boolean jumped = false;
        for (int hops = 0; hops < 64 && offset < length; hops++) {
            int size = data[offset++] & 255;
            if (size == 0) { if (!jumped) position[0] = offset; return name.toString(); }
            if ((size & 192) == 192) {
                if (offset >= length) break;
                if (!jumped) position[0] = offset + 1;
                offset = (size & 63) << 8 | data[offset] & 255;
                jumped = true;
            } else {
                if (size > 63 || offset + size > length) break;
                if (name.length() > 0) name.append('.');
                name.append(new String(data, offset, size, StandardCharsets.US_ASCII));
                offset += size;
                if (!jumped) position[0] = offset;
            }
        }
        throw new IOException("invalid DNS name");
    }
}
