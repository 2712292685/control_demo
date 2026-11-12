import fcntl
import socket
import struct


def route_ip_for(robot_ip, port):
    """Return the local IP the OS would use to reach robot_ip.

    No packet is sent: connect() on a UDP socket only selects a route.
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.connect((robot_ip, port))
        return sock.getsockname()[0]
    except OSError:
        return None
    finally:
        sock.close()


def list_ipv4():
    """List non-loopback IPv4 addresses of local interfaces (Linux)."""
    addresses = []
    try:
        interfaces = socket.if_nameindex()
    except OSError:
        interfaces = []

    for _, name in interfaces:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            packed = struct.pack('256s', name.encode('utf-8')[:15])
            result = fcntl.ioctl(sock.fileno(), 0x8915, packed)  # SIOCGIFADDR
            address = socket.inet_ntoa(result[20:24])
            if not address.startswith('127.') and not address.startswith('169.254.'):
                addresses.append(address)
        except OSError:
            pass
        finally:
            sock.close()

    return sorted(set(addresses))
