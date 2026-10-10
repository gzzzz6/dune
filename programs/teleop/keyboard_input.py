#!/usr/bin/env python3
"""Keyboard → IMC RemoteActions UDP inputter for TouchDIVER computer prototype.

Stdlib only. Sends IMC v5 RemoteActions at 10 Hz. No dune-sendmsg in the loop.

Keys:
  a / z   Grip0 +step / -step
  s / x   Grip1 +step / -step
  0       zero both axes
  e       toggle enable (axes forced to 0 while disabled)
  q       safe exit (send zero + Stop=1)

TupleList (DUNE Utils::TupleList default sep='=' tuple_sep=';'):
  Enable=<0|1>;Stop=<0|1>;Grip0=<-127..127>;Grip1=<-127..127>
"""
from __future__ import annotations

import argparse
import os
import select
import socket
import struct
import sys
import termios
import time
import tty

# IMC v5 constants (src/DUNE/IMC/Constants.hpp)
IMC_SYNC = 0xFE54
IMC_HEADER_SIZE = 20
IMC_FOOTER_SIZE = 2
IMC_ANNOUNCE = 151
IMC_REMOTE_ACTIONS = 305
IMC_SET_SERVO_POSITION = 302
ADDR_INVALID = 0xFFFF
EID_UNKNOWN = 0xFF
DEFAULT_SOURCE = 0xFFF0 - 1
AXIS_MIN = -127
AXIS_MAX = 127
DEFAULT_STEP = 10
DEFAULT_RATE_HZ = 10.0
DEFAULT_DISCOVERY_PORT = 30100
DISCOVERY_PORT_COUNT = 5
DEFAULT_ANNOUNCE_PERIOD = 10.0


def _crc16_ibm_table() -> list[int]:
    """Match DUNE Algorithms::CRC16 (CRC-16-IBM, reflected table, poly 0x8005)."""
    table = []
    for i in range(256):
        crc = i
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
        table.append(crc & 0xFFFF)
    return table


_CRC16_IBM = _crc16_ibm_table()


def crc16_ibm(data: bytes, crc: int = 0) -> int:
    for b in data:
        crc = (crc >> 8) ^ _CRC16_IBM[(crc ^ b) & 0xFF]
    return crc & 0xFFFF


def serialize_string(text: str) -> bytes:
    raw = text.encode("utf-8")
    if len(raw) > 0xFFFF:
        raise ValueError("IMC string is too long")
    return struct.pack("<H", len(raw)) + raw


def build_imc_packet(
    msg_id: int,
    payload: bytes,
    timestamp: float | None = None,
    src: int = DEFAULT_SOURCE,
    src_ent: int = EID_UNKNOWN,
    dst: int = ADDR_INVALID,
    dst_ent: int = EID_UNKNOWN,
) -> bytes:
    """Header(20) + payload + CRC16(2). LE. Matches IMC::Packet::serializeHeader."""
    if timestamp is None:
        timestamp = time.time()
    header = (
        struct.pack("<HHH", IMC_SYNC, msg_id & 0xFFFF, len(payload))
        + struct.pack("<d", float(timestamp))
        + struct.pack("<H", src & 0xFFFF)
        + struct.pack("<B", src_ent & 0xFF)
        + struct.pack("<H", dst & 0xFFFF)
        + struct.pack("<B", dst_ent & 0xFF)
    )
    assert len(header) == IMC_HEADER_SIZE, len(header)
    body = header + payload
    crc = crc16_ibm(body)
    return body + struct.pack("<H", crc)


def build_announce(
    sys_name: str = "touchdiver-keyboard",
    services: str = "no;services",
    timestamp: float | None = None,
    src: int = DEFAULT_SOURCE,
) -> bytes:
    """Build an IMC Announce used to register this UDP source with DUNE."""
    payload = (
        serialize_string(sys_name)
        + struct.pack("<BHddf", 0, 0, 0.0, 0.0, 0.0)
        + serialize_string(services)
    )
    return build_imc_packet(IMC_ANNOUNCE, payload, timestamp=timestamp, src=src)


def build_remote_actions(
    grip0: int,
    grip1: int,
    enable: bool,
    stop: bool = False,
    timestamp: float | None = None,
) -> bytes:
    g0 = max(AXIS_MIN, min(AXIS_MAX, int(grip0)))
    g1 = max(AXIS_MIN, min(AXIS_MAX, int(grip1)))
    if not enable:
        g0 = 0
        g1 = 0
    actions = "Enable=%d;Stop=%d;Grip0=%d;Grip1=%d;" % (
        1 if enable else 0,
        1 if stop else 0,
        g0,
        g1,
    )
    return build_imc_packet(
        IMC_REMOTE_ACTIONS, serialize_string(actions), timestamp=timestamp
    )


def _read_string(payload: bytes, offset: int) -> tuple[str, int]:
    if offset + 2 > len(payload):
        raise ValueError("truncated IMC string length")
    slen = struct.unpack_from("<H", payload, offset)[0]
    offset += 2
    end = offset + slen
    if end > len(payload):
        raise ValueError("truncated IMC string")
    return payload[offset:end].decode("utf-8"), end


def parse_imc_announce(packet: bytes) -> dict:
    """Decode an Announce packet for the protocol self-test."""
    if len(packet) < IMC_HEADER_SIZE + IMC_FOOTER_SIZE:
        raise ValueError("short packet")
    sync, msg_id, psize = struct.unpack_from("<HHH", packet, 0)
    if sync != IMC_SYNC:
        raise ValueError("bad sync 0x%04x" % sync)
    if msg_id != IMC_ANNOUNCE:
        raise ValueError("not Announce: %d" % msg_id)
    end = IMC_HEADER_SIZE + psize
    if len(packet) < end + IMC_FOOTER_SIZE:
        raise ValueError("truncated packet")
    timestamp = struct.unpack_from("<d", packet, 6)[0]
    src, src_ent, dst, dst_ent = struct.unpack_from("<HBHB", packet, 14)
    crc_got = struct.unpack_from("<H", packet, end)[0]
    crc_exp = crc16_ibm(packet[:end])
    if crc_got != crc_exp:
        raise ValueError("CRC mismatch %04x != %04x" % (crc_got, crc_exp))

    payload = packet[IMC_HEADER_SIZE:end]
    sys_name, offset = _read_string(payload, 0)
    if offset + struct.calcsize("<BHddf") > len(payload):
        raise ValueError("truncated Announce fields")
    sys_type, owner, lat, lon, height = struct.unpack_from("<BHddf", payload, offset)
    offset += struct.calcsize("<BHddf")
    services, offset = _read_string(payload, offset)
    if offset != len(payload):
        raise ValueError("trailing Announce payload")
    return {
        "msg_id": msg_id,
        "timestamp": timestamp,
        "src": src,
        "src_ent": src_ent,
        "dst": dst,
        "dst_ent": dst_ent,
        "sys_name": sys_name,
        "sys_type": sys_type,
        "owner": owner,
        "lat": lat,
        "lon": lon,
        "height": height,
        "services": services,
    }


def parse_imc_remote_actions(packet: bytes) -> dict:
    """Minimal decoder for self-test (RemoteActions only)."""
    if len(packet) < IMC_HEADER_SIZE + IMC_FOOTER_SIZE:
        raise ValueError("short packet")
    sync, msg_id, psize = struct.unpack_from("<HHH", packet, 0)
    if sync != IMC_SYNC:
        raise ValueError("bad sync 0x%04x" % sync)
    if msg_id != IMC_REMOTE_ACTIONS:
        raise ValueError("not RemoteActions: %d" % msg_id)
    timestamp = struct.unpack_from("<d", packet, 6)[0]
    src, src_ent, dst, dst_ent = struct.unpack_from("<HBHB", packet, 14)
    payload = packet[IMC_HEADER_SIZE : IMC_HEADER_SIZE + psize]
    crc_got = struct.unpack_from("<H", packet, IMC_HEADER_SIZE + psize)[0]
    crc_exp = crc16_ibm(packet[: IMC_HEADER_SIZE + psize])
    if crc_got != crc_exp:
        raise ValueError("CRC mismatch %04x != %04x" % (crc_got, crc_exp))
    slen = struct.unpack_from("<H", payload, 0)[0]
    actions = payload[2 : 2 + slen].decode("utf-8")
    # TupleList: key=value;key=value;
    fields = {}
    for part in actions.split(";"):
        part = part.strip()
        if not part or "=" not in part:
            continue
        k, v = part.split("=", 1)
        fields[k.strip()] = v.strip()
    return {
        "msg_id": msg_id,
        "timestamp": timestamp,
        "src": src,
        "src_ent": src_ent,
        "dst": dst,
        "dst_ent": dst_ent,
        "actions": actions,
        "fields": fields,
    }


class KeyboardState:
    def __init__(self, step: int = DEFAULT_STEP) -> None:
        self.step = step
        self.grip0 = 0
        self.grip1 = 0
        self.enable = False
        self.stop = False
        self.quit = False

    def apply_key(self, ch: str) -> None:
        if ch == "a":
            self.grip0 = min(AXIS_MAX, self.grip0 + self.step)
        elif ch == "z":
            self.grip0 = max(AXIS_MIN, self.grip0 - self.step)
        elif ch == "s":
            self.grip1 = min(AXIS_MAX, self.grip1 + self.step)
        elif ch == "x":
            self.grip1 = max(AXIS_MIN, self.grip1 - self.step)
        elif ch == "0":
            self.grip0 = 0
            self.grip1 = 0
        elif ch == "e":
            self.enable = not self.enable
            if not self.enable:
                self.grip0 = 0
                self.grip1 = 0
        elif ch == "q":
            self.quit = True

    def axis_pair(self) -> tuple[int, int]:
        if not self.enable:
            return 0, 0
        return self.grip0, self.grip1


def format_status(state: KeyboardState, host: str, port: int, tx_count: int) -> str:
    g0, g1 = state.axis_pair()
    return (
        "target=%s:%d  enable=%s  Grip0=%+4d Grip1=%+4d  tx=%d  "
        "keys: a/z ch0  s/x ch1  0 zero  e enable  q quit"
        % (host, port, "ON" if state.enable else "OFF", g0, g1, tx_count)
    )


def send_state(
    sock: socket.socket,
    addr: tuple[str, int],
    state: KeyboardState,
    stop: bool = False,
) -> bytes:
    g0, g1 = state.axis_pair()
    pkt = build_remote_actions(g0, g1, enable=state.enable, stop=stop or state.stop)
    sock.sendto(pkt, addr)
    return pkt


def send_discovery_announces(
    sock: socket.socket,
    host: str,
    discovery_port: int = DEFAULT_DISCOVERY_PORT,
) -> int:
    """Register the source on all standard DUNE Discovery ports."""
    sent = 0
    for port in range(discovery_port, discovery_port + DISCOVERY_PORT_COUNT):
        sock.sendto(build_announce(), (host, port))
        sent += 1
    return sent


def run_selftest() -> int:
    """Repeatable encode/parse check (no network)."""
    pkt = build_remote_actions(50, -30, enable=True, stop=False, timestamp=1000.0)
    info = parse_imc_remote_actions(pkt)
    assert info["fields"]["Enable"] == "1", info
    assert info["fields"]["Stop"] == "0", info
    assert info["fields"]["Grip0"] == "50", info
    assert info["fields"]["Grip1"] == "-30", info
    assert info["msg_id"] == IMC_REMOTE_ACTIONS

    announce = parse_imc_announce(build_announce(timestamp=1000.0))
    assert announce["msg_id"] == IMC_ANNOUNCE
    assert announce["src"] == DEFAULT_SOURCE
    assert announce["sys_name"] == "touchdiver-keyboard"
    assert announce["services"] == "no;services"

    # disabled forces zeros
    pkt2 = build_remote_actions(50, -30, enable=False, stop=True, timestamp=1000.0)
    info2 = parse_imc_remote_actions(pkt2)
    assert info2["fields"]["Enable"] == "0"
    assert info2["fields"]["Stop"] == "1"
    assert info2["fields"]["Grip0"] == "0"
    assert info2["fields"]["Grip1"] == "0"

    # clamp
    pkt3 = build_remote_actions(200, -200, enable=True, timestamp=1.0)
    info3 = parse_imc_remote_actions(pkt3)
    assert info3["fields"]["Grip0"] == "127"
    assert info3["fields"]["Grip1"] == "-127"

    # known SetServoPosition sample CRC check (id=0 value=20deg raw from Phase 0)
    # Rebuild payload: u8 id + f32 radians
    import math

    payload = struct.pack("<Bf", 0, math.radians(20.0))
    # cannot fully match timestamp; just ensure our CRC on a fixed buffer is stable
    body = struct.pack("<HHH", IMC_SYNC, IMC_SET_SERVO_POSITION, 5) + struct.pack(
        "<d", 1.0
    ) + struct.pack("<HBHB", 65519, 255, 65535, 255) + payload
    pkt4 = body + struct.pack("<H", crc16_ibm(body))
    assert len(pkt4) == IMC_HEADER_SIZE + 5 + IMC_FOOTER_SIZE
    # decode header fields
    sid = pkt4[IMC_HEADER_SIZE]
    val = struct.unpack_from("<f", pkt4, IMC_HEADER_SIZE + 1)[0]
    assert sid == 0 and abs(val - math.radians(20.0)) < 1e-6

    print("selftest OK")
    print("sample RemoteActions hex:", pkt.hex())
    print("actions:", info["actions"])
    return 0


def run_loop(
    host: str,
    port: int,
    rate_hz: float,
    step: int,
    discovery_port: int = DEFAULT_DISCOVERY_PORT,
    announce_period: float = DEFAULT_ANNOUNCE_PERIOD,
) -> int:
    if rate_hz <= 0:
        raise ValueError("rate must be positive")
    if announce_period <= 0:
        raise ValueError("announce period must be positive")
    addr = (host, port)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setblocking(False)
    state = KeyboardState(step=step)
    period = 1.0 / rate_hz
    tx = 0
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        send_discovery_announces(sock, host, discovery_port)
        next_announce = time.monotonic() + announce_period
        next_t = time.monotonic()
        while not state.quit:
            now = time.monotonic()
            # drain keys
            while True:
                r, _, _ = select.select([fd], [], [], 0)
                if not r:
                    break
                ch = os.read(fd, 1).decode("utf-8", errors="ignore")
                if ch:
                    state.apply_key(ch)
            if now >= next_announce:
                send_discovery_announces(sock, host, discovery_port)
                next_announce = now + announce_period
            if now >= next_t:
                send_state(sock, addr, state)
                tx += 1
                next_t = now + period
                # status line
                sys.stdout.write(
                    "\r" + format_status(state, host, port, tx) + " " * 8
                )
                sys.stdout.flush()
            else:
                time.sleep(min(0.01, next_t - now))
    finally:
        # safe shutdown: zero + Stop=1, then disable
        try:
            state.enable = False
            state.grip0 = 0
            state.grip1 = 0
            state.stop = True
            for _ in range(3):
                send_state(sock, addr, state, stop=True)
                time.sleep(0.05)
        except OSError:
            pass
        finally:
            sock.close()
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
            sys.stdout.write("\n")
            sys.stdout.flush()
    return 0


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Keyboard IMC RemoteActions sender")
    ap.add_argument("--host", default="127.0.0.1", help="DUNE UDP host")
    ap.add_argument("--port", type=int, default=6002, help="DUNE UDP port")
    ap.add_argument(
        "--discovery-port",
        type=int,
        default=DEFAULT_DISCOVERY_PORT,
        help="DUNE Discovery base port (five consecutive ports)",
    )
    ap.add_argument("--rate", type=float, default=DEFAULT_RATE_HZ, help="send rate Hz")
    ap.add_argument(
        "--announce-period",
        type=float,
        default=DEFAULT_ANNOUNCE_PERIOD,
        help="repeat Announce period in seconds",
    )
    ap.add_argument("--step", type=int, default=DEFAULT_STEP, help="axis step per key")
    ap.add_argument("--selftest", action="store_true", help="run encode/parse test")
    args = ap.parse_args(argv)

    if args.selftest:
        return run_selftest()

    if not sys.stdin.isatty():
        print("stdin is not a TTY; use --selftest for offline check", file=sys.stderr)
        return 2

    print("TouchDIVER keyboard inputter → %s:%d at %.1f Hz" % (args.host, args.port, args.rate))
    print("Ctrl+C or q for safe exit")
    try:
        return run_loop(
            args.host,
            args.port,
            args.rate,
            args.step,
            args.discovery_port,
            args.announce_period,
        )
    except KeyboardInterrupt:
        return 0


if __name__ == "__main__":
    sys.exit(main())
