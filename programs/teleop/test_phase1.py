#!/usr/bin/env python3
"""Phase 1 acceptance checks for keyboard_input.py (stdlib only)."""
import errno
import os
import pty
import select
import signal
import socket
import subprocess
import sys
import termios
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import keyboard_input as ki


def test_keys():
    st = ki.KeyboardState(step=10)
    st.apply_key("e")  # enable
    st.apply_key("a")
    st.apply_key("a")
    st.apply_key("s")
    assert st.grip0 == 20 and st.grip1 == 10 and st.enable
    st.apply_key("z")
    st.apply_key("x")
    assert st.grip0 == 10 and st.grip1 == 0
    st.apply_key("0")
    assert st.grip0 == 0 and st.grip1 == 0
    st.apply_key("e")  # disable forces zero
    assert st.enable is False and st.axis_pair() == (0, 0)
    st.apply_key("q")
    assert st.quit
    print("test_keys OK")


def test_disabled_no_nonzero():
    pkt = ki.build_remote_actions(80, 80, enable=False, timestamp=1.0)
    info = ki.parse_imc_remote_actions(pkt)
    assert info["fields"]["Grip0"] == "0" and info["fields"]["Grip1"] == "0"
    assert info["fields"]["Enable"] == "0"
    print("test_disabled_no_nonzero OK")


def test_announce():
    pkt = ki.build_announce(timestamp=1.0)
    info = ki.parse_imc_announce(pkt)
    assert info["msg_id"] == ki.IMC_ANNOUNCE
    assert info["src"] == ki.DEFAULT_SOURCE
    assert info["sys_name"] == "touchdiver-keyboard"
    assert info["services"] == "no;services"
    print("test_announce OK")


def test_rate_and_udp():
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(("127.0.0.1", 0))
    port = rx.getsockname()[1]
    addr = ("127.0.0.1", port)
    n = 25
    rate = 10.0
    period = 1.0 / rate
    st = ki.KeyboardState()
    st.enable = True
    st.grip0 = 30
    st.grip1 = -20
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    times = []
    t0 = time.monotonic()
    for i in range(n):
        ki.send_state(tx, addr, st)
        times.append(time.monotonic())
        if i + 1 < n:
            time.sleep(period)
    t1 = time.monotonic()
    got = []
    rx.settimeout(0.2)
    while True:
        try:
            data, _ = rx.recvfrom(2048)
            got.append(ki.parse_imc_remote_actions(data))
        except socket.timeout:
            break
    tx.close()
    rx.close()
    elapsed = t1 - t0
    hz = (n - 1) / elapsed if elapsed > 0 else 0
    print("test_rate_and_udp packets_sent=%d received=%d elapsed=%.3f hz≈%.2f" % (n, len(got), elapsed, hz))
    assert len(got) == n, (len(got), n)
    assert 8.0 <= hz <= 12.5, hz
    assert got[0]["fields"]["Grip0"] == "30"
    assert got[0]["fields"]["Grip1"] == "-20"
    assert got[0]["fields"]["Enable"] == "1"
    # inter-send gaps roughly 100ms
    gaps = [times[i + 1] - times[i] for i in range(len(times) - 1)]
    avg_gap = sum(gaps) / len(gaps)
    print("avg_gap=%.4f s" % avg_gap)
    assert 0.07 <= avg_gap <= 0.15, avg_gap
    print("test_rate_and_udp OK")


def _test_tty_exit(key):
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(("127.0.0.1", 0))
    rx.setblocking(False)
    port = rx.getsockname()[1]
    master, slave = pty.openpty()
    original = termios.tcgetattr(slave)
    proc = subprocess.Popen(
        [
            sys.executable,
            str(Path(ki.__file__).resolve()),
            "--host",
            "127.0.0.1",
            "--port",
            str(port),
            "--discovery-port",
            "39200",
        ],
        stdin=slave,
        stdout=slave,
        stderr=slave,
        close_fds=True,
        start_new_session=True,
    )
    sent_exit = False
    received_stop = False
    deadline = time.monotonic() + 5.0
    try:
        while time.monotonic() < deadline:
            readable, _, _ = select.select([master, rx], [], [], 0.1)
            for fd in readable:
                if fd == master:
                    try:
                        os.read(master, 4096)
                    except OSError as exc:
                        if exc.errno != errno.EIO:
                            raise
                else:
                    while True:
                        try:
                            data, _ = rx.recvfrom(2048)
                        except BlockingIOError:
                            break
                        fields = ki.parse_imc_remote_actions(data)["fields"]
                        if fields["Stop"] == "1":
                            received_stop = True
                            assert fields["Enable"] == "0"
                            assert fields["Grip0"] == "0" and fields["Grip1"] == "0"
                        elif not sent_exit:
                            current = termios.tcgetattr(slave)
                            assert not current[3] & termios.ECHO
                            assert not current[3] & termios.ICANON
                            if key == "SIGINT":
                                os.kill(proc.pid, signal.SIGINT)
                            else:
                                os.write(master, key)
                            sent_exit = True

            if proc.poll() is not None:
                if received_stop:
                    break

        assert sent_exit, "interactive program did not send its initial packet"
        assert received_stop, "exit did not send Stop=1 with zero axes"
        assert proc.poll() is not None, "interactive program did not exit"
        assert proc.returncode == 0, proc.returncode
        assert termios.tcgetattr(slave) == original, "terminal settings were not restored"
    finally:
        if proc.poll() is None:
            proc.terminate()
            proc.wait(timeout=2.0)
        os.close(slave)
        os.close(master)
        rx.close()


def test_tty_exit():
    _test_tty_exit(b"q")
    _test_tty_exit("SIGINT")
    print("test_tty_exit q_and_sigint OK")


def test_stop_payload():
    pkt = ki.build_remote_actions(0, 0, enable=False, stop=True, timestamp=2.0)
    info = ki.parse_imc_remote_actions(pkt)
    assert info["fields"]["Stop"] == "1"
    print("test_stop_payload OK")


if __name__ == "__main__":
    test_keys()
    test_disabled_no_nonzero()
    test_announce()
    test_stop_payload()
    test_rate_and_udp()
    test_tty_exit()
    print("ALL_PHASE1_CHECKS_OK")
