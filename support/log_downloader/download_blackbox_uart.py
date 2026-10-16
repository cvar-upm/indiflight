#!/usr/bin/env python3
"""Download the indiflight blackbox flash over the pi-protocol UART.

The FC hands its pi-protocol port over to MSP only while it is disarmed and has
received no pi-protocol frame for PI_MSP_HANDOVER_SILENCE_MS (pi.c). Stop the
platform node first. The handover lasts until the FC reboots, which this script
always requests on exit, and blocks arming until then.

The script refuses to start while another process has the device open, but it
only sees processes this user may inspect: run it as the node's user, or with
sudo if the node runs as root or in a container.

Requires pyserial. Output is the raw flash content (.bbl), readable by the
blackbox explorer and indiflightLogAnalyzer.
"""

import argparse
import fcntl
import os
import struct
import sys
import termios
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

import serial

MSP_REBOOT = 68
MSP_DATAFLASH_SUMMARY = 70
MSP_DATAFLASH_READ = 71
MSP_DATAFLASH_ERASE = 72

MSP_REBOOT_FIRMWARE = 0
FLASHFS_FLAG_READY = 0x01
FLASHFS_FLAG_SUPPORTED = 0x02

MSP_V2_HEADER_LEN = 8  # '$', 'X', direction, flag, cmd (u16), size (u16)
MSP_MAX_REPLY_PAYLOAD = 4112  # MSP_PORT_OUTBUF_SIZE on flash targets
READ_CHUNK_BYTES = 4096
READ_REPLY_HEADER = struct.Struct('<IHB')  # address, length, compression

REQUEST_TIMEOUT_S = 0.5
CHUNK_ATTEMPTS = 5
ERASE_TIMEOUT_S = 300.0  # W25Q128 chip erase is 40 s typical, 200 s max
ERASE_POLL_S = 1.0
UART_BITS_PER_BYTE = 10  # 8N1


class MspError(RuntimeError):
    """The FC answered a request with an MSP error frame."""


def crc8_dvb_s2(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def encode_request(cmd: int, payload: bytes = b'') -> bytes:
    body = struct.pack('<BHH', 0, cmd, len(payload)) + payload
    return b'$X<' + body + bytes([crc8_dvb_s2(body)])


class MspV2Link:
    """MSPv2 request/reply over a serial port that may still carry pi-protocol bytes."""

    def __init__(self, port: serial.Serial):
        self._port = port
        self._rx = bytearray()

    def request(self, cmd: int, payload: bytes = b'', timeout_s: float = REQUEST_TIMEOUT_S) -> Optional[bytes]:
        """Send one request and return the payload of its reply, or None on timeout."""
        self._port.write(encode_request(cmd, payload))
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self._rx += self._port.read(self._port.in_waiting or 1)
            while (frame := self._extract_frame()) is not None:
                reply_cmd, reply_payload, is_error = frame
                if reply_cmd != cmd:
                    continue
                if is_error:
                    raise MspError(f'FC rejected MSP command {cmd}')
                return reply_payload
        return None

    def _extract_frame(self) -> Optional[tuple]:
        """Pop the next CRC-valid reply from the receive buffer, skipping anything else."""
        while True:
            start = self._rx.find(b'$X')
            if start < 0:
                del self._rx[:-1]
                return None
            del self._rx[:start]
            if len(self._rx) < MSP_V2_HEADER_LEN:
                return None

            direction = self._rx[2]
            _, cmd, size = struct.unpack_from('<BHH', self._rx, 3)
            if direction not in b'>!' or size > MSP_MAX_REPLY_PAYLOAD:
                del self._rx[:1]
                continue

            end = MSP_V2_HEADER_LEN + size + 1
            if len(self._rx) < end:
                return None
            if crc8_dvb_s2(self._rx[3:end - 1]) != self._rx[end - 1]:
                del self._rx[:1]
                continue

            payload = bytes(self._rx[MSP_V2_HEADER_LEN:end - 1])
            del self._rx[:end]
            return cmd, payload, direction == ord('!')


def pids_holding(device: str) -> list:
    """PIDs with the device open, among the processes this user may inspect."""
    target = os.path.realpath(device)
    holders = []
    for fd_dir in Path('/proc').glob('[0-9]*/fd'):
        try:
            if any(os.readlink(fd) == target for fd in fd_dir.iterdir()):
                holders.append(int(fd_dir.parent.name))
        except (PermissionError, FileNotFoundError, ProcessLookupError):
            continue
    return holders


def parse_summary(payload: bytes) -> tuple:
    flags, _sectors, total_bytes, used_bytes = struct.unpack_from('<BIII', payload)
    return flags, total_bytes, used_bytes


def take_over_port(link: MspV2Link, timeout_s: float) -> bytes:
    """Repeat the handover request until the FC answers it as MSP."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        summary = link.request(MSP_DATAFLASH_SUMMARY)
        if summary is not None:
            return summary
    raise TimeoutError(
        f'FC did not hand the port over within {timeout_s:.0f} s: it is armed, '
        'something is still sending pi-protocol, or its firmware lacks the handover')


def read_flash(link: MspV2Link, used_bytes: int, baud: int) -> bytes:
    chunk_timeout_s = REQUEST_TIMEOUT_S + READ_CHUNK_BYTES * UART_BITS_PER_BYTE / baud
    data = bytearray()
    started = time.monotonic()
    next_report = 0
    while len(data) < used_bytes:
        address = len(data)
        request = READ_REPLY_HEADER.pack(address, min(READ_CHUNK_BYTES, used_bytes - address), 0)
        for _ in range(CHUNK_ATTEMPTS):
            reply = link.request(MSP_DATAFLASH_READ, request, chunk_timeout_s)
            if reply is None or len(reply) < READ_REPLY_HEADER.size:
                continue
            reply_address, length, compression = READ_REPLY_HEADER.unpack_from(reply)
            chunk = reply[READ_REPLY_HEADER.size:]
            if reply_address == address and compression == 0 and 0 < length == len(chunk):
                data += chunk
                break
        else:
            raise TimeoutError(f'no valid reply for flash address {address} after {CHUNK_ATTEMPTS} attempts')

        if len(data) >= next_report:
            rate_kbs = len(data) / 1024 / max(time.monotonic() - started, 1e-3)
            print(f'  {len(data) / 1e6:6.2f} / {used_bytes / 1e6:.2f} MB  {rate_kbs:5.0f} KiB/s', flush=True)
            next_report += used_bytes // 20
    return bytes(data)


def erase_flash(link: MspV2Link) -> None:
    if link.request(MSP_DATAFLASH_ERASE) is None:
        raise TimeoutError('FC did not acknowledge the erase request')
    deadline = time.monotonic() + ERASE_TIMEOUT_S
    while time.monotonic() < deadline:
        time.sleep(ERASE_POLL_S)
        summary = link.request(MSP_DATAFLASH_SUMMARY)
        if summary is None:
            continue
        flags, _, used_bytes = parse_summary(summary)
        if flags & FLASHFS_FLAG_READY and used_bytes == 0:
            return
    raise TimeoutError(f'flash erase did not finish within {ERASE_TIMEOUT_S:.0f} s')


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--device', default='/dev/ttyTHS1', help='UART wired to the FC pi-protocol port')
    parser.add_argument('--baud', type=int, default=921600, help='pi-protocol baud rate of that port')
    parser.add_argument('--output', type=Path, help='output .bbl (default: blackbox_<UTC time>.bbl here)')
    parser.add_argument('--erase', action='store_true', help='erase the flash after a complete download')
    parser.add_argument('--handover-timeout', type=float, default=10.0, help='seconds to wait for the FC')
    args = parser.parse_args()

    holders = pids_holding(args.device)
    if holders:
        print(f'{args.device} is open by PID {holders}: stop the platform node first', file=sys.stderr)
        return 1

    output = args.output or Path(f'blackbox_{datetime.now(timezone.utc):%Y%m%d_%H%M%S}.bbl')
    port = serial.Serial(args.device, args.baud, timeout=0.05)
    # Nobody else may open the port while it carries MSP
    fcntl.ioctl(port.fileno(), termios.TIOCEXCL)
    link = MspV2Link(port)
    handed_over = False
    try:
        print(f'Requesting the port from the FC on {args.device} @ {args.baud}...')
        summary = take_over_port(link, args.handover_timeout)
        handed_over = True

        flags, total_bytes, used_bytes = parse_summary(summary)
        if not flags & FLASHFS_FLAG_SUPPORTED:
            print('FC reports no blackbox flash', file=sys.stderr)
            return 1
        print(f'Flash: {used_bytes / 1e6:.2f} of {total_bytes / 1e6:.2f} MB used')
        if used_bytes == 0:
            print('Nothing to download')
            return 0

        data = read_flash(link, used_bytes, args.baud)
        output.write_bytes(data)
        print(f'Saved {len(data)} bytes to {output}')

        if args.erase:
            print('Erasing flash...')
            erase_flash(link)
            print('Flash erased')
        return 0
    except (TimeoutError, MspError) as error:
        print(f'Error: {error}', file=sys.stderr)
        return 1
    finally:
        # Also sent without a confirmed handover, whose reply may just have been lost: pi-protocol ignores it
        if handed_over:
            print('Rebooting the FC back to pi-protocol')
        try:
            link.request(MSP_REBOOT, bytes([MSP_REBOOT_FIRMWARE]))
        except MspError:
            pass
        fcntl.ioctl(port.fileno(), termios.TIOCNXCL)
        port.close()


if __name__ == '__main__':
    sys.exit(main())
