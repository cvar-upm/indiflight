"""Checks download_blackbox_uart.py against pi.c. Run: python3 test_download_blackbox_uart.py"""

import re
import struct
from pathlib import Path

import download_blackbox_uart as dl

PI_C = Path(__file__).resolve().parents[2] / 'src/main/telemetry/pi.c'


class FakePort:
    def __init__(self, rx: bytes):
        self.rx = bytearray(rx)
        self.tx = bytearray()

    @property
    def in_waiting(self) -> int:
        return len(self.rx)

    def write(self, data: bytes) -> None:
        self.tx += data

    def read(self, size: int) -> bytes:
        out = bytes(self.rx[:size])
        del self.rx[:size]
        return out


def encode_reply(cmd: int, payload: bytes, direction: bytes = b'>') -> bytes:
    body = struct.pack('<BHH', 0, cmd, len(payload)) + payload
    return b'$X' + direction + body + bytes([dl.crc8_dvb_s2(body)])


def test_handover_request_matches_firmware() -> None:
    initializer = re.search(r'piMspHandoverRequest\[\] = \{(.*?)\};', PI_C.read_text()).group(1)
    tokens = [token.strip() for token in initializer.split(',')]
    firmware = bytes(ord(t[1]) if t.startswith("'") else int(t, 16) for t in tokens)
    assert dl.encode_request(dl.MSP_DATAFLASH_SUMMARY) == firmware


def test_reply_found_among_pi_bytes() -> None:
    summary = struct.pack('<BIII', 3, 256, 16777216, 1234)
    fake_header_bad_crc = b'$X>\x00\x46\x00\x02\x00ab\x00'
    rx = (b'\xfe\x01\x24\x58' + fake_header_bad_crc + encode_reply(dl.MSP_DATAFLASH_READ, b'stale')
          + b'\xfe$X' + encode_reply(dl.MSP_DATAFLASH_SUMMARY, summary) + b'\xfe\x00')
    port = FakePort(rx)
    assert dl.MspV2Link(port).request(dl.MSP_DATAFLASH_SUMMARY) == summary
    assert port.tx == dl.encode_request(dl.MSP_DATAFLASH_SUMMARY)


def test_error_reply_raises() -> None:
    port = FakePort(encode_reply(dl.MSP_DATAFLASH_ERASE, b'', direction=b'!'))
    try:
        dl.MspV2Link(port).request(dl.MSP_DATAFLASH_ERASE)
    except dl.MspError:
        return
    raise AssertionError('error frame not reported')


if __name__ == '__main__':
    test_handover_request_matches_firmware()
    test_reply_found_among_pi_bytes()
    test_error_reply_raises()
    print('ok')
