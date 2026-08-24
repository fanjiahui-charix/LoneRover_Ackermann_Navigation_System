"""Small parser for the published STM32 IMU frame format."""

from __future__ import annotations

import struct
from dataclasses import dataclass


HEAD0 = 0xAA
HEAD1 = 0x55
CMD_IMU9_RAW = 0x10
PAYLOAD_LEN = 20
FRAME_LENGTH = 1 + PAYLOAD_LEN + 1  # command + payload + XOR


@dataclass(frozen=True)
class ImuRawSample:
    ax: int
    ay: int
    az: int
    gx: int
    gy: int
    gz: int
    mx: int
    my: int
    mz: int
    temp_raw: int


def xor_checksum(data: bytes) -> int:
    value = 0
    for byte in data:
        value ^= byte
    return value & 0xFF


class FrameParser:
    def __init__(self) -> None:
        self._buffer = bytearray()

    def feed(self, data: bytes) -> None:
        self._buffer.extend(data)

    def pop(self) -> ImuRawSample | None:
        while True:
            if len(self._buffer) < 3:
                return None
            start = self._buffer.find(bytes((HEAD0,)))
            if start < 0:
                self._buffer.clear()
                return None
            if start:
                del self._buffer[:start]
            if len(self._buffer) < 3:
                return None
            if self._buffer[1] != HEAD1:
                del self._buffer[0]
                continue

            length = self._buffer[2]
            total = 3 + length
            if len(self._buffer) < total:
                return None
            frame = bytes(self._buffer[:total])
            del self._buffer[:total]
            if length != FRAME_LENGTH or frame[3] != CMD_IMU9_RAW:
                continue
            if xor_checksum(frame[2:-1]) != frame[-1]:
                continue
            values = struct.unpack("<10h", frame[4:4 + PAYLOAD_LEN])
            return ImuRawSample(*values)
