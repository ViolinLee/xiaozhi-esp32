#!/usr/bin/env python3
"""NodeHexa UART v2/legacy encoder, parser, self-test, and serial console."""

import argparse
import json
import struct
import time

try:
    import serial
except ImportError:
    serial = None

MAGIC = b"\xA5\x4E"
VERSION = 2
MAX_PAYLOAD = 512
TYPE_HELLO = 1
TYPE_REQUEST = 2
TYPE_RESPONSE = 3
TYPE_EVENT = 4
TYPE_HEARTBEAT = 5


def crc16_ccitt(data, initial=0xFFFF):
    crc = initial
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def encode_v2(message_type, sequence, payload, flags=0):
    payload_bytes = payload.encode("utf-8") if isinstance(payload, str) else bytes(payload)
    if len(payload_bytes) > MAX_PAYLOAD:
        raise ValueError("payload exceeds 512 bytes")
    body = struct.pack("<BBBHH", VERSION, message_type, flags, sequence, len(payload_bytes)) + payload_bytes
    return MAGIC + body + struct.pack("<H", crc16_ccitt(body))


def encode_legacy(payload):
    text = payload if isinstance(payload, str) else json.dumps(payload, separators=(",", ":"))
    return b"$" + text.encode("utf-8") + b"\n"


class StreamParser:
    def __init__(self):
        self.buffer = bytearray()
        self.crc_errors = 0
        self.length_errors = 0

    def feed(self, data):
        self.buffer.extend(data)
        frames = []
        while self.buffer:
            if self.buffer[0] == ord("$"):
                newline = self.buffer.find(b"\n")
                carriage = self.buffer.find(b"\r")
                ends = [index for index in (newline, carriage) if index >= 0]
                next_start = self.buffer.find(b"$", 1)
                if next_start >= 0 and (not ends or next_start < min(ends)):
                    del self.buffer[:next_start]
                    continue
                if not ends:
                    if len(self.buffer) > MAX_PAYLOAD + 1:
                        self.length_errors += 1
                        self.buffer.clear()
                    break
                end = min(ends)
                if end > 1:
                    frames.append(("legacy", TYPE_RESPONSE, 0, bytes(self.buffer[1:end])))
                del self.buffer[: end + 1]
                continue
            if self.buffer.startswith(MAGIC):
                if len(self.buffer) < 9:
                    break
                version, msg_type, flags, sequence, length = struct.unpack("<BBBHH", self.buffer[2:9])
                del flags
                if version != VERSION:
                    del self.buffer[0]
                    continue
                if length > MAX_PAYLOAD:
                    self.length_errors += 1
                    del self.buffer[:9]
                    continue
                total = 11 + length
                if len(self.buffer) < total:
                    break
                body = bytes(self.buffer[2 : 9 + length])
                received_crc = struct.unpack("<H", self.buffer[9 + length : total])[0]
                if crc16_ccitt(body) == received_crc:
                    frames.append(("v2", msg_type, sequence, bytes(self.buffer[9 : 9 + length])))
                else:
                    self.crc_errors += 1
                del self.buffer[:total]
                continue
            magic_index = self.buffer.find(MAGIC)
            legacy_index = self.buffer.find(b"$")
            candidates = [index for index in (magic_index, legacy_index) if index >= 0]
            if not candidates:
                if self.buffer.endswith(MAGIC[:1]):
                    self.buffer[:] = MAGIC[:1]
                    break
                self.buffer.clear()
                break
            del self.buffer[: min(candidates)]
        return frames


def run_self_test():
    assert crc16_ccitt(b"123456789") == 0x29B1
    payload = b'{"movementMode":2}'
    encoded = encode_v2(TYPE_REQUEST, 42, payload)
    parser = StreamParser()
    frames = []
    for byte in b"noise\xA5\x00" + encoded + encode_legacy('{"status":"success"}'):
        frames.extend(parser.feed(bytes([byte])))
    assert frames[0] == ("v2", TYPE_REQUEST, 42, payload)
    assert frames[1] == ("legacy", TYPE_RESPONSE, 0, b'{"status":"success"}')
    assert StreamParser().feed(b'$truncated${"status":"success"}\n') == [
        ("legacy", TYPE_RESPONSE, 0, b'{"status":"success"}')
    ]
    damaged = bytearray(encoded)
    damaged[-1] ^= 0xFF
    assert parser.feed(damaged) == []
    assert parser.crc_errors == 1
    oversized = MAGIC + struct.pack("<BBBHH", VERSION, TYPE_REQUEST, 0, 1, MAX_PAYLOAD + 1)
    parser.feed(oversized)
    assert parser.length_errors == 1
    event = encode_v2(TYPE_EVENT, 0, '{"event":"lowBattery"}')
    response = encode_v2(TYPE_RESPONSE, 77, '{"status":"success"}')
    mixed = StreamParser().feed(event + response)
    assert mixed[0][1] == TYPE_EVENT and mixed[1][1:3] == (TYPE_RESPONSE, 77)
    print("NodeHexa UART protocol self-test: PASS")


class SerialConsole:
    def __init__(self, port, protocol):
        if serial is None:
            raise RuntimeError("pyserial is required for live serial mode")
        self.serial = serial.Serial(port, 115200, timeout=0.1, write_timeout=1)
        self.protocol = protocol
        self.sequence = 1
        self.parser = StreamParser()

    def close(self):
        self.serial.close()

    def send_json(self, payload):
        text = json.dumps(payload, separators=(",", ":"), ensure_ascii=False)
        if self.protocol == "v2":
            sequence = self.sequence
            self.sequence = 1 if sequence == 0xFFFF else sequence + 1
            frame = encode_v2(TYPE_REQUEST, sequence, text, flags=0x02)
        else:
            frame = encode_legacy(text)
        self.serial.write(frame)

    def read_for(self, seconds=1.0):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            data = self.serial.read(self.serial.in_waiting or 1)
            for frame_format, message_type, sequence, payload in self.parser.feed(data):
                print(frame_format, message_type, sequence, payload.decode("utf-8", errors="replace"))


def motion_payload(mode, count=3):
    names = ("standby", "forward", "forwardfast", "backward", "turnleft", "turnright",
             "shiftleft", "shiftright", "climb", "rotatex", "rotatey", "rotatez", "twist")
    if not 0 <= mode < len(names) or not 1 <= count <= 10:
        raise ValueError("mode must be 0-12; count must be 1-10")
    if mode == 0:
        return {"stop": True, "movementMode": 1}
    return {"mode": names[mode], "cycles" if mode >= 9 else "steps": count}


def main():
    argument_parser = argparse.ArgumentParser()
    argument_parser.add_argument("port", nargs="?", default="/dev/ttyUSB0")
    argument_parser.add_argument("--protocol", choices=("v2", "legacy"), default="v2")
    argument_parser.add_argument("--self-test", action="store_true")
    args = argument_parser.parse_args()
    if args.self_test:
        run_self_test()
        return
    console = SerialConsole(args.port, args.protocol)
    try:
        while True:
            value = input("mode 0-12 [count 1-10], s0-s3 speed, p showtime/freestyle/beatsway, q: ").strip().lower()
            if value == "q":
                break
            if value.startswith("p "):
                name = value[2:].strip()
                if name not in ("showtime", "freestyle", "beatsway"):
                    print("unknown performance")
                    continue
                console.send_json({"performance": name, "repeat": False})
            elif value.startswith("s"):
                level = int(value[1:])
                if not 0 <= level <= 3:
                    raise ValueError("speedLevel must be 0-3")
                console.send_json({"speedLevel": level})
            else:
                parts = value.split()
                mode = int(parts[0])
                count = int(parts[1]) if len(parts) > 1 else (1 if mode >= 9 else 3)
                console.send_json(motion_payload(mode, count))
            console.read_for()
    finally:
        console.close()


if __name__ == "__main__":
    main()
