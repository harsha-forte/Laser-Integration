#!/usr/bin/env python3
"""
servo_off.py - connect to the A6-RS drive on /dev/ttyACM0 and disable the servo.

How it works (per A6-RS manual, sections 8.5 and 9.1):
  DI5 function = S-ON (C04.10 = 1, factory default)
  DI5 logic    = C04.11  (0 = NO, 1 = NC, writable during operation)
  With nothing wired to DI5:  C04.11 = 1 -> S-ON active   -> servo ON
                              C04.11 = 0 -> S-ON inactive -> servo OFF
Modbus address = group byte + offset byte, so C04.11 -> 0x0411.

Only dependency: pyserial   (pip install pyserial)
"""
import sys
import time
import serial

PORT = "/dev/ttyACM0"
BAUD = 115200
SLAVE = 0x01

STATE = {0: "not ready", 1: "ready (servo OFF)", 2: "running (servo ON)", 3: "FAULT"}


def crc16(data: bytes) -> bytes:
    """Modbus CRC-16, returned low byte first."""
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return bytes([crc & 0xFF, crc >> 8])


def transact(ser, pdu: bytes, resp_len: int) -> bytes:
    frame = pdu + crc16(pdu)
    ser.reset_input_buffer()
    ser.write(frame)
    resp = ser.read(resp_len)
    if len(resp) >= 5 and resp[1] & 0x80:
        raise IOError(f"Drive returned error frame: {resp.hex(' ')}")
    if len(resp) != resp_len:
        raise IOError(f"Short/no response ({len(resp)} bytes): {resp.hex(' ')}")
    if crc16(resp[:-2]) != resp[-2:]:
        raise IOError(f"CRC mismatch: {resp.hex(' ')}")
    return resp


def read16(ser, grp: int, off: int) -> int:
    resp = transact(ser, bytes([SLAVE, 0x03, grp, off, 0x00, 0x01]), 7)
    return (resp[3] << 8) | resp[4]


def write16(ser, grp: int, off: int, val: int) -> None:
    pdu = bytes([SLAVE, 0x06, grp, off, (val >> 8) & 0xFF, val & 0xFF])
    resp = transact(ser, pdu, 8)
    if resp[:6] != pdu:
        raise IOError(f"Write echo mismatch: {resp.hex(' ')}")


def main():
    try:
        ser = serial.Serial(PORT, BAUD, bytesize=8, parity="N", stopbits=1, timeout=0.5)
    except serial.SerialException as e:
        sys.exit(f"Cannot open {PORT}: {e}")

    with ser:
        time.sleep(0.2)  # let the USB-serial link settle

        state = read16(ser, 0x41, 0x0A)                       # U41.0A servo status
        print(f"Connected. Servo state before: {state} = {STATE.get(state, '?')}")

        write16(ser, 0x04, 0x11, 0)                           # C04.11 = 0 -> servo OFF
        time.sleep(0.2)

        logic = read16(ser, 0x04, 0x11)
        state = read16(ser, 0x41, 0x0A)
        print(f"C04.11 = {logic}   Servo state after: {state} = {STATE.get(state, '?')}")

        if state == 2:
            print("WARNING: servo still running - check whether DI5 is physically wired/active.")
        elif state == 3:
            print("Drive is in FAULT - check the panel display for the fault code.")
        else:
            print("Servo is disabled.")


if __name__ == "__main__":
    main()