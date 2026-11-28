#!/usr/bin/env python3
"""Read or persist RH56DFX serial settings with the hand driver stopped."""

import argparse

import serial

READ = 0x11
WRITE = 0x12
HAND_ID = 1000
BAUDRATE = 1002
SAVE = 1005
BAUD_CODES = {115200: 0, 57600: 1, 19200: 2}


def device_id(text):
    value = int(text)
    if not 1 <= value <= 254:
        raise argparse.ArgumentTypeError("ID must be in the range 1-254")
    return value


def receive(port, hand_id, op, address):
    frame = port.read(9)
    header = bytes([0x90, 0xEB, hand_id, 4, op]) + address.to_bytes(2, "little")
    if len(frame) != 9 or frame[:7] != header or frame[8] != sum(frame[2:8]) % 256:
        raise RuntimeError(
            f"Expected a valid reply from ID {hand_id} for register {address}, "
            f"received {frame.hex(' ') or 'no bytes'}"
        )
    if op == WRITE and frame[7] != 1:
        raise RuntimeError(f"ID {hand_id} rejected the write to register {address}")
    return frame[7]


def request(port, hand_id, op, address, value, *, reply_id=None):
    body = bytes([hand_id, 4, op]) + address.to_bytes(2, "little") + bytes([value])
    port.write(b"\xeb\x90" + body + bytes([sum(body) % 256]))
    return receive(port, hand_id if reply_id is None else reply_id, op, address)


def configure(port, args):
    hand_id = args.hand_id
    port.reset_input_buffer()
    if request(port, hand_id, READ, HAND_ID, 1) != hand_id:
        raise RuntimeError("Reported hand ID does not match the addressed device")
    if args.new_hand_id is None and args.new_baudrate is None:
        code = request(port, hand_id, READ, BAUDRATE, 1)
        rates = {code: rate for rate, code in BAUD_CODES.items()}
        if code not in rates:
            raise RuntimeError(f"Unrecognized baud rate register value {code}")
        print(
            f"ID {hand_id}, active baud rate {port.baudrate}, configured baud rate {rates[code]}"
        )
        return

    if args.new_hand_id is not None and args.new_hand_id != hand_id:
        # The acknowledgement and all subsequent replies carry the new ID.
        request(
            port, hand_id, WRITE, HAND_ID, args.new_hand_id, reply_id=args.new_hand_id
        )
        hand_id = args.new_hand_id
        print(f"Active hand ID is now {hand_id}.")
    if args.new_baudrate is not None:
        request(port, hand_id, WRITE, BAUDRATE, BAUD_CODES[args.new_baudrate])

    request(port, hand_id, WRITE, SAVE, 1)
    if receive(port, hand_id, READ, SAVE) != 0:
        raise RuntimeError("The hand did not confirm that saving completed")
    print(f"Settings saved for ID {hand_id}.")
    if args.new_baudrate is not None:
        print(f"Power-cycle the hand to use {args.new_baudrate} baud.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--device", required=True, help="serial port connected to one hand"
    )
    parser.add_argument(
        "--baudrate",
        type=int,
        choices=BAUD_CODES,
        default=115200,
        help="current baud rate (default: 115200)",
    )
    parser.add_argument(
        "--hand-id", type=device_id, default=1, help="current device ID (default: 1)"
    )
    parser.add_argument(
        "--new-baudrate",
        type=int,
        choices=BAUD_CODES,
        help="baud rate after saving and power cycling",
    )
    parser.add_argument(
        "--new-hand-id", type=device_id, help="new device ID (takes effect immediately)"
    )
    args = parser.parse_args()
    try:
        with serial.Serial(
            args.device, args.baudrate, timeout=3, write_timeout=1, exclusive=True
        ) as port:
            configure(port, args)
    except (OSError, RuntimeError) as exc:
        parser.exit(1, f"{parser.prog}: {exc}\n")


if __name__ == "__main__":
    main()
