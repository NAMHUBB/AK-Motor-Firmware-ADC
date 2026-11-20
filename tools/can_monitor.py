"""Read the board's CAN monitor (Modbus 0x1400) over RS-485; standard library only.

    python3 tools/can_monitor.py /dev/cu.usbserial-110            # what the board has seen
    python3 tools/can_monitor.py /dev/cu.usbserial-110 --scan     # ask every motor ID, then show
    python3 tools/can_monitor.py /dev/cu.usbserial-110 --watch    # refresh every second

The scan sends only disable frames (servo 0x0F00|id for IDs 1-255, MIT "exit motor
mode" for IDs 1-127) and runs only while every motor is OFF. Close the console first:
it holds the port.
"""
import argparse
import os
import select
import termios
import time

BASE = 0x1400
REGISTERS = 82
START_SCAN, CLEAR = 1, 2
ERRORS = {0: "none", 1: "stuff", 2: "form", 3: "no ACK", 4: "bit recessive", 5: "bit dominant",
          6: "CRC", 7: "software", 8: "bus-off", 9: "error passive", 10: "error warning"}
LEC = ["none", "stuff", "form", "no ACK", "bit recessive", "bit dominant", "CRC", "software"]
SCAN = {0: "not run", 1: "running", 2: "done", 3: "stopped (a motor was switched on)"}


def crc16(data):
    crc = 0xffff
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ (0xa001 if crc & 1 else 0)
    return crc


def frame(data):
    return bytes(data) + crc16(data).to_bytes(2, "little")


def open_port(path):
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    attrs = termios.tcgetattr(fd)
    attrs[0] = attrs[1] = attrs[3] = 0
    attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
    attrs[4] = attrs[5] = termios.B115200
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    return fd


def transact(fd, request, expected, timeout=0.3):
    termios.tcflush(fd, termios.TCIFLUSH)
    os.write(fd, request)
    data, end = b"", time.time() + timeout
    while time.time() < end and len(data) < expected:
        if select.select([fd], [], [], 0.02)[0]:
            data += os.read(fd, 512)
    if len(data) != expected or crc16(data[:-2]) != int.from_bytes(data[-2:], "little"):
        return None
    return data


def command(fd, slave, value):
    request = frame([slave, 6, BASE >> 8, BASE & 0xff, 0, value])
    return transact(fd, request, 8) == request


def read(fd, slave):
    reply = transact(fd, frame([slave, 4, BASE >> 8, BASE & 0xff, 0, REGISTERS]), 5 + REGISTERS * 2)
    if not reply or reply[0] != slave:
        return None
    return [int.from_bytes(reply[3 + 2 * i:5 + 2 * i], "big") for i in range(REGISTERS)]


def meaning(extended, can_id, data):
    if extended and can_id >> 8 == 0x29:
        kind = "disable ack" if len(data) == 8 and data[7] == 0x77 else "status"
        return f"servo-mode reply from motor CAN ID {can_id & 0xff} ({kind})"
    if extended:
        return f"servo-mode command (mode 0x{can_id >> 8:02X}) to CAN ID {can_id & 0xff}, from another master"
    if data:
        return f"standard ID: MIT-mode frame; a motor reply carries its ID in data[0] = {data[0]}"
    return "standard ID, no data"


def show(words):
    if words[0] != 0x434D:
        print("no CAN monitor in this firmware (magic missing)")
        return
    frames = words[2] << 16 | words[3]
    flags = words[8]
    state = "bus-off" if flags & 4 else "error passive" if flags & 2 else "error warning" if flags & 1 else "error active"
    print(f"frames received {frames}  bus errors {words[4]} (last: {ERRORS.get(words[5], words[5])})")
    print(f"controller {state}  TEC {words[6]}  REC {words[7]}  last LEC {LEC[flags >> 4 & 7]}")
    print(f"ID scan: {SCAN.get(words[9] >> 12, '?')} (next ID {words[9] & 0xfff})")
    rows = []
    for i in range(8):
        at = 10 + i * 9
        if not words[at] & 0x4000:
            continue
        extended = bool(words[at] & 0x8000)
        can_id = (words[at] & 0x1fff) << 16 | words[at + 1]
        dlc = words[at + 4]
        data = b"".join(w.to_bytes(2, "big") for w in words[at + 5:at + 9])[:dlc]
        rows.append((extended, can_id, words[at + 2], words[at + 3], data))
    if not rows:
        print("no frames from any other node")
    for extended, can_id, count, age, data in rows:
        ident = f"0x{can_id:08X}" if extended else f"0x{can_id:03X}    "
        print(f"  {'EXT' if extended else 'STD'} {ident}  x{count:<5} {age:>5} ms ago  "
              f"[{data.hex(' ')}]  {meaning(extended, can_id, data)}")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("port")
    parser.add_argument("--slave", type=int, default=1, help="any motor's slave address (default 1)")
    parser.add_argument("--scan", action="store_true", help="clear, ask every motor ID, then show")
    parser.add_argument("--clear", action="store_true", help="clear the table and counters")
    parser.add_argument("--watch", action="store_true", help="refresh every second")
    args = parser.parse_args()
    fd = open_port(args.port)
    try:
        if args.clear or args.scan:
            if not command(fd, args.slave, CLEAR):
                raise SystemExit("no reply to the monitor command (firmware without CAN monitor?)")
        if args.scan:
            command(fd, args.slave, START_SCAN)
            end = time.time() + 5
            while time.time() < end:
                words = read(fd, args.slave)
                if words and words[9] >> 12 in (2, 3):
                    break
                time.sleep(0.1)
            time.sleep(0.2)  # late replies
        while True:
            words = read(fd, args.slave)
            if words is None:
                print(f"slave {args.slave}: no reply")
            else:
                show(words)
            if not args.watch:
                break
            time.sleep(1)
            print()
    finally:
        os.close(fd)


if __name__ == "__main__":
    main()
