"""Robotiq RTU bench client. Without --port, only print the request (no hardware IO)."""
import argparse
import time


def crc16(data):
    crc = 0xffff
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ (0xa001 if crc & 1 else 0)
    return crc


def frame(data):
    return bytes(data) + crc16(data).to_bytes(2, "little")


def request(args):
    if args.command == "status":
        return frame([args.slave, 4, 7, 0xd0, 0, 3])
    actions = {"reset": 0, "activate": 1, "stop": 1, "move": 9,
               "release-open": 0x31, "release-close": 0x11}
    return frame([args.slave, 0x10, 3, 0xe8, 0, 3, 6, actions[args.command],
                  0, 0, args.position, args.speed, args.force])


def transact(port, req):
    port.reset_input_buffer()
    port.write(req)
    port.flush()
    head = port.read(3)
    if len(head) != 3:
        raise RuntimeError("No complete response header")
    remaining = (head[2] + 2) if head[1] in (3, 4, 0x17) else (2 if head[1] & 0x80 else 5)
    response = head + port.read(remaining)
    if len(response) != 3 + remaining or crc16(response[:-2]) != int.from_bytes(response[-2:], "little"):
        raise RuntimeError("Truncated response or CRC mismatch")
    if response[:2] != req[:2]:
        raise RuntimeError("Unexpected address/function: " + response.hex(" "))
    if req[1] == 0x10 and response[:6] != req[:6]:
        raise RuntimeError("Write acknowledgement does not match request")
    print("RX", response.hex(" "))
    if response[1] in (3, 4) and response[2] >= 6:
        s = response[3:9]
        print(f"gACT={s[0]&1} gSTA={(s[0]>>4)&3} gGTO={(s[0]>>3)&1} "
              f"gOBJ={s[0]>>6} gFLT=0x{s[2]:02x} echo={s[3]} position={s[4]} current={s[5]*10}mA")
    return response


def byte(value):
    value = int(value, 0)
    if not 0 <= value <= 255:
        raise argparse.ArgumentTypeError("Expected 0..255")
    return value


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("command", choices=["status", "reset", "activate", "stop", "move", "release-open", "release-close"])
    p.add_argument("--port", help="e.g. COM7; omitted = dry run")
    p.add_argument("--slave", type=byte, default=9)
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--position", type=byte, default=0)
    p.add_argument("--speed", type=byte, default=0)
    p.add_argument("--force", type=byte, default=0)
    p.add_argument("--poll-seconds", type=float, default=0, help="Read status at 50Hz to keep the host watchdog alive")
    args = p.parse_args()
    if not 1 <= args.slave <= 247 or args.poll_seconds < 0:
        p.error("slave must be 1..247 and poll-seconds must be nonnegative")
    req = request(args)
    print("TX", req.hex(" "))
    if not args.port:
        return
    try:
        import serial
    except ImportError:
        p.error("Serial IO requires: python -m pip install pyserial")
    with serial.Serial(args.port, args.baud, timeout=0.2, write_timeout=0.2) as port:
        transact(port, req)
        deadline = time.monotonic() + args.poll_seconds
        while time.monotonic() < deadline:
            time.sleep(0.02)
            transact(port, frame([args.slave, 4, 7, 0xd0, 0, 3]))


if __name__ == "__main__":
    main()
