"""Export the STM32 ELF32 load segments to BIN and Intel HEX; standard library only."""
import argparse
import hashlib
import json
from pathlib import Path
import struct


def export(elf_path, out_dir):
    data = Path(elf_path).read_bytes()
    if data[:6] != b"\x7fELF\x01\x01" or struct.unpack_from("<H", data, 18)[0] != 40:
        raise ValueError("Expected little-endian ARM ELF32")
    phoff = struct.unpack_from("<I", data, 28)[0]
    phsize, phnum = struct.unpack_from("<HH", data, 42)
    base, limit = 0x08000000, 0x08100000  # STM32F407VG (Clicker 4): 1 MiB Flash
    segments = []
    for index in range(phnum):
        kind, offset, _, physical, size, *_ = struct.unpack_from("<8I", data, phoff + index * phsize)
        if kind == 1 and size and base <= physical < limit:
            if physical + size > limit or offset + size > len(data):
                raise ValueError("ELF load segment exceeds image bounds")
            segments.append((physical, data[offset:offset + size]))
    if not segments:
        raise ValueError("No flash load segments found")
    image = bytearray(b"\xff" * (max(a + len(b) for a, b in segments) - base))
    for address, chunk in segments:
        image[address - base:address - base + len(chunk)] = chunk
    stack, reset = struct.unpack_from("<II", image)
    if not (0x20000000 <= stack <= 0x20020000 and base <= (reset & ~1) < limit and reset & 1):
        raise ValueError("Invalid STM32 vector table")
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    prefix = out / "ak80_9_robotiq"
    prefix.with_suffix(".elf").write_bytes(data)
    prefix.with_suffix(".bin").write_bytes(image)

    def record(address, kind, payload):
        raw = bytes([len(payload), address >> 8, address & 255, kind]) + payload
        return ":" + (raw + bytes([-sum(raw) & 255])).hex().upper()

    lines, upper = [], None
    for offset in range(0, len(image), 16):
        address = base + offset
        if address >> 16 != upper:
            upper = address >> 16
            lines.append(record(0, 4, upper.to_bytes(2, "big")))
        lines.append(record(address & 0xffff, 0, image[offset:offset + 16]))
    lines.append(record(0, 1, b""))
    prefix.with_suffix(".hex").write_text("\n".join(lines) + "\n", encoding="ascii")
    result = {"flash_base": hex(base), "binary_bytes": len(image),
              "initial_sp": hex(stack), "reset_vector": hex(reset),
              "files": {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                        for p in (prefix.with_suffix(s) for s in (".elf", ".bin", ".hex"))}}
    (out / "manifest.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("elf", nargs="?", default="target/thumbv7em-none-eabi/release/stm-gripper-controller")
    parser.add_argument("--out", default="output/firmware")
    args = parser.parse_args()
    export(args.elf, args.out)
