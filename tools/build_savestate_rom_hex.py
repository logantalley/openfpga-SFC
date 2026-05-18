#!/usr/bin/env python3
"""Convert savestates.bin into a Verilog $readmemh-compatible .hex file.

Output format: one 16-bit hex word per line, little-endian byte order
(LSB byte first), matching how the firmware would be stored in the
core's SDRAM (which is read as 16-bit words with `ROM_Q[7:0]` being
the byte at the even address).

Usage:
    python tools/build_savestate_rom_hex.py

Re-run this script whenever src/assembly/savestates.asm changes (i.e.,
whenever dist/Assets/snes/common/savestates.bin is regenerated).
"""

from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
BIN_PATH  = REPO_ROOT / "dist" / "Assets" / "snes" / "common" / "savestates.bin"
HEX_PATH  = REPO_ROOT / "src" / "fpga" / "core" / "rtl" / "savestate_rom.hex"

ROM_WORDS = 2048   # 4 KB / 16 bits = 2048 entries; covers the 3107-byte firmware

def main() -> None:
    data = BIN_PATH.read_bytes()
    if len(data) > ROM_WORDS * 2:
        raise SystemExit(
            f"savestates.bin is {len(data)} bytes, exceeds ROM capacity of {ROM_WORDS*2} bytes. "
            f"Increase ROM_WORDS in this script and the BRAM module."
        )
    # Pad with $00 so $readmemh fills the whole memory.
    padded = data.ljust(ROM_WORDS * 2, b"\x00")

    lines = []
    for i in range(ROM_WORDS):
        lo = padded[2 * i]
        hi = padded[2 * i + 1]
        word = (hi << 8) | lo
        lines.append(f"{word:04x}")

    HEX_PATH.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"Wrote {HEX_PATH}  ({len(data)} input bytes, {ROM_WORDS} words emitted)")

if __name__ == "__main__":
    main()
