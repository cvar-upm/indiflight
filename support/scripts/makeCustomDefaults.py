#!/usr/bin/env python3
"""Turn the CLI part of configs/boards and configs/profiles files into the
custom defaults text that is linked into the firmware (.custom_defaults).

The firmware applies it on the CLI command `defaults` (and `defaults nosave`),
see resetConfigToCustomDefaults() in src/main/cli/cli.c. `defaults bare`
still gives the plain firmware defaults.
"""

from argparse import ArgumentParser

# must match CUSTOM_DEFAULTS_START_PREFIX in src/main/cli/cli.c
HEADER_PREFIX = "# Betaflight"

# Commands that make no sense while the firmware replays its own defaults:
# batch/save/defaults would recurse or reboot, the identity ones are compiled
# in from the BOARD_NAME/MANUFACTURER_ID build flags or are per-chip
SKIPPED_COMMANDS = {
    "batch", "defaults", "save", "exit",
    "board_name", "manufacturer_id", "mcu_id", "signature",
}

parser = ArgumentParser(description=__doc__)
parser.add_argument("output", help="custom defaults text file to write")
parser.add_argument("--target", required=True, help="target name for the header line")
parser.add_argument("config_files", nargs="+", help="board/profile files, in order")
args = parser.parse_args()

lines = [f"{HEADER_PREFIX} / {args.target} custom defaults from {' '.join(args.config_files)}"]
for path in args.config_files:
    with open(path) as f:
        for line in f:
            # "#define" build flags and comments are both dropped here
            line = line.split("#")[0].strip()
            if not line or line.split()[0] in SKIPPED_COMMANDS:
                continue
            lines.append(line)

text = "\n".join(lines) + "\n"
# the linker fails with "region FLASH_CUSTOM_DEFAULTS overflowed" if it does not fit
size = len(text.encode()) + 1  # NUL terminator appended by the assembler stub

print(f"custom defaults: {len(lines) - 1} commands, {size} bytes")

# Only touch the file when it changes, so make relinks only then
try:
    with open(args.output) as f:
        if f.read() == text:
            raise SystemExit(0)
except FileNotFoundError:
    pass

with open(args.output, "w") as f:
    f.write(text)
