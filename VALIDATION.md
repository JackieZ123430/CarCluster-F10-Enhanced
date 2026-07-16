# Validation report

Completed checks:

- Main sketch host syntax check
- BMW F10 cluster module strict host syntax check
- Better_CAN receiver strict host syntax check
- SimHub module strict host syntax check
- Forza module strict host syntax check
- WebDashboard module strict host syntax check
- CRC module strict host syntax check
- Warnings treated as errors (`-Wall -Wextra -Werror`)
- Better_CAN Lua/C++ field count: 105 / 105
- Better_CAN Lua/C++ field order: exact match
- BetterCANPacket size assertion: 148 bytes
- `0x26A` transmission scan: no send call present
- Vehicle implementation scan: only `BMW_F` remains

A complete PlatformIO firmware build was attempted, but this execution environment could not finish downloading the `espressif32` platform/toolchain. The source-level checks above passed; perform `pio run` on a machine with the ESP32 PlatformIO toolchain installed before flashing hardware.
