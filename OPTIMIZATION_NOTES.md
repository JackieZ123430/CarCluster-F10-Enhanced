# Optimization and compatibility notes

## Removed vehicle implementations

- BMW E series / E60
- BMW E46
- MINI F55 mode
- VW MQB
- VW PQ25
- VW PQ46
- MQB passthrough

## Retained integrations

- Better_CAN / BeamNG UDP
- SimHub serial JSON
- Forza UDP
- Wi-Fi configuration portal
- HTTP WebDashboard

## 0x26A clue

The source keeps a comment near the F10 basic-drive keep-alive frames stating that `0x26A` is associated with an F-series EHC / ride-height status path. No frame, payload, CRC, counter, or transmission call is included.

## Validation performed

- BetterCANPacket host compile and static offset checks
- Retained C++ modules checked with `-Wall -Wextra -Werror`
- Deleted-model reference scan
- Better_CAN Lua and C++ field-order comparison
