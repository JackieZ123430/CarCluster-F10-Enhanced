# CarCluster-F10-Enhanced

**Enhanced GPL-3.0 fork of r00li/CarCluster focused on BMW F10 cluster improvements.**

Forked from:
https://github.com/r00li/CarCluster

Thanks to r00li for the original CarCluster project.

---

## What is this?

CarCluster-F10-Enhanced allows you to control a real BMW F10 instrument cluster using an ESP32 and CAN interface.

This fork introduces a rewritten BMW F10 adaptation layer and extended LCD functionality while preserving compatibility with the original project structure.

---

![Main image](https://github.com/r00li/CarCluster/blob/main/Misc/main_display.jpg?raw=true)

---

## Key Enhancements

This fork specifically focuses on:

- Rewritten F10-specific CAN adaptation
- Extended LCD message handling
- Refined gear mapping logic
- Improved stability and alert behavior
- Structural cleanup of BMW F cluster logic

---

## Integration with Better_CAN

This project is designed to optionally work together with:

https://github.com/JackieZ123430/Better_CAN

Better_CAN provides an extensible telemetry-to-CAN bridge for BeamNG and other simulation environments.

Architecture overview:

BeamNG → Better_CAN → CarCluster-F10-Enhanced → BMW F10 Cluster

Better_CAN is a separate project and is licensed independently.

---

## Differences from the Original Project

Compared to upstream CarCluster:

- F10 adaptation logic has been refactored
- LCD message handling has been extended
- CAN mapping logic refined for improved control
- F10 behavior tuned for desk simulation scenarios

Other cluster platforms remain unchanged unless explicitly modified.

---

## Hardware Requirements

Same hardware requirements as the original CarCluster:

- ESP32 development board
- MCP2515 CAN module
- 12V power supply
- Supported BMW F10 cluster
- Wiring and optional fuel simulation components

Refer to upstream documentation for wiring and setup details.

---

## Installation

Installation process remains identical to the upstream project:

1. Install ESP32 support in Arduino IDE
2. Open `CarCluster.ino`
3. Select BMW F cluster in configuration
4. Upload to ESP32
5. Configure network or serial connection

---

## License

This project is a derivative work of:
https://github.com/r00li/CarCluster

It remains licensed under GNU GPL v3.
See the LICENSE file for details.
