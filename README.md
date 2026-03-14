# CarCluster-F10-Enhanced

**Enhanced GPL-3.0 fork of r00li/CarCluster focused on BMW F10 cluster improvements.**  
**基于 r00li/CarCluster 的 GPL-3.0 增强版分支，主要针对 BMW F10 仪表进行功能改进。**

Forked from / 原项目：  
https://github.com/r00li/CarCluster

Thanks to r00li for the original CarCluster project.  
感谢 r00li 创建原始 CarCluster 项目。

---

## What is this? / 项目简介

CarCluster-F10-Enhanced allows you to control a real BMW F10 instrument cluster using an ESP32 and CAN interface.  
CarCluster-F10-Enhanced 允许使用 ESP32 和 CAN 接口驱动真实 BMW F10 仪表。

This fork introduces multiple improvements and behavior fixes for BMW F-series clusters on top of the original CarCluster project.  
该分支在原始 CarCluster 项目的基础上，对 BMW F 系列仪表进行了多项功能改进和行为优化。

---

![Main image](https://github.com/JackieZ123430/CarCluster-F10-Enhanced/blob/main/Misc/main_display.jpeg?raw=true)
<iframe width="560" height="315" src="https://www.youtube.com/embed/0wwqyG7KJ9c?si=phiJW64uejFUoLku" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

---

## Key Enhancements / 主要改进

Compared to the original CarCluster implementation, this fork introduces a number of functional improvements for BMW F-series clusters.

与原始 CarCluster 项目相比，本分支增加了多项针对 BMW F 系列仪表的功能改进。

Main additions include:  
主要新增功能包括：

- **Auto Hold support**  
  Auto Hold 图标与状态支持

- **Tire puncture / TPMS warning detection**  
  爆胎检测 / 胎压报警提示

- **Door open detection**  
  车门开启检测与警告显示

- **Correct D gear display logic**  
  修复 D 档显示逻辑

- **Neutral (N) gear warning**  
  增加 N 档警告提示

- **Improved tachometer behavior**  
  优化转速表逻辑，使转速变化更加流畅

- **Ignition self-check simulation**  
  模拟点火未启动时的仪表故障灯自检

- **Sport (S) and Manual (M) gear display support**  
  支持 S 档 / M 档显示

- **Cruise control indicator support**  
  增加定速巡航图标显示

- **Engine temperature warning display**  
  增加发动机温度警告提示

- **Web steering wheel button fix**  
  修复原项目 Web 界面方向盘按钮无法正常工作的情况

- **Various additional improvements and behavior fixes**  
  以及其他多项仪表行为优化和功能改进等

---

## Integration with Better_CAN / 与 Better_CAN 集成

This project can optionally work together with:

该项目可以与以下项目配合使用：

https://github.com/JackieZ123430/Better_CAN

Better_CAN provides a telemetry-to-CAN bridge for BeamNG and other simulation environments.

Better_CAN 提供模拟器遥测数据 → CAN 总线的桥接功能。

Architecture overview / 架构示意：


BeamNG → Better_CAN → CarCluster-F10-Enhanced → BMW F10 Cluster


---

## Hardware Requirements / 硬件需求

Same hardware requirements as the original CarCluster project:

与原始 CarCluster 项目相同的硬件需求：

- ESP32 development board  
- MCP2515 CAN module  
- 12V power supply  
- BMW F10 instrument cluster  

---

## Installation / 安装

Installation process remains the same as the upstream project:

安装流程与原项目一致：

1. Install ESP32 support in Arduino IDE  
2. Open `CarCluster.ino`  
3. Select BMW F cluster configuration  
4. Upload to ESP32  
5. Configure network or serial communication

---

## License / 许可证

This project is a derivative work of:

该项目基于以下项目：

https://github.com/r00li/CarCluster

Licensed under **GNU GPL v3**.  
遵循 **GNU GPL v3 开源协议**。

---

## Credits / 项目来源

This project is based on the original **CarCluster** project by r00li.

本项目基于 r00li 创建的 **CarCluster** 项目进行二次开发。

Original project:
https://github.com/r00li/CarCluster

---

## Acknowledgements / 致谢

During development, several open-source projects were referenced for research
on BMW instrument cluster behavior, CAN messaging, and simulator integration.

在开发过程中参考了多个开源项目，用于研究 BMW 仪表 CAN 报文、
仪表行为逻辑以及模拟器集成方案。

Special thanks to the authors of the following projects:


- BMW_6WA_Controller_ESP32 – https://github.com/gizmo87898/BMW_6WA_Controller_ESP32
- BMW_6WA_Controller_BeamNG – https://github.com/gizmo87898/BMW_6WA_Controller_BeamNG
- Bmw-F10-Dash – https://github.com/byte-biter/Bmw-F10-Dash-
- BMW-F20-KOMBI-Cluster-Bench-testing – https://github.com/floeplala/BMW-F20-KOMBI-Cluster-Bench-testing
- G30_Kombi_Controller – https://github.com/gizmo87898/G30_Kombi_Controller
- Bmw-F3x-6wa-cluster-with-simhub-for-beamng – https://github.com/CreeBoom2020/Bmw-F3x-6wa-cluster-with-simhub-for-beamng
- BeamNG-Telemetry – https://github.com/OfficialLambdax/BeamNG-Telemetry
- @ameFISH – code contribution and merge assistance

  
These projects provided useful insights into BMW cluster CAN messaging and
simulation telemetry integration.

这些项目为 BMW 仪表 CAN 报文结构和模拟器遥测集成提供了重要参考。

---

## Technical References / 技术参考

The following documentation resources were used to identify BMW cluster warning
messages and CC-ID error codes.

以下资料用于确认 BMW 仪表报警信息以及 CC-ID 错误代码。

BMW CC-ID error codes reference:

https://www.autobulbsdirect.co.uk/blog/bmw-cc-id-error-codes/

This reference was used to validate cluster warning indicators during
development and testing.

该资料在开发和测试过程中用于验证仪表报警提示逻辑。
