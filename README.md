# CarCluster-F10-Enhanced — F10 Optimized Build

这是面向 **BMW F10 / F 系列仪表硬件环境** 的精简版本。项目保留实际使用的输入链路，并删除 BMW E 系列、BMW E46、MINI、VW MQB、VW PQ25、VW PQ46 和 MQB Passthrough 等其他车型适配。

- Enhanced project: https://github.com/JackieZ123430/CarCluster-F10-Enhanced
- Better_CAN protocol: https://github.com/JackieZ123430/Better_CAN
- Original project: https://github.com/r00li/CarCluster

## 保留的功能

- BMW F10 仪表 CAN 输出
- BeamNG + `Better_CAN.lua` UDP 协议
- SimHub 串口 JSON 协议
- Forza Horizon / Forza Motorsport UDP
- Wi-Fi 配网门户
- HTTP Web 仪表盘
- MCP2515 CAN 接口

## 主要改动

1. **Better_CAN 完整适配**
   - UDP 端口统一为 `4444`
   - 接收端严格匹配 Better_CAN 的 `148` 字节数据结构
   - 增加字段偏移和结构大小的编译期检查
   - 修复旧接收端缺少字段、把单字节输入错误声明为 `float` 所造成的数据错位

2. **修复燃油单位问题**
   - Better_CAN、SimHub 和 WebDashboard 统一使用 `0–100%`
   - 旧代码按 `0.0–1.0` 截断，容易把正常燃油值当成满油

3. **修复上次移除 ID 后的编译问题**
   - 删除无实现的 `sendFixedLIM()` 调用和声明
   - 删除未使用的 ACC/旧巡航链路
   - 修复时间发送代码中未定义的 `hours` / `minutes`

4. **0x26A 处理**
   - 不发送 `0x26A`
   - 在 `BMWFSeriesCluster.cpp` 内保留 EHC / 车身高度状态链路线索
   - 不公开载荷、计数器或 CRC 映射

5. **体积与运行优化**
   - 删除其他车型源码和无关库
   - PlatformIO 只编译 F10、Better_CAN、Forza、SimHub、Wi-Fi 和 WebDashboard 模块
   - 关闭 WebDashboard HTTPS，仅保留 HTTP，减少固件体积
   - 主循环加入 ESP32 task yield
   - 清理串口超长 JSON 和无效 CAN ID
   - 移除旧 F15 专用逻辑和错误的“ESC 持续介入后自动强制 DSC OFF”逻辑

## Better_CAN 安装

项目内提供：

```text
BeamNG/Better_CAN.lua
```

打开文件并修改：

```lua
local targetAddress = "192.168.31.61"
```

把它改成 ESP32 连接 Wi-Fi 后在串口监视器中显示的 IP 地址。发送端口保持：

```text
4444
```

## 默认网络端口

| 功能 | 端口 |
|---|---:|
| Better_CAN / BeamNG UDP | 4444 |
| Forza UDP | 1101 |
| WebDashboard HTTP | 80 |

## 编译

推荐使用 Visual Studio Code + PlatformIO：

```bash
pio run
pio run -t upload
pio device monitor -b 921600
```

默认硬件参数：

- ESP32 Dev Module
- MCP2515
- CAN：500 kbit/s
- MCP2515 晶振：8 MHz
- CS：GPIO 5
- INT：GPIO 2

晶振不是 8 MHz 时，需要修改 `CarCluster/CarCluster.ino` 中的 `MCP_8MHZ`。

## 目录说明

```text
CarCluster/                 ESP32 主程序
CarCluster/src/Clusters/    仅保留 BMW_F
CarCluster/src/Games/       Better_CAN、Forza、SimHub
CarCluster/src/Other/       Wi-Fi 与 WebDashboard
BeamNG/Better_CAN.lua       BeamNG UDP 发送端
Misc/                       F10 接线和使用资料
```

## 协议兼容规则

Better_CAN UDP 字段不得随意删除或重排。即使某些字段固定输出 `0`，仍然必须保留其字节位置，否则后续字段会整体错位。

当前接收结构：

- 总长度：`148 bytes`
- `cruiseControlTarget` 偏移：`52`
- `airspeedKmh` 偏移：`76`
- `brakeOverHeatFL` 偏移：`100`
- `taillightInnerR` 偏移：`145`

## License

固件项目继续遵循仓库内的 **GNU GPL v3**。Better_CAN Lua 文件保留其文件头内单独声明的使用条件。原项目作者与第三方库版权信息不得删除。
