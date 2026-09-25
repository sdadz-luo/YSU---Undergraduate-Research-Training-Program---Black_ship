# Black Boat — RA6M5 无人船固件

燕山大学大学生创新创业训练计划「无人船」项目的船体控制固件。基于
Renesas RA6M5（Cortex-M33）裸机开发，实现 IMU 姿态解算、LoRa 遥控解析、
GPS 定位、N10 毫米波雷达探测与 4G 数据回传。

## 项目组成

燕山大学大创「无人船」项目由上位机遥控双船协同作业，共三个仓库：

- **Black_ship**（本仓库）—— 黑船固件：双模式运动控制、毫米波雷达与 4G 回传
- [White_ship][white-ship] —— 白船固件：水质、温湿度等多传感器采集与 4G 回传
- [Host-Computer][host-computer] —— 上位机：串口屏与双摇杆指令，经 LoRa 转发双船

[white-ship]: https://github.com/sdadz-luo/YSU---Undergraduate-Research-Training-Program---White_ship
[host-computer]: https://github.com/sdadz-luo/YSU---Undergraduate-Research-Training-Program---Unmanned-Ship---Host-Computer

## 硬件平台

| 组件 | 接口 | 用途 |
| --- | --- | --- |
| Renesas RA6M5（R7FA6M5BF2CBG） | — | 主控，1 MB ROM / 512 KB RAM |
| IMU JY901B | SCI5 + DMAC0 | 陀螺仪 / 加速度，供转向 PID 使用 |
| LoRa 模块 | SCI2 | 接收遥控器指令 |
| GPS 模块 | SCI9 | NMEA 定位（9600 波特率） |
| N10 毫米波雷达 | SCI3 + DMAC4 | 18 点距离探测 |
| 4G 模块 | SCI8 | JSON 数据上云 |
| 双电机 + 驱动 | GPT6 / GPT7 | 10 kHz PWM，支持正反转 |

## 功能

- **双模式运动控制**，控制周期 5 ms
  - PID 循线：以陀螺仪角度为反馈做差速转向
  - 摇杆遥控：遥控速度叠加陀螺仪修正
- **LoRa 遥控协议**：带 CRC8 校验的命令包与摇杆包
- **4G 数据回传**：GPS 与雷达数据交替上传
- **状态指示**：尾灯随控制模式切换；船箱信号由遥控指令触发

## 目录结构

```text
black_boat/
├── src/                    用户代码（唯一可编辑区）
│   ├── hal_entry.c         入口：初始化后进入主循环
│   ├── hal_warmstart.c     BSP 温启动钩子
│   └── user/               应用层模块
│       ├── uart.c/h        UART + DMAC 通信（5 路）
│       ├── gpt.c/h         定时器：5 ms 控制 + 1 s 上报调度
│       ├── pwm.c/h         双路电机 PWM
│       ├── pid.c/h         PID 控制器
│       ├── gps.c/h         NMEA RMC 语句解析
│       └── host_computer.c/h   占位，尚未实现
├── ra_gen/                 RASC 生成代码（勿手改）
├── ra_cfg/                 FSP 配置头（勿手改）
├── ra/                     FSP SDK
├── configuration.xml       RASC 配置源文件
└── black_boat.uvprojx      Keil MDK 工程
```

## 通信协议

### LoRa 遥控

命令包（6 字节）：

```text
EE 02 CMD VALUE CRC8 FF
```

| 命令 | 值 | 含义 |
| --- | --- | --- |
| `CMD_SPEED` | 0x07 | 速度 0–10 |
| `CMD_SWITCH` | 0x08 | 0 = PID 循线，1 = 摇杆模式 |
| `CMD_P500` | 0x09 | 触发船箱信号（P500 保持 1 s 高电平） |

摇杆包（6 字节）：

```text
CC 01 俯仰值 02 转向值 CRC8
```

转向值：0 停止 / 1 前进 / 2 后退 / 3 左转 / 4 右转。
CRC8 多项式 `0x31`，初值 `0x00`。

### 4G 上报

每包独立发送，拼包会导致 4G 模块丢数据：

```json
{"id":"123","version":"1.0","params":{"black_lat":{"value":31.846245},"black_lon":{"value":117.198970}}}
{"id":"123","version":"1.0","params":{"N10":{"value":[0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17]}}}
```

## 构建

1. 用 Keil MDK V5.38+ 打开 `black_boat.uvprojx`，Build；或走命令行：

```bash
"D:\keil5\UV4\UV4.exe" -r black_boat.uvprojx -j0 -o build.log
```

2. 通过 J-Link 烧录到目标板

修改引脚 / 外设 / 时钟后，需先用 `rasc_launcher.bat` 打开 RASC 重新生成代码。

## 开发环境

- Keil MDK-ARM V5.43 + ARMClang V6.24
- Renesas FSP v6.4.0
- C 标准 C17
