# Black Boat — RA6M5 船控固件

基于 Renesas RA6M5 (R7FA6M5BF2CBG) 的无人船控制系统，使用 Keil MDK + ARMClang V6.24 开发。

## 硬件资源分配

| 外设 | 引脚 | SCI | DMAC | 用途 | 波特率 |
|------|:----:|:---:|:----:|------|:------:|
| **IMU** JY901B | P501(RX)/P502(TX) | SCI5 | DMAC0 (RX) | 姿态角/角速度 | 115200 |
| **LoRa** 无线 | P301(RX)/P302(TX) | SCI2 | — (中断 RX) | 遥控协议解析 | 115200 |
| **GPS** | P203(RX)/P202(TX) | SCI9 | — (中断 RX) | NMEA 定位 | **9600** |
| **N10 雷达** | P706(RX)/P707(TX) | SCI3 | DMAC4 (RX) | 18 点距离探测 | 230400 |
| **4G 数传** | **P607(RX)/P1000(TX)** | SCI8 | — (中断 TX) | 上发 JSON 数据 | 115200 |

## 工程结构

```
black_boat/
├── src/
│   ├── hal_entry.c             # 入口：初始化 + 主循环
│   ├── user/
│   │   ├── headfile.h           # 统一头文件包含
│   │   ├── uart.c / uart.h     # 所有 UART/DMA 通信
│   │   ├── gpt.c / gpt.h       # 定时器 (5ms PID + 1s 4G 上报)
│   │   ├── pwm.c / pwm.h       # 双路 PWM 电机控制
│   │   ├── pid.c / pid.h       # PID 控制器
│   │   ├── gps.c / gps.h       # NMEA RMC 解析
│   │   └── host_computer.c/h   # 上位机通信
│   └── hal_warmstart.c
├── ra_gen/                      # FSP 生成 (勿手动编辑)
├── ra/fsp/                      # FSP 库源码
├── ra_cfg/fsp_cfg/              # FSP 配置头文件
├── configuration.xml            # RASC 工程配置
└── black_boat.uvprojx           # Keil MDK 工程
```

## 通信协议

### LoRa 遥控

**屏包** (6 字节)：
```
EE 02 CMD VALUE CRC8 FF
```
- `CMD_SPEED (0x07)` → 速度 `v` (0-10)
- `CMD_SWITCH (0x08)` → `move_flag` (0=PID自稳, 1=摇杆)

**摇杆包** (7 字节)：
```
CC 01 白方向 02 黑方向 CRC8 33
```
- 黑方向 → `move` (0=停止, 1=前进, 2=后退, 3=左转, 4=右转)

CRC8 多项式 `0x31`。

### 4G 上报 JSON

GPT1 以 **1 秒**周期中断，前 28 秒等待 4G 注网，之后交替发送：

```
t=28s:  发 GPS
t=29s:  发 N10 雷达
t=30s:  跳过（额外等待）
t=31s:  发 GPS → 循环...
```

**GPS 数据：**
```json
{"id":"123","version":"1.0","params":{"black_lat":{"value":31.846245},"black_lon":{"value":117.198970}}}
```

**N10 雷达数据：**
```json
{"id":"123","version":"1.0","params":{"N10":{"value":[N0,N1,...,N17]}}}
```

### GPS NMEA

解析 `$GPRMC` / `$GNRMC`，提取经纬度（十进制 DD.DDDDD）。

## 控制逻辑

### move_flag = 0（纯自稳）
仅 PID 根据陀螺仪角速度修正 PWM，无前进速度。

### move_flag = 1（摇杆模式）
```
左轮 = v×1000 ± PID 转向
右轮 = v×1000 ± PID 转向
```
方向特殊处理：左/右转关闭 PID，后退两轮反转。

### 定时器

| 定时器 | 周期 | 作用 |
|:------:|:----:|------|
| GPT0 | 5ms | IMU → PID → PWM 输出 |
| GPT1 | 1s | 4G 上报调度（28s 启动延迟） |

## 常见问题

| 问题 | 原因 | 解决 |
|------|------|------|
| GPS 数据为空 | 波特率默认 9600 | UART9 BRR=162, CKS=1, BGDM=0 |
| 只发 `{` 后卡死 | ISR 中等 UART 中断 | 主循环标志发送 |
| UART TXI 只进一次 | 误清 IELSR[TXI] | 只清 `IELSR[SCIx_RXI_IRQn]` |
| 4G 收不到数据 | 单次发包太大 | 交替发送，GPS/雷达分开 |
| n10_data 全为 0 | DMAC 回调名不匹配 | 检查 `transfer_N10_rx_callback` |
| 编译 symbol 重复 | 旧 .o 残留 | Rebuild All |

## 开发环境

- IDE: Keil MDK V5.38+
- 编译器: ARMClang V6.24
- FSP: Renesas Flexible Software Package
- MCU: R7FA6M5BF2CBG, 176-pin BGA
- 主频: PCLKA=100MHz, ICLK=200MHz
