# Black Boat — RA6M5 船控固件

基于 Renesas RA6M5 (R7FA6M5BF2CBG) 的无人船控制系统，使用 Keil MDK + ARMClang V6.24 开发。

## 硬件资源分配

| 外设 | SCI | DMAC | 用途 | 波特率 |
|------|:---:|:----:|------|:------:|
| **IMU** JY901B | SCI5 | DMAC0 (RX) | 姿态角/角速度 | 115200 |
| **LoRa** 无线 | SCI2 | — (中断 RX) | 遥控协议解析 | 115200 |
| **GPS** | SCI9 | — (中断 RX) | NMEA 定位 | **9600** |
| **N10 雷达** | SCI3 | DMAC4 (RX) | 18 点距离探测 | 230400 |
| **4G 数传** | SCI8 | — (中断 TX) | 上发 JSON 数据 | 115200 |

## 工程结构

```
black_boat/
├── src/
│   ├── hal_entry.c             # 入口：初始化 + 主循环
│   ├── user/
│   │   ├── headfile.h           # 统一头文件包含
│   │   ├── uart.c / uart.h     # 所有 UART/DMA 通信
│   │   ├── gpt.c / gpt.h       # 定时器控制 (5ms PID + 4s 上报)
│   │   ├── pwm.c / pwm.h       # 双路 PWM 电机控制
│   │   ├── pid.c / pid.h       # PID 控制器
│   │   ├── gps.c / gps.h       # NMEA RMC 解析
│   │   └── host_computer.c/h   # 上位机通信
│   ├── hal_entry.c
│   └── hal_warmstart.c
├── ra_gen/                      # FSP 生成 (勿手动编辑)
│   ├── hal_data.c / hal_data.h
│   ├── pin_data.c
│   ├── vector_data.c / vector_data.h
│   └── ...
├── ra/fsp/                      # FSP 库源码
├── ra_cfg/fsp_cfg/              # FSP 配置头文件
├── configuration.xml            # RASC 工程配置
└── black_boat.uvprojx           # Keil MDK 工程
```

## 通信协议

### LoRa 遥控协议

**屏包** (6 字节) — 速度/开关：
```
EE 02 CMD VALUE CRC8 FF
```
- `CMD_SPEED (0x07)` → 更新速度 `v` (0-10)
- `CMD_SWITCH (0x08)` → 更新 `move_flag` (0=PID自稳, 1=摇杆模式)

**摇杆包** (7 字节)：
```
CC 01 白方向 02 黑方向 CRC8 33
```
- 黑方向 → 更新 `move` (0=停止, 1=前进, 2=后退, 3=左转, 4=右转)

CRC8 多项式 `0x31`。

### 4G 上报 JSON (每 4 秒)

```json
{"id":"123","version":"1.0","params":{"N10":{"value":[N0,N1,...,N17]}}}
{"id":"123","version":"1.0","params":{"black_lat":{"value":31.846245}}}
{"id":"123","version":"1.0","params":{"black_lon":{"value":117.198970}}}
```

三条连续发出，每条阻塞等待发送完成。

### GPS NMEA

解析 `$GPRMC` / `$GNRMC` 语句，提取经纬度（十进制）。

## 控制逻辑

### move_flag = 0（纯自稳）
仅 PID 根据陀螺仪角速度修正 PWM，无前进速度。

### move_flag = 1（摇杆模式）
```
左轮 = 速度 v × 1000 - PID 转向量
右轮 = 速度 v × 1000 + PID 转向量
```
方向会覆盖速度方向：
- 左转：左轮反转，关闭 PID
- 右转：右轮反转，关闭 PID
- 后退：两轮反转

### 定时器
| 定时器 | 周期 | 作用 |
|:------:|:----:|------|
| GPT0 | 5ms | IMU 读取 → PID 计算 → PWM 输出 |
| GPT1 | 4s | 置发送标志，主循环上报 4G |

## 常见问题

| 问题 | 原因 | 解决 |
|------|------|------|
| GPS 数据为空 | 波特率默认 9600 | `hal_data.c` 中 UART9 BRR=162, CKS=1, BGDM=0 |
| 只发 `{` 后卡死 | 在 ISR 中等 UART 中断 | 改用主循环标志发送 |
| UART TXI 只进一次 | 误清 IELSR[TXI] | 只清 `IELSR[SCIx_RXI_IRQn]` |
| n10_data 全为 0 | DMAC 回调名不匹配 | 检查 `hal_data.h` 中 `transfer_N10_rx_callback` |
| 编译报 symbol 重复 | 旧 .o 残留 | Rebuild All |

## 开发环境

- IDE: Keil MDK V5.38+
- 编译器: ARMClang V6.24
- FSP: Renesas Flexible Software Package
- MCU: R7FA6M5BF2CBG, 176-pin BGA
- 主频: PCLKA=100MHz, ICLK=200MHz
