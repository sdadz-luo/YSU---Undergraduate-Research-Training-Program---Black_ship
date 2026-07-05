# AGENTS.md — Black Boat (RA6M5 无人船固件)

## 项目身份

Bare-metal embedded C 固件，运行于 **Renesas RA6M5 (R7FA6M5BF2CBG, Cortex-M33)**, 无 RTOS。  
IDE: **Keil MDK V5.38+**，编译器 **ARMClang V6.24**。  
FSP: **Renesas Flexible Software Package v6.4.0**，通过 RASC (Renesas Advanced Smart Configurator) 生成代码。

> **注意**：仓库已配置 TrustZone 安全分区 (`secure_azone` / `secure_rzone` / `secure_xml`)，操作安全相关配置时需谨慎。

## 构建

- **只支持 Keil MDK**：项目文件 `black_boat.uvprojx`，双击或在 Keil µVision 中打开。
- 无 Makefile / CMakeLists.txt。命令行构建可用 `UV4.exe -b black_boat.uvprojx`（需 Keil 安装路径在 PATH）。
- `rasc_launcher.bat` 启动 RASC 图形化外设配置工具，在下次构建时自动重新生成 `ra_gen/` / `ra_cfg/` 等目录。

## 关键目录

| 目录 | 性质 | 说明 |
|------|------|------|
| `src/` | **手写代码** | 唯一可手动编辑的源代码目录 |
| `src/user/` | **手写代码** | 所有应用层逻辑 (UART/PWM/GPT/PID/GPS) |
| `src/hal_entry.c` | 入口 | `hal_entry()` = main loop |
| `ra_gen/` | **自动生成** | FSP 生成，**不要手动编辑**，RASC 重新生成时会覆盖 |
| `ra_cfg/` | **自动生成** | FSP 配置头文件，同上 |
| `ra/fsp/` | **FSP SDK** | 外部库，由 RASC 管理版本，已 gitignore |
| `RTE/` | **运行时环境** | Keil RTE 文件，已 gitignore |
| `script/` | **自动生成** | 链接脚本，已 gitignore |

## Rebuild 规则

- `configuration.xml` + `ra_cfg.txt` = RASC 配置的真实来源。编辑 `ra_gen/hal_data.c` 是无效的——RASC 会覆盖。
- 如果需要添加/修改外设：启动 RASC (双击 `rasc_launcher.bat`)，在 GUI 中配置，保存后 FSP 自动重新生成代码。
- **Duplicate symbol 错误** → 删除 `Objects/` 目录中残留的 `.o` 文件后 Rebuild All。

## LSP

通过 `.opencode/lsp.json` 配置了 **clangd** (`.c/.cpp/.h/.hpp`)。  
`compile_flags.txt` 是 clangd 的编译数据库替代——如需添加 include 路径或宏定义，修改此文件。

## 外设与 UART 映射

| 外设 | 芯片引脚 | SCI | DMAC | 波特率 | 中断回调 |
|------|----------|-----|------|--------|----------|
| **IMU** JY901B (陀螺仪/加速度) | P501(TX)/P502(RX) | **SCI5** | DMAC0 (RX) | **115200** | `imu_callback` → `transfer_imu_rx_callback` (DMAC) |
| **LoRa** (遥控) | P301(RX)/P302(TX) | **SCI2** | 无 (中断 RX) | **115200** | `lora_callback` (中断内解析协议) |
| **GPS** | P203(RX)/P202(TX) | **SCI9** | 无 (中断 RX) | **9600** | `gps_callback` (按 `\n` 分割 NMEA) |
| **N10 雷达** | P706(RX)/P707(TX) | **SCI3** | DMAC4 (RX) | **230400** | N10_callback → `transfer_N10_rx_callback` (DMAC) |
| **4G 模块** | P607(RX)/**PA00**(TX) | **SCI8** | 无 (中断 TX) | **115200** | `G_callback` (仅 TX_COMPLETE) |

注意 4G 的 TX 脚是 PA00（不是 P 口连续编号），容易写错。

### DMAC 通道

| 通道 | 用途 | 源地址 | 目的地址 | 长度 |
|------|------|--------|----------|------|
| DMAC0 | IMU (SCI5 RXI) | `R_SCI5->RDR` | `imu_rx_buf` | 22 字节 |
| DMAC2 | 4G TX (预留，当前未使用) | — | — | — |
| DMAC4 | N10 雷达 (SCI3 RXI) | `R_SCI3->RDR` | `n10_rx_buf` | 58 字节 |

DMAC 完成后回调中需要**手动重置** (reconfigure)。参考 `transfer_imu_rx_callback` 和 `transfer_N10_rx_callback` 的写法。

## 时钟树

- XTAL: 24MHz → PLL (÷3 × 25) → **ICLK = 200MHz**
- PCLKA = 100MHz, PCLKB = 50MHz, PCLKC = 50MHz, FCLK = 50MHz

## GPT 定时器

| 定时器 | 模式 | 周期 | 用途 |
|--------|------|------|------|
| GPT0 | Periodic | **5 ms** | IMU 读取 → PID 计算 → PWM 输出 |
| GPT1 | Periodic | **1 s** | 4G 数据上传调度 |
| GPT6 | Saw-wave PWM | **10 kHz** (周期=10000) | 左电机正反转 (GTIOCA=正转, GTIOCB=反转) |
| GPT7 | Saw-wave PWM | **10 kHz** (周期=10000) | 右电机正反转 |

PWM 最大值 `PWM_MAX = 8000`(周期 = 10000, 实际 duty 限制到 80%)。

## 4G 上传时序 (GPT1 回调)

```
t =  0-27s: 启动等待 (28 秒 4G 注册)
t = 28s:    发 GPS JSON
t = 29s:    发 N10 雷达 JSON
t = 30s:    空闲 (等待 1s)
t = 31s:    重复 (GPS → N10 → 空闲 循环)
```

JSON 格式：每个包独立发送（不要拼接，否则 4G 模块会丢数据）。

## LoRa 遥控协议

- **命令包** (6 字节): `EE 02 CMD VALUE CRC8 FF`
  - `CMD_SPEED (0x07)` → `v` (速度 0-10)
  - `CMD_SWITCH (0x08)` → `move_flag` (0=PID 循线, 1=摇杆模式)
- **摇杆包** (7 字节): `CC 01 俯仰 02 转向 CRC8 33`
  - 转向 → `move` (0=停止, 1=前进, 2=后退, 3=左转, 4=右转)
- CRC8 多项式: `0x31`

## 控制逻辑 (`gpt0_callback`, 5ms)

```
if move_flag == 0: PID 循线模式 — PID(陀螺仪) → 差速转向
if move_flag == 1: 摇杆模式 — v * 1000 ± PID(陀螺仪) → 左/右 PWM
```

## 已知陷阱 (来自经验)

| 现象 | 原因 | 修复 |
|------|------|------|
| GPS 输出为空 | UART9 默认 9600 波特率需精确匹配 | `BRR=162, CKS=1, BGDM=0` |
| 只输出 `{` | UART 中断处理在 ISR 中阻塞 | 用主循环标志位 |
| UART TXI 只触发一次 | 错误地检查了 `IELSR[TXI]` | 只检查 `IELSR[SCIx_RXI_IRQn]` |
| 4G 上传不完整 | 单次发送 JSON 太长 | GPS 和 N10 分开发送（间隔 1s） |
| `n10_data` 全为 0 | DMAC 回调未正确重置 | 确认 `transfer_N10_rx_callback` 中 **reconfigure** |
| Duplicate symbol | 切换分支后 .o 文件残留 | 删除 `Objects/` 目录后 Rebuild All |
| `R_ICU->IELSR` 清零 | FSP 启动时不会清零 | 在 UART init 后手动清: `R_ICU->IELSR[SCIx_RXI_IRQn] = 0U` |

## 测试

**无测试框架。** 这是一个没有自动化测试的嵌入式固件项目。修改后需通过 Keil 编译 + 硬件烧录验证。

## 其他

- `REAMD.md` (文件名不是 README.md——这是已知拼写错误，但有价值的内容在里面)
- C 标准: C17 (`-std=c17`)
- 注释和回复用中文，标识符用英文
- 仓库的 `.gitignore` 会忽略 `ra/`、`ra_gen/`、`ra_cfg/`、`RTE/`、`script/`、`via/`、`Objects/`、`Listings/`
