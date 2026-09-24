# CLAUDE.md — Black Boat（RA6M5 无人船固件）

供 Claude Code 使用的项目说明：构建方式、代码边界、外设映射与已知陷阱。
面向人的项目介绍见 `README.md`。

## 项目身份

- **MCU**：Renesas RA6M5（R7FA6M5BF2CBG，Cortex-M33），1 MB ROM / 512 KB RAM
- **FSP**：Renesas Flexible Software Package v6.4.0，经 RASC 生成
- **工具链**：Keil MDK V5.43 + ARMClang V6.24，C17
- **架构**：裸机超级循环，无 RTOS，入口 `hal_entry()`
- **TrustZone**：已配置安全分区（`.secure_azone` / `.secure_rzone` /
  `.secure_xml`），改动安全相关配置需谨慎

## 构建

工程文件 `black_boat.uvprojx`，无 Makefile / CMake。两条路径：

- **Keil GUI**：打开工程后 Build，或
- **命令行**（已实测可用）：

```bash
"D:\keil5\UV4\UV4.exe" -r black_boat.uvprojx -j0 -o build.log
```

`-r` 全量重建，`-j0` 抑制弹窗。退出码 0 = 无错无警，1 = 有警告，2 = 有错误。
编译前会自动调用 `rasc_launcher.bat`：仅当 `configuration.xml` 比
`output.rasc` 新时才真正执行代码生成。

## 目录与代码边界

| 目录 | 性质 | 可否手改 |
| --- | --- | --- |
| `src/`、`src/user/` | 用户代码 | ✅ 唯一可编辑处 |
| `ra_gen/` | RASC 生成 | ❌ 重新生成时被覆盖 |
| `ra_cfg/` | FSP 配置头 | ❌ |
| `ra/` | FSP SDK 源码 | ❌ |
| `RTE/`、`script/`、`via/` | 构建支撑文件 | ❌ |

**改外设的正确路径**：编辑 `configuration.xml`（用 `rasc_launcher.bat` 打开
RASC）→ 生成 → FSP 重写 `ra_gen/` 与 `ra_cfg/`。直接改生成文件无效。
`configuration.xml` 与 `ra_cfg.txt` 是配置的唯一事实源。

## 外设映射

### UART / SCI / DMAC

| 外设 | 引脚 | SCI | DMAC | 波特率 | 回调 |
| --- | --- | --- | --- | --- | --- |
| IMU JY901B | P501(TX)/P502(RX) | SCI5 | DMAC0 (RX) | 115200 | `imu_callback` → `transfer_imu_rx_callback` |
| LoRa 遥控 | P301(RX)/P302(TX) | SCI2 | 无（中断接收） | 115200 | `lora_callback` |
| GPS | P203(RX)/P202(TX) | SCI9 | 无（中断接收） | 9600 | `gps_callback` |
| N10 雷达 | P706(RX)/P707(TX) | SCI3 | DMAC4 (RX) | 230400 | `N10_callback` → `transfer_N10_rx_callback` |
| 4G 模块 | P607(RX)/**PA00**(TX) | SCI8 | 无（中断发送） | 115200 | `G_callback` |

4G 的 TX 是 **PA00**，不是连续的 P 口编号，容易写错。

缓冲区大小定义在 `src/user/uart.h`：IMU 22 B、LoRa 256 B、
N10 58 B（解析为 18 个值）、4G 发送 256 B。

### GPIO

| 引脚 | 用途 | 逻辑 |
| --- | --- | --- |
| P015 | 尾灯 / 模式指示 | `move_flag==1`（摇杆）高电平，`==0`（PID 循线）低电平 |
| P500 | 船箱信号 | 收到 LoRa `CMD_P500` 后保持 1 s 高电平（`p500_hold_count = 200`，5 ms × 200） |

两处均在 `gpt0_callback` 中操作。引脚枚举须用 `BSP_IO_PORT_xx_PIN_yy`，
写成 `IOPORT_PORT_xx_PIN_yy` 会触发 `-Wenum-conversion` 警告。

### DMAC

| 通道 | 用途 | 源 → 目的 | 长度 |
| --- | --- | --- | --- |
| DMAC0 | IMU（SCI5 RXI） | `R_SCI5->RDR` → `imu_rx_buf` | 22 B |
| DMAC2 | 4G TX（预留，未使用） | — | — |
| DMAC4 | N10 雷达（SCI3 RXI） | `R_SCI3->RDR` → `n10_rx_buf` | 58 B |

**DMAC 复位必须走 `set_transfer_length()` +
`set_transfer_dst_src_address()` + `reconfigure()`**，不要用
`open()` + `enable()`——后者会残留旧地址配置。参见
`transfer_imu_rx_callback` 与 `transfer_N10_rx_callback`。

## 时钟

XTAL 24 MHz → PLL（÷3 ×25）= 200 MHz：

| 时钟 | 分频 | 频率 |
| --- | --- | --- |
| ICLK | /1 | 200 MHz |
| PCLKA、PCLKD | /2 | 100 MHz |
| PCLKB、PCLKC、FCLK | /4 | 50 MHz |

## GPT 定时器

| 定时器 | 模式 | 周期 | 用途 |
| --- | --- | --- | --- |
| GPT0 | Periodic | 5 ms | IMU 读取 → PID 计算 → PWM 输出 |
| GPT1 | Periodic | 1 s | 4G 上传调度 |
| GPT6 | Saw-wave PWM | 10 kHz | 左电机（GTIOCA 正转 / GTIOCB 反转） |
| GPT7 | Saw-wave PWM | 10 kHz | 右电机 |

`PWM_MAX = 8000`（周期 10000，即限幅到 80%）。

## 4G 上传时序

GPT1 每秒触发一次，节拍由 `gpt1_callback` 的 `startup` 与 `tick` 控制
（`tick > 4` 归零）：

```text
t = 0..9 s   启动等待（4G 注册）
t = 10 s     发 GPS JSON
t = 11 s     空闲
t = 12 s     发 N10 雷达 JSON
t = 13..14 s 空闲，随后循环
```

若 4G 实际注册慢于 10 s，启动包会丢，需调大 `startup`。
每包独立发送，**不要拼接**——4G 模块会丢数据。

JSON 外壳：`{"id":"123","version":"1.0","params":{ ... }}`

## LoRa 遥控协议

- **命令包**（6 B）：`EE 02 CMD VALUE CRC8 FF`，CRC 覆盖前 4 字节
- **摇杆包**（6 B）：`CC 01 俯仰 02 转向 CRC8`，CRC 覆盖前 5 字节
- CRC8：多项式 `0x31`，初值 `0x00`

| 命令 | 值 | 状态 |
| --- | --- | --- |
| `CMD_LIGHT` | 0x01 | 仅宏定义，未实现 |
| `CMD_PUMP` | 0x02 | 仅宏定义，未实现 |
| `CMD_GIMBAL_UD` / `CMD_GIMBAL_LR` | 0x03 / 0x04 | 仅宏定义，未实现 |
| `CMD_ARM_SERVO` / `CMD_ARM_DUTY` | 0x05 / 0x06 | 仅宏定义，未实现 |
| `CMD_SPEED` | 0x07 | 已实现 → `v`（0–10） |
| `CMD_SWITCH` | 0x08 | 已实现 → `move_flag`（0 = PID 循线，1 = 摇杆） |
| `CMD_P500` | 0x09 | 已实现 → `p500_hold_count = 200` |

摇杆包转向值 → `move`（0 停止 / 1 前进 / 2 后退 / 3 左转 / 4 右转）。

## 控制逻辑

`gpt0_callback` 每 5 ms 执行一次：

```text
move_flag == 0 → PID 循线：PID(陀螺仪) → 差速转向
move_flag == 1 → 摇杆模式：v × 1000 ± PID(陀螺仪) → 左/右 PWM
```

## 已知陷阱

| 现象 | 原因 | 解法 |
| --- | --- | --- |
| GPS 输出为空 | SCI9 波特率需精确匹配 | `BRR=162, CKS=1, BGDM=0` |
| 只收到 `{` | 在 ISR 中阻塞处理 | 移到主循环用标志位 |
| TXI 只触发一次 | 错查了 `IELSR[TXI]` | 只查 `IELSR[SCIx_RXI_IRQn]` |
| 4G 上传不完整 | 单包过长 | GPS 与 N10 分开发送，间隔 1 s |
| `n10_data` 全为 0 | DMAC 用 `open()+enable()` 复位失败 | 改用 `reconfigure()` |
| Duplicate symbol | 同一文件在工程里登记了两次 | 查 `.uvprojx` 的 `<Group>`，删除重复条目 |
| 开串口后立刻进中断 | FSP 不清 IELSR | 初始化后手动 `R_ICU->IELSR[SCIx_RXI_IRQn] = 0U` |

## 源码编码

`uart.c`（UTF-8 带 BOM）、`uart.h`、`gpt.c`（UTF-8 无 BOM）已转码，
其余源文件仍是 **GBK**。编辑 GBK 文件时必须用能保持原编码的工具——
按 UTF-8 读写会把中文注释变成替换字符（曾因此丢失过整段代码）。

## 测试

无自动化测试框架。验证方式：Keil 编译通过 + 烧录实测。

## 约定

- 注释与交流用中文，标识符用英文
- `Objects/`、`Listings/`、`DebugConfig/` 不纳入版本控制
  （`.gitignore` 已覆盖），也不要提交 `ra/`、`ra_gen/`、`ra_cfg/`
