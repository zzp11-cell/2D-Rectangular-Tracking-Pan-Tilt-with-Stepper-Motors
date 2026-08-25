# Yuntai1 双轴云台视觉追踪

基于 K230 + STM32F103 的双轴云台自动追踪项目。K230 通过摄像头识别矩形目标，把像素偏移通过 UART 发给 STM32；STM32 运行 PID 位置伺服，通过两路闭环步进电机驱动（Emm_V5.0）控制云台 X/Y 轴，让激光点始终对准目标中心。

## 系统架构

```text
                  ┌─────────────┐
   摄像头 ──────► │   K230 视觉  │
                  │ 矩形检测/追踪 │
                  └──────┬──────┘
                         │ UART2 (115200, 8N1)
                         │ +xxxx+yyyy\n / SCAN / HOME / SAVE
                         ▼
                  ┌─────────────┐
                  │  STM32F103   │  PID 位置伺服
                  │  主控制器    │  超时释放 / 扫描状态机
                  └──────┬──────┘
                         │ USART1 (Emm_V5 串口协议)
                         ▼
                  ┌─────────────┐
                  │ X/Y 闭环步进 │  X 轴左右、Y 轴上下
                  │  电机驱动    │
                  └─────────────┘
```

## 硬件

- K230 庐山派 LCKFB（含摄像头和 800x480 ST7701 LCD）
- STM32F103C8T6 主控
- 2 路 Emm_V5.0 闭环步进电机驱动（X 轴地址 1，Y 轴地址 2）
- K230 USER 按键（GPIO53）触发自动扫描
- K230 GPIO33 控制激光笔，锁定后自动点亮

## 目录结构

```text
Yuntai1/
├─ 25e_k230_cv/    # K230 视觉代码（矩形检测 + UART 发送）
├─ BSP/            # Emm_V5 电机驱动库（第三方）
├─ Core/           # STM32 主程序（main.c、HAL 配置）
├─ Drivers/        # STM32 HAL / CMSIS 驱动
├─ MDK-ARM/        # Keil MDK 工程
└─ this.ioc        # STM32CubeMX 配置
```

## 通信协议

K230 → STM32，USART2，115200 8N1：

- 位置数据：`+xxxx+yyyy\n`，固定 10 字符，X/Y 像素偏移同时到达，例如 `+0123-0045\n`
- 目标丢失：`+0000+0000\n`
- 按键命令：`SCAN\n`（触发自动扫描找目标）
- 附加命令：`HOME\n`（双轴回零）、`SAVE\n`（保存当前原点）

STM32 → 电机驱动，USART1，使用 Emm_V5.0 串口协议（地址 + 命令字 + 校验字节），详见 `BSP/Emm_V5.c`。

## 功能

- K230 端 Canny 边缘检测 + `approxPolyDP` 四边形筛选，对角线交点作为目标中心，EMA 平滑抑制抖动
- STM32 端 X/Y 双轴 PID 位置伺服，死区判断、积分限幅、单帧脉冲限幅
- 自动扫描状态机：`IDLE → STARTING → SWEEPING → LOCKING → LOCKED`，按下 USER 键即可让云台自行转圈找目标并锁定
- K230 自判中心重合后拉高 GPIO33 点亮激光，目标丢失自动熄灭
- 追踪超时自动释放电机，防止长时间堵转
- 激光笔与镜头纵向视差可通过 `Y_TARGET_OFFSET` 补偿

## 编译与烧录

### STM32

1. 用 Keil MDK 打开 `MDK-ARM/this.uvprojx`
2. 编译后下载到 STM32F103C8T6
3. 工程由 STM32CubeMX（`this.ioc`）生成，改引脚/时钟后在 CubeMX 重新生成即可

### K230

1. 把 `25e_k230_cv/25e_k230_cv.py` 复制到 K230
2. 用 CanMV / K230 IDE 运行，或复制到 TF 卡脱机运行
3. 按 K230 USER 键发送 `SCAN`，云台开始自动搜索并锁定矩形目标

## 主要调参位置

均在 `Core/Src/main.c` 顶部宏定义：

| 参数 | 说明 |
| --- | --- |
| `X_KP` / `Y_KP` | 像素误差比例系数，追不上调大，振荡调小 |
| `X_DEAD_ZONE` / `Y_DEAD_ZONE` | 死区像素，越小越跟手 |
| `PULSE_PER_PIXEL` | 像素转脉冲系数，需实测标定 |
| `POS_SPEED_RPM` | X 轴位置模式速度 |
| `Y_POS_SPEED_RPM` / `Y_FOC_mA` | Y 轴重载速度与闭环电流，抬不动时调整 |
| `Y_TARGET_OFFSET` | 激光/镜头纵向视差补偿像素 |
| `LOST_TIMEOUT_MS` | 追踪丢失后释放电机超时 |

## 注意事项

- `BSP/Emm_V5.h` 为第三方闭环步进电机库，版权归原作者所有
- 代码中参数为起步值，实际机械结构不同时需要重新标定
- K230 与 STM32 的串口波特率、引脚接线需一致，详见双方代码顶部注释
