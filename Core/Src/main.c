/**
 * @file    main.c  (K230 矩形追踪 + 双轴步进电机 PID 位置伺服)
 * @brief   接收 K230 的像素偏移指令, PID控制器驱动双轴云台追踪矩形中心
 *          USART1→电机驱动器  USART2←K230
 *          协议: +xxxx+yyyy\n  (10字符固定长, X/Y 同时到达,
 *                 符号+4位数字+符号+4位数字+换行)
 *
 *  优化日志:
 *  - 固定长数据包 [+xxxx+yyyy\n] 替代分行 DX:/DY:，X/Y原子同步
 *  - 修正包长: K230 发 %+04d%+04d = 10 字符(+4位+号+4位), 非旧注释的 8 字符
 *  - Set_PWM 发送去重 (last-value cache)
 *  - Y 轴目标偏移可调 (Y_TARGET_OFFSET)
 *  - 加速度提升
 *  - PID 在主循环由 track_new 事件驱动, dt 固定 PID_INTERVAL_MS
 */
#include "main.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

/* ---- 电机控制库 ---- */
#include "Emm_V5.h"

/* ==================== 宏定义 ==================== */
#define MOTOR1_ADDR   1      /* 左右电机 (X轴) */
#define MOTOR2_ADDR   2      /* 上下电机 (Y轴) */

/* ========== X轴 (电机1, 左右) PID控制 ==========
   位置模式: 单帧脉冲 = KP × 误差(像素) × PULSE_PER_PIXEL.
   调参直觉: 追过头/振荡 → 降 KP 或加 KD; 追不上/慢 → 加 KP.
   位置模式视觉伺服通常纯P即可, D放大视觉噪声易抖, 宁小勿大. */
#define X_KP           0.3f    /* 比例: 0.3×8=2.4 脉冲/像素 起步, 待标定 */
#define X_KI           0.0f    /* 积分先关, 稳了再开 */
#define X_KD           0.0f    /* 微分先关, 抑制振荡再开(小值如0.05) */
#define X_DEAD_ZONE    8       /* 像素死区 */
#define X_INTEGRAL_MAX 300.0f  /* 积分限幅 */
#define X_INVERT       1       /* 裸脉冲实测: +脉冲=CCW(逆/左), -脉冲=CW(顺/右).
                                   DX>0(矩形在右)需云台右转=CW=-脉冲, 故取反 → 1.
                                   双轴连发曾因 QPos_Control 共享 static cmd[] 被
                                   DMA 异步覆盖导致 X 反/Y 丢, 现已改双缓冲区修复. */

/* ========== Y轴 (电机2, 上下) PID控制 ========== */
#define Y_KP           0.5f
#define Y_KI           0.0f
#define Y_KD           0.0f
#define Y_DEAD_ZONE    8
#define Y_INTEGRAL_MAX 300.0f
#define Y_INVERT       0
/* Y轴目标偏移 (像素): 补偿激光笔与镜头的纵向视差.
   物理结构 Y轴→K230→激光笔(激光笔在镜头正上方), 激光点落在矩形中心上方.
   实测标定: 取正值会让激光点更偏上(方向反), 故取负值.
   标定: 激光点在矩形中心上方→|值|调大(更负); 在下方→调小; 重合即可. */
#define Y_TARGET_OFFSET -25      /* Y轴目标偏移 (像素), 起步值, 待实测标定 */

/* ========== 位置模式 (快速位置 FC) 参数 ==========
   ZDT 闭环步进: 默认细分16, 3200脉冲=1圈. 直驱云台: 电机转1圈=云台转360°.
   视觉伺服每帧: PID误差(像素) × PULSE_PER_PIXEL → 脉冲数, 相对当前位置走一小步.
   PULSE_PER_PIXEL 需实测标定: 追过头→调小, 追不上→调大. 初值8. */
#define PULSE_PER_PIXEL   1      /* 像素→脉冲换算系数 (待标定) */
#define POS_SPEED_RPM     300    /* X轴位置模式速度(RPM): 300仍偏慢, 提到600加快跟踪 */
#define POS_ACC           150    /* X轴加速度档位: 配合提速, 200起停快但不至于摇摆 */

/* ========== Y轴重载补偿 (加电池+开发板后抬不动) ==========
   步进扭矩来源: 闭环FOC电流(FOC_mA) + 低速(高速扭矩下降) + 低加速度(避免加速矩抢重力矩额度).
   PID不增扭, 只改方向/距离; 抬不动是扭矩<重力矩, 改这三处. */
#define Y_POS_SPEED_RPM   120    /* Y重载: 100~150, 步进RPM越高输出扭矩越小 */
#define Y_POS_ACC         60     /* Y重载: 50~80, acc越大起步冲击越大, 重载易堵转 */
#define Y_FOC_mA          3000   /* Y轴闭环电流(mA): X42S/Y42上限5000, 重载3500, 抬不动再加/发烫再降 */
#define POS_MAX_PULSE     3200   /* 单帧脉冲数上限(=1/4圈), 防止单帧冲太远 */

#define RX_BUF_SIZE    32
#define LOST_TIMEOUT_MS       2000

/* PID 定时间隔 (主循环事件驱动时作为固定 dt, ms) */
#define PID_INTERVAL_MS  33      /* ~30Hz, 跟K230发送频率匹配 */

/* ========== USER键扫描找矩形模式 ==========
   K230 发 SCAN 触发, STM32 自行扫描:
   X 匀速 CW 转 360° 找矩形 + Y 三角波上下小扫;
   看到矩形且进画面 → Stop_Now 切 PID 精调到中心死区 → LOCKED.
   激光由 K230 GPIO33 自判 (STM32 不发反馈, UART 单向). */
#define SCAN_X_RPM            30      /* X 扫描速度, 慢转便于 K230 抓帧 (30RPM≈2s/圈) */
#define SCAN_X_ACC            30      /* X 扫描加速度, 小避免起步冲过 */
#define SCAN_X_FULL_TURN_MS   2500    /* X 转满360°超时 (30RPM≈2s/圈 + 余量) */
#define SCAN_Y_RANGE_PULSE    200     /* Y 小扫幅度 ±200脉冲=±22.5° (先小后大, 稳了加到400) */
#define SCAN_Y_PERIOD_MS      600     /* Y 三角波半周期 (上下各600ms) */
#define SCAN_Y_VEL_RPM        60      /* Y 小扫速度, 低于PID的120, 重载稳 */
#define SCAN_Y_ACC            60      /* 同 Y_POS_ACC, 重载稳 */
#define SCAN_DETECT_THR       200     /* 目标进画面判定阈值 (像素, 距画面中心) */
#define SCAN_SEEN_FRAMES      3       /* 连续看到且进画面的帧数才切锁定 */
#define LOCK_CONFIRM_FRAMES   4       /* 死区内连续帧数才算锁定 */
#define SCAN_LOCK_TIMEOUT_MS  3000    /* LOCKING 超时, 放弃回 SWEEPING 重扫 */

/* ==================== 全局变量 ==================== */
uint8_t  rx_byte = 0;
char     rx_buf[RX_BUF_SIZE];
uint8_t  rx_idx = 0;

volatile int16_t  track_x = 0;     /* 最新 X 偏移 */
volatile int16_t  track_y = 0;     /* 最新 Y 偏移 */
volatile bool     track_new = false; /* 收到新数据 */
volatile bool     track_lost = false;/* 是否丢失追踪 */
volatile bool     home_req = false;
volatile bool     save_req = false;
volatile bool     scan_req = false;      /* K230 发 SCAN 触发扫描 */

volatile bool uart1_tx_busy = false;

volatile bool     motor_enabled = false;
volatile uint32_t last_track_ms = 0;

/* ---- PID 控制器状态 ---- */
static float x_integral = 0.0f, x_prev_err = 0.0f, x_d_prev = 0.0f;
static float y_integral = 0.0f, y_prev_err = 0.0f, y_d_prev = 0.0f;

/* ---- 扫描状态机 ----
   IDLE→STARTING(使能+启动)→SWEEPING(X匀速转+Y三角波)→LOCKING(Stop_Now+PID精调到死区)→LOCKED(停, K230发激光)
   扫描期间屏蔽主循环PID, Process_DX/Process_DY 只在 LOCKING/LOCKED 态由 Scan_Tick 内部调用. */
typedef enum { SCAN_IDLE, SCAN_STARTING, SCAN_SWEEPING, SCAN_LOCKING, SCAN_LOCKED } scan_state_t;
volatile scan_state_t scan_state = SCAN_IDLE;
volatile bool scan_failed = false;
static uint32_t scan_start_ms = 0;       /* 扫描起始时刻 (判 X 转满一圈超时) */
static uint32_t scan_y_base_ms = 0;      /* Y 三角波节拍基准时刻 */
static uint32_t scan_lock_deadline = 0; /* LOCKING 超时时刻 */
static int8_t   scan_y_dir = 0;          /* Y 当前方向 +1/-1 */
static uint8_t  seen_count = 0;          /* SWEEPING 连续看到+进画面帧数 */
static uint8_t  lock_confirm_count = 0;  /* LOCKING 连续在死区帧数 */

/* ==================== 函数声明 ==================== */
void SystemClock_Config(void);
static void Motor_Init(void);
static void Process_DX(int32_t offset);
static void Process_DY(int32_t offset);
static void Do_Home(void);
static void Do_SaveOrigin(void);
static void Motor_Disable(void);
static void Motor_Enable(void);
static void wait_uart1_free(void);
static void scan_enter_starting(void);
static void scan_enter_locking(void);
static void scan_enter_locked(bool failed);
static void scan_enter_idle(void);
static void Scan_Tick(void);

/* 去重缓存: 上次发送的脉冲数(位置模式), -1 表示尚未发过 */
static int16_t last_x_pulse = -1;
static int16_t last_y_pulse = -1;

/* 通用: PID 计算 + 位置脉冲指令下发 (快速位置模式 FC)
   电机走完指定脉冲自动停, 无惯性空转, 适合视觉伺服. */
static inline void Motor_Pos_Out(uint8_t addr,
                                 int16_t *last_pulse,
                                 float *integral, float *prev_err,
                                 float *d_prev,
                                 float kp, float ki, float kd,
                                 float integral_max, float dead_zone,
                                 float max_pulse,
                                 float bias,
                                 uint8_t invert,
                                 float error, float dt)
{
    float p_term, i_term, d_term;
    int32_t out_pulse;

    /* PID 计算 */
    p_term = kp * error;

    /* 过零清积分 */
    if ((error > 0.0f && *prev_err < 0.0f) || (error < 0.0f && *prev_err > 0.0f)) {
        *integral = 0.0f;
    }
    if (fabs(error) > dead_zone) {
        *integral += ki * error * dt;
    }
    if (*integral >  integral_max) *integral =  integral_max;
    if (*integral < -integral_max) *integral = -integral_max;
    i_term = *integral;

    /* 微分 + EMA滤波 */
    if (dt > 0.001f) {
        float raw_d = (error - *prev_err) / dt;
        *d_prev = 0.8f * raw_d + 0.2f * (*d_prev);
        d_term = kd * (*d_prev);
    } else {
        d_term = 0.0f;
    }
    *prev_err = error;

    float output_val = p_term + i_term + d_term + bias;

    /* 输出限幅 (脉冲数上限) */
    if (output_val >  max_pulse) output_val =  max_pulse;
    if (output_val < -max_pulse) output_val = -max_pulse;

    /* 死区：误差在死区内 → 走0脉冲(不发命令, 电机停着等) + 清积分 */
    if (fabs(error) <= dead_zone) {
        *integral = 0.0f;
        out_pulse = 0;
    } else {
        out_pulse = (int32_t)(output_val * PULSE_PER_PIXEL);
    }

    /* 像素误差→脉冲换算后再次限幅 */
    if (out_pulse >  (int32_t)(POS_MAX_PULSE)) out_pulse =  (int32_t)POS_MAX_PULSE;
    if (out_pulse < -(int32_t)(POS_MAX_PULSE)) out_pulse = -(int32_t)POS_MAX_PULSE;

    /* 方向反转: invert=1 则脉冲取反 */
    if (invert) out_pulse = -out_pulse;

    /* 位置模式伺服: 死区外每帧都发位置命令(不去重), 保证目标一变化电机立即动,
       无延迟. 死区内(out_pulse=0)不发, 避免持续刷0脉冲命令. */
    *last_pulse = out_pulse;
    if (out_pulse == 0) {
        return;
    }

    /* 发快速位置命令 FC: clk 有符号int32, 正CW负CCW */
    Emm_V5_QPos_Control(addr, out_pulse);
}

/* ==================== X轴 PID 控制 ==================== */
static void Process_DX(int32_t offset)
{
    float error = (float)offset;
    float dt    = (float)PID_INTERVAL_MS * 0.001f;

    Motor_Pos_Out(MOTOR1_ADDR,
                  &last_x_pulse,
                  &x_integral, &x_prev_err, &x_d_prev,
                  X_KP, X_KI, X_KD,
                  X_INTEGRAL_MAX, (float)X_DEAD_ZONE,
                  (float)(POS_MAX_PULSE / PULSE_PER_PIXEL), 0.0f, X_INVERT,
                  error, dt);
}

/* ==================== Y轴 PID 控制 ==================== */
static void Process_DY(int32_t offset)
{
    /* 减 Y_TARGET_OFFSET: offset=Y当前位置误差, 减偏移后就是相对目标偏移的误差 */
    float error = (float)(offset - Y_TARGET_OFFSET);
    float dt    = (float)PID_INTERVAL_MS * 0.001f;

    Motor_Pos_Out(MOTOR2_ADDR,
                  &last_y_pulse,
                  &y_integral, &y_prev_err, &y_d_prev,
                  Y_KP, Y_KI, Y_KD,
                  Y_INTEGRAL_MAX, (float)Y_DEAD_ZONE,
                  (float)(POS_MAX_PULSE / PULSE_PER_PIXEL), 0.0f, Y_INVERT,
                  error, dt);
}

/* ==================== USART1 TX 串行化 ====================
   Emm_V5_* 除 QPos_Control 外自身不等 DMA, 全靠调用方串行化.
   复用 Do_Home/Motor_Enable 里的 while(uart1_tx_busy&&--timeout) 模式. */
static void wait_uart1_free(void)
{
    uint32_t t = 100;
    while (uart1_tx_busy && --t) {
        HAL_Delay(1);
    }
    uart1_tx_busy = true;   /* 预占, 调用方紧接着发 Emm_V5_* */
}

/* ==================== 扫描状态机: 进入各态 ==================== */

/* SCAN_STARTING → SCAN_SWEEPING
   使能电机 + X 匀速 CW 转 + Y 第一次往 + 方向扫. */
static void scan_enter_starting(void)
{
    if (!motor_enabled) {
        Motor_Enable();   /* 阻塞一次, 内部 HAL_Delay(200)+参数设置 */
    }
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);   /* 扫描中 LED 常亮 */

    scan_start_ms   = HAL_GetTick();
    scan_y_base_ms  = scan_start_ms;
    seen_count      = 0;
    scan_failed     = false;

    /* X 匀速 CW: dir=0(CW), 立即执行. 慢转便于 K230 抓帧. */
    wait_uart1_free();
    Emm_V5_Vel_Control(MOTOR1_ADDR, 0, SCAN_X_RPM, SCAN_X_ACC, 0);
    HAL_Delay(10);

    /* Y 第一步: 设扫描用慢速参数 + 往 + 方向走 SCAN_Y_RANGE_PULSE. */
    scan_y_dir = +1;
    wait_uart1_free();
    Emm_V5_Set_QPos_Params(MOTOR2_ADDR, SCAN_Y_VEL_RPM, SCAN_Y_ACC, 2, 0);
    HAL_Delay(10);
    wait_uart1_free();
    Emm_V5_QPos_Control(MOTOR2_ADDR, SCAN_Y_RANGE_PULSE);

    scan_state = SCAN_SWEEPING;
}

/* SWEEPING → LOCKING
   找到矩形: 急停两轴 → 收敛后再切位置模式参数 → 复位 PID 状态.
   关键: Stop_Now 后必须 HAL_Delay(20) 让速度环收敛到 0 再切位置模式,
   否则电机会先完成匀速运动再切, 冲过. */
static void scan_enter_locking(void)
{
    /* 1. 急停两轴 */
    wait_uart1_free();
    Emm_V5_Stop_Now(MOTOR1_ADDR, 0);
    HAL_Delay(10);
    wait_uart1_free();
    Emm_V5_Stop_Now(MOTOR2_ADDR, 0);
    HAL_Delay(20);   /* 速度环收敛余量 */

    /* 2. 重设快速位置模式参数 (从匀速模式切回位置模式, raF=2 相对实时位置) */
    wait_uart1_free();
    Emm_V5_Set_QPos_Params(MOTOR1_ADDR, POS_SPEED_RPM, POS_ACC, 2, 0);
    HAL_Delay(10);
    wait_uart1_free();
    Emm_V5_Set_QPos_Params(MOTOR2_ADDR, Y_POS_SPEED_RPM, Y_POS_ACC, 2, 0);
    HAL_Delay(10);

    /* 3. 复位 PID 状态和去重缓存, 避免扫描期间累积污染 */
    x_integral = y_integral = 0.0f;
    x_prev_err = y_prev_err = 0.0f;
    x_d_prev   = y_d_prev   = 0.0f;
    last_x_pulse = last_y_pulse = -1;
    lock_confirm_count = 0;
    scan_lock_deadline = HAL_GetTick() + SCAN_LOCK_TIMEOUT_MS;

    scan_state = SCAN_LOCKING;
}

/* LOCKING → LOCKED (或失败回 IDLE)
   失败时彻底停转; 正常锁定时电机已在死区停转, 这里只切态. */
static void scan_enter_locked(bool failed)
{
    scan_failed = failed;
    wait_uart1_free();
    Emm_V5_Stop_Now(MOTOR1_ADDR, 0);
    HAL_Delay(10);
    wait_uart1_free();
    Emm_V5_Stop_Now(MOTOR2_ADDR, 0);
    HAL_Delay(10);
    /* 进入 LOCKED: K230 自判中心重合后拉高 GPIO33 发激光, 持续亮直到丢失.
       STM32 在 LOCKED 态继续 PID 维持中心 (见 Scan_Tick). */
    scan_state = SCAN_LOCKED;
}

/* → SCAN_IDLE: 退出扫描, 回正常 track/超时释放流程 */
static void scan_enter_idle(void)
{
    scan_state = SCAN_IDLE;
    last_track_ms = HAL_GetTick();   /* 刷新, 避免立刻被 LOST_TIMEOUT_MS 释放 */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);   /* 退出扫描 LED 熄灭 */
}

/* ==================== 扫描状态机: 主循环每帧 tick ==================== */
static void Scan_Tick(void)
{
    switch (scan_state) {
    case SCAN_IDLE:
        /* 不做事, 让现有 home/save/track_new/超时释放正常跑 */
        break;

    case SCAN_STARTING:
        scan_enter_starting();   /* 完成后置 SCAN_SWEEPING */
        break;

    case SCAN_SWEEPING: {
        /* (1) X 转满一圈没找到 → 失败回 IDLE (不反向退, 避免线缆再扭一层) */
        if (HAL_GetTick() - scan_start_ms > SCAN_X_FULL_TURN_MS) {
            scan_enter_locked(true);
            scan_enter_idle();   /* 失败不保持 LOCKED, 直接回 IDLE */
            return;
        }
        /* (2) Y 三角波: 按节拍奇偶切方向, 方向变时发一次 QPos_Control */
        uint32_t phase = (HAL_GetTick() - scan_y_base_ms) / SCAN_Y_PERIOD_MS;
        int8_t new_dir = (phase % 2 == 0) ? +1 : -1;
        if (new_dir != scan_y_dir) {
            scan_y_dir = new_dir;
            wait_uart1_free();
            Emm_V5_QPos_Control(MOTOR2_ADDR, SCAN_Y_RANGE_PULSE * scan_y_dir);
        }
        /* (3) 看到矩形且进画面 → 连续 SCAN_SEEN_FRAMES 帧切 LOCKING
               注意: track_new 在本态消费置 false, 不走主循环末尾 PID. */
        if (track_new) {
            track_new = false;
            if (!track_lost &&
                (int32_t)track_x > -SCAN_DETECT_THR && (int32_t)track_x < SCAN_DETECT_THR &&
                (int32_t)track_y > -SCAN_DETECT_THR && (int32_t)track_y < SCAN_DETECT_THR) {
                if (++seen_count >= SCAN_SEEN_FRAMES) {
                    scan_enter_locking();
                }
            } else {
                seen_count = 0;
            }
        }
        break;
    }

    case SCAN_LOCKING: {
        /* (1) 锁定超时 → 重新扫 */
        if (HAL_GetTick() > scan_lock_deadline) {
            scan_enter_starting();
            return;
        }
        /* (2) track_new 时判死区, 在死区连续 LOCK_CONFIRM_FRAMES 帧 → LOCKED */
        if (track_new) {
            track_new = false;
            if (track_lost) {
                lock_confirm_count = 0;
            } else {
                int32_t dx = track_x;
                int32_t dy = (int32_t)track_y - Y_TARGET_OFFSET;
                if (dx > -X_DEAD_ZONE && dx < X_DEAD_ZONE &&
                    dy > -Y_DEAD_ZONE && dy < Y_DEAD_ZONE) {
                    if (++lock_confirm_count >= LOCK_CONFIRM_FRAMES) {
                        scan_enter_locked(false);
                    }
                } else {
                    /* 不在死区, 继续 PID 精调到中心 */
                    lock_confirm_count = 0;
                    Process_DX((int32_t)track_x);
                    Process_DY((int32_t)track_y);
                }
            }
        }
        break;
    }

    case SCAN_LOCKED: {
        /* 继续微调维持中心: 目标小幅移动时激光点跟随 (死区8px, 视觉重合即可) */
        if (track_new) {
            track_new = false;
            if (!track_lost) {
                Process_DX((int32_t)track_x);
                Process_DY((int32_t)track_y);
            }
        }
        /* 丢失/偏移的激光灭逻辑由 K230 自己管 (GPIO33), STM32 保持 LOCKED 态 */
        break;
    }

    default:
        scan_state = SCAN_IDLE;
        break;
    }
}

/* ==================== USART1 TX DMA 完成回调 ==================== */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        uart1_tx_busy = false;
    }
}

/* ==================== USART2 中断回调 ==================== */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

        if (rx_byte == '\n' || rx_byte == '\r') {
            if (rx_idx == 10) {   /* +xxxx+yyyy = 10字符 (符号+4位 ×2) */
                rx_buf[rx_idx] = '\0';
                if (strncmp(rx_buf, "HOME", 4) == 0) {
                    home_req = true;
                } else if (strncmp(rx_buf, "SAVE", 4) == 0) {
                    save_req = true;
                } else if (strncmp(rx_buf, "+0000+0000", 10) == 0) {
                    /* 精确匹配 K230 丢失包 */
                    track_lost = true;
                } else {
                    char x_str[5] = {0};
                    char y_str[5] = {0};
                    strncpy(x_str, rx_buf,     5);   /* "+xxxx" */
                    strncpy(y_str, rx_buf + 5, 5);   /* "+yyyy" */
                    track_x   = (int16_t)atoi(x_str);
                    track_y   = (int16_t)atoi(y_str);
                    track_new  = true;
                    track_lost = false;
                }
            } else if (rx_idx == 4) {
                rx_buf[rx_idx] = '\0';
                if (strncmp(rx_buf, "HOME", 4) == 0) {
                    home_req = true;
                } else if (strncmp(rx_buf, "SAVE", 4) == 0) {
                    save_req = true;
                } else if (strncmp(rx_buf, "SCAN", 4) == 0) {
                    scan_req = true;
                }
            }
            rx_idx = 0;
        } else {
            if (rx_idx < RX_BUF_SIZE - 1) {
                rx_buf[rx_idx++] = (char)rx_byte;
            } else {
                rx_idx = 0;
            }
        }
        HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
    }
}

/* ==================== 保存原点 ==================== */
static void Do_SaveOrigin(void)
{
    uint32_t timeout;
    timeout = 100;
    while (uart1_tx_busy && --timeout) { HAL_Delay(1); }
    uart1_tx_busy = true;
    Emm_V5_Origin_Set_O(MOTOR1_ADDR, 1);
    HAL_Delay(10);

    timeout = 100;
    while (uart1_tx_busy && --timeout) { HAL_Delay(1); }
    uart1_tx_busy = true;
    Emm_V5_Origin_Set_O(MOTOR2_ADDR, 1);
    HAL_Delay(10);
}

/* ==================== 双轴回零 ==================== */
static void Do_Home(void)
{
    uint32_t timeout;
    timeout = 100;
    while (uart1_tx_busy && --timeout) { HAL_Delay(1); }
    uart1_tx_busy = true;
    Emm_V5_Origin_Trigger_Return(MOTOR1_ADDR, 0, 0);
    HAL_Delay(10);

    timeout = 100;
    while (uart1_tx_busy && --timeout) { HAL_Delay(1); }
    uart1_tx_busy = true;
    Emm_V5_Origin_Trigger_Return(MOTOR2_ADDR, 0, 0);
    HAL_Delay(10);
}

/* ==================== 电机释放/使能 ==================== */
static void Motor_Disable(void)
{
    uint32_t timeout;
    timeout = 100;
    while (uart1_tx_busy && --timeout) { HAL_Delay(1); }
    uart1_tx_busy = true;
    Emm_V5_En_Control(MOTOR1_ADDR, 0, 0);
    HAL_Delay(10);
    timeout = 100;
    while (uart1_tx_busy && --timeout) { HAL_Delay(1); }
    uart1_tx_busy = true;
    Emm_V5_En_Control(MOTOR2_ADDR, 0, 0);
    HAL_Delay(10);
    motor_enabled = false;
    last_x_pulse = -1;  /* 重置去重缓存 */
    last_y_pulse = -1;
}

static void Motor_Enable(void)
{
    uint32_t timeout;
    timeout = 100;
    while (uart1_tx_busy && --timeout) { HAL_Delay(1); }
    uart1_tx_busy = true;
    Emm_V5_En_Control(MOTOR1_ADDR, 1, 0);
    HAL_Delay(10);
    timeout = 100;
    while (uart1_tx_busy && --timeout) { HAL_Delay(1); }
    uart1_tx_busy = true;
    Emm_V5_En_Control(MOTOR2_ADDR, 1, 0);
    HAL_Delay(10);

    /* 设定快速位置模式参数(F1): 速度/加速度/相对运动模式/同步标志
       每帧后续只发 FC + 脉冲数即可运动. raF=0 相对上一目标位置. */
    timeout = 100;
    while (uart1_tx_busy && --timeout) { HAL_Delay(1); }
    uart1_tx_busy = true;
    Emm_V5_Set_QPos_Params(MOTOR1_ADDR, POS_SPEED_RPM, POS_ACC, 2, 0);
    HAL_Delay(10);
    timeout = 100;
    while (uart1_tx_busy && --timeout) { HAL_Delay(1); }
    uart1_tx_busy = true;
    Emm_V5_Set_QPos_Params(MOTOR2_ADDR, Y_POS_SPEED_RPM, Y_POS_ACC, 2, 0);
    HAL_Delay(200);
    motor_enabled = true;
    last_x_pulse = -1;  /* 重置去重缓存 */
    last_y_pulse = -1;
}

/* ==================== 电机初始化 ==================== */
static void Motor_Init(void)
{
    HAL_Delay(500);

    uart1_tx_busy = true;
    Emm_V5_En_Control(MOTOR1_ADDR, 1, 0);
    HAL_Delay(200);

    uart1_tx_busy = true;
    Emm_V5_En_Control(MOTOR2_ADDR, 1, 0);
    HAL_Delay(200);

    /* Y轴重载补偿: 切闭环FOC + 提满电流5000mA增扭(电池+开发板加重后开环默认电流抬不动).
       编码器校准(Trig_Encoder_Cal)是出厂一次性操作, 不放这里——每次开机跑会自转+滴响,
       且重载下校准转动易被负载干扰. 电机已校准过(0.00err)直接用即可.
       svF=1写入Flash: 首次配置写一次, 后续调参改 svF=0 只改运行时不写Flash延长寿命. */
    uart1_tx_busy = true;
    Emm_V5_Modify_Ctrl_Mode(MOTOR2_ADDR, 1, 1);   /* ctrl_mode=1=闭环FOC, 闭环自动补丢步抗重载 */
    HAL_Delay(200);                         /* 切模式后电机需重新初始化, 多等 */

    uart1_tx_busy = true;
    Emm_V5_Modify_FOC_mA(MOTOR2_ADDR, 1, Y_FOC_mA); /* 闭环电流拉满5000mA, 直接决定扭矩 */
    HAL_Delay(50);

    /* 设定快速位置模式参数(F1): 速度/加速度/相对运动模式/同步标志 */
    uart1_tx_busy = true;
    Emm_V5_Set_QPos_Params(MOTOR1_ADDR, POS_SPEED_RPM, POS_ACC, 2, 0);
    HAL_Delay(10);

    uart1_tx_busy = true;
    Emm_V5_Set_QPos_Params(MOTOR2_ADDR, Y_POS_SPEED_RPM, Y_POS_ACC, 2, 0);
    HAL_Delay(10);
}

/* ==================== 主函数 ==================== */
int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_DMA_Init();
    MX_USART1_UART_Init();
    MX_USART2_UART_Init();

    bool need_home = (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST) != RESET);
    __HAL_RCC_CLEAR_RESET_FLAGS();

    HAL_UART_Receive_IT(&huart2, &rx_byte, 1);

    /* 上电闪灯 3 次 */
    for (int i = 0; i < 3; i++) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
        HAL_Delay(150);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
        HAL_Delay(150);
    }

    HAL_Delay(500);
    char *msg = "TEST\r\n";
    HAL_UART_Transmit(&huart1, (uint8_t *)msg, 6, 1000);
    HAL_Delay(200);

    Motor_Init();

    if (need_home) {
        for (int i = 0; i < 5; i++) {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
            HAL_Delay(50);
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
            HAL_Delay(50);
        }
        Do_Home();
    }

    /* 主循环: 处理 HOME/SAVE/SCAN 命令 + 扫描状态机 + 追踪 PID */
    while (1) {
        /* HOME 命令 */
        if (home_req) {
            home_req = false;
            track_new = false;
            scan_req = false;
            if (scan_state != SCAN_IDLE) scan_enter_idle();   /* HOME 打断扫描 */
            if (!motor_enabled) Motor_Enable();
            Do_Home();
            last_track_ms = HAL_GetTick();
        }

        /* SAVE 命令 */
        if (save_req) {
            save_req = false;
            track_new = false;
            scan_req = false;
            if (scan_state != SCAN_IDLE) scan_enter_idle();   /* SAVE 打断扫描 */
            if (!motor_enabled) Motor_Enable();
            Do_SaveOrigin();
            last_track_ms = HAL_GetTick();
        }

        /* SCAN 命令 (K230 USER 键触发) */
        if (scan_req) {
            scan_req = false;
            if (scan_state == SCAN_IDLE) {
                scan_state = SCAN_STARTING;   /* Scan_Tick 会调 scan_enter_starting */
            }
            /* 扫描中再按一次: 忽略 (避免重启扫描打乱节奏) */
        }

        /* 追踪超时 → 释放电机 (仅 IDLE 态判; 扫描中可能暂无 K230 包不释放) */
        if (motor_enabled && scan_state == SCAN_IDLE &&
            (HAL_GetTick() - last_track_ms > LOST_TIMEOUT_MS)) {
            Motor_Disable();   /* 内部已重置 last_x/y_pulse 缓存 */
        }

        /* 扫描状态机 vs 原 PID: 二选一, 确保扫描期间 Process_DX/Process_DY
           不被这里调用 (否则 Vel_Control 与 QPos_Control 给同一电机打架).
           Process_DX/Process_DY 只在 LOCKING/LOCKED 态由 Scan_Tick 内部调,
           且进入前已 Stop_Now + 重设参数. */
        if (scan_state != SCAN_IDLE) {
            Scan_Tick();
        } else if (track_new) {
            track_new = false;

            if (!motor_enabled) Motor_Enable();

            if (!track_lost) {
                /* 稳定追踪 → 根据 K230 偏移走 PID */
                last_track_ms = HAL_GetTick();
                Process_DX((int32_t)track_x);
                Process_DY((int32_t)track_y);
            } else {
                /* 丢失 → 全停转 (K230 发 +0000+0000 触发, 与 K230 侧"丢失即发0"对齐) */
                last_track_ms = HAL_GetTick();
                Process_DX(0);
                Process_DY(0);
            }
        }

        HAL_Delay(1);
    }
}

/* ==================== 系统时钟 ==================== */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
        Error_Handler();

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
        Error_Handler();
}

void Error_Handler(void) { __disable_irq(); while (1) {} }
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *f, uint32_t l) {}
#endif
