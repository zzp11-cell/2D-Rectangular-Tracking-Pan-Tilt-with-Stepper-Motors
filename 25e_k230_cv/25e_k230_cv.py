import os
import gc
import time
import cv2
import image
from machine import UART, Pin
from machine import FPIOA
from media.sensor import *
from media.display import *
from media.media import *

# 摄像头处理分辨率
# 保持 640x480 (而非 800x480): 800x480 会让 Canny/findContours 多算 25% 像素,
# 省掉的末尾 resize 抵不过处理开销, 实测反而掉帧。显示侧仍 resize 铺满 LCD。
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

# LCD 分辨率
LCD_WIDTH = 800
LCD_HEIGHT = 480

# 常见 LCD：ST7701
DISPLAY_TYPE = Display.ST7701

# Canny 阈值 (固定值, 避免 auto_canny 的 calcHist 拖慢 FPS)
CANNY_LOW = 50
CANNY_HIGH = 150

# Contour approximation accuracy
APPROX_RATIO = 0.02

# Rectangle filter
MIN_AREA = 800
MAX_AREA = 60000
MIN_RECT_WIDTH = 40
MIN_RECT_HEIGHT = 40

# 帧间平滑参数
SMOOTH_ALPHA = 0.85         # EMA 平滑系数，0.85 = 跟手快
MIN_STABLE_FRAMES = 2       # 更快进入稳定追踪
MAX_LOST_FRAMES = 5         # 丢失多少帧内保持上一次结果

# UART 发送间隔 (ms)，避免洪水般发包
UART_SEND_INTERVAL = 30     # 30ms = ~33Hz，跟手更快

# UART 调试开关: 设为 True 时在 REPL 打印发送的数据
UART_DEBUG = False

# GC 间隔 (帧): 每帧 gc.collect() 是稳定税, 改为周期性触发省 CPU。
# 识别逻辑不产生需立即回收的对象, 周期 GC 不影响。
GC_INTERVAL_FRAMES = 30

# UART2 引脚 (K230庐山派 LCKFB: TX=GPIO11, RX=GPIO12)
# 接 STM32 USART2: K230 TX(GPIO11) → STM32 PA3(RX), K230 RX(GPIO12) ← STM32 PA2(TX)
UART_TX_PIN = 11
UART_RX_PIN = 12

# USER 按键 + 激光控制 (引脚号取自 RUN offline/脱机阈值调节以及按键调整.py)
# K230 引脚功能可重映射, USER 键物理脚=53, 必须先 FPIOA 设成 GPIO53 功能才可用.
# 下拉 PULL_DOWN, 按下=高电平(1), 故按下检测上升沿(0→1). 激光高电平亮, 默认关防上电闪.
USER_BTN_PIN = 53      # 庐山派 USER 键接物理脚 53
LASER_PIN    = 33      # 激光信号口接 GPIO33, 需核原理图未占用

# K230 自判中心重合 → 拉高 GPIO33 发激光 (STM32 不发反馈, UART 单向)
# 死区略宽于 STM32 的 8, 保证 STM32 先停稳再亮; 锁定中持续亮, 丢失即灭.
LOCK_DEAD_ZONE      = 10   # 像素
LOCK_CONFIRM_FRAMES = 5    # 连续5帧(≈150ms)在死区才亮激光
LOCK_LOSE_FRAMES    = 3    # 连续3帧不在死区/丢失 → 拉低激光


def quad_diag_intersection(quad):
    """4 顶点两条对角线交点作为矩形中心, 对旋转/透视矩形比 boundingRect 中心更准。
    quad 顶点顺序与 cv2.approxPolyDP 一致: [p0, p1, p2, p3] 沿轮廓, 对角线为 p0-p2 / p1-p3。"""
    pts = quad.reshape((4, 2))
    x1, y1 = int(pts[0][0]), int(pts[0][1])
    x2, y2 = int(pts[2][0]), int(pts[2][1])
    x3, y3 = int(pts[1][0]), int(pts[1][1])
    x4, y4 = int(pts[3][0]), int(pts[3][1])
    denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if abs(denom) < 1e-9:
        # 对角线接近平行 (退化情况), 回退到 boundingRect 中心
        x, y, w, h = cv2.boundingRect(quad)
        return x + w // 2, y + h // 2
    t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom
    return int(x1 + t * (x2 - x1)), int(y1 + t * (y2 - y1))


def is_target_rectangle(approx, area):
    x, y, w, h = cv2.boundingRect(approx)
    if w <= 0 or h <= 0:
        return False

    if w < MIN_RECT_WIDTH or h < MIN_RECT_HEIGHT:
        return False

    if area < MIN_AREA or area > MAX_AREA:
        return False

    rect_area = w * h
    if rect_area <= 0:
        return False

    # 注意: Canny+findContours 检到的是矩形"边框"(空心轮廓),
    # cv2.contourArea 返回的是边框本身的面积, 而非外接矩形填充面积.
    # 因此不能用 area/rect_area 做填充率过滤(对边框会远小于 0.5 被误杀).
    # 改用周长规整度判断: 4 顶点构成的多边形周长应接近外接矩形周长.
    peri = cv2.arcLength(approx, True)
    rect_peri = 2 * (w + h)
    if rect_peri <= 0:
        return False
    # 边框多边形周长与外接矩形周长比值, 规整矩形应接近 1.0
    # 留余量容忍轮廓抖动, 取 [0.75, 1.35]
    peri_ratio = peri / rect_peri
    if peri_ratio < 0.75 or peri_ratio > 1.35:
        return False

    return True


def smooth_rect(new_rect, prev_rect, alpha=SMOOTH_ALPHA):
    """EMA 平滑矩形 (x, y, w, h)"""
    if prev_rect is None:
        return new_rect
    x1, y1, w1, h1 = new_rect
    x2, y2, w2, h2 = prev_rect
    return (
        int(alpha * x1 + (1 - alpha) * x2),
        int(alpha * y1 + (1 - alpha) * y2),
        int(alpha * w1 + (1 - alpha) * w2),
        int(alpha * h1 + (1 - alpha) * h2),
    )


def main():
    sensor = None
    clock = time.clock()
    uart = None
    last_uart_send = 0

    try:
        os.exitpoint(os.EXITPOINT_ENABLE)

        # 先清理可能残留的状态 (防止 "already inited" 报错)
        try:
            MediaManager.deinit()
            time.sleep_ms(100)
        except Exception:
            pass

        sensor = Sensor()
        try:
            sensor.reset()
        except AssertionError:
            pass
        sensor.set_framesize(width=FRAME_WIDTH, height=FRAME_HEIGHT)
        sensor.set_pixformat(Sensor.RGB888)

        Display.init(DISPLAY_TYPE, width=LCD_WIDTH, height=LCD_HEIGHT, to_ide=True)
        MediaManager.init()

        # 初始化 UART2: 115200 波特率, 接 STM32 USART2
        # K230 庐山派 UART2 硬件固定: TX=GPIO11, RX=GPIO12
        try:
            uart = UART(UART.UART2, baudrate=115200,
                        tx=UART_TX_PIN, rx=UART_RX_PIN,
                        bits=UART.EIGHTBITS,
                        parity=UART.PARITY_NONE,
                        stop=UART.STOPBITS_ONE)
            print("[UART] UART2 init OK, tx=%d rx=%d" % (UART_TX_PIN, UART_RX_PIN))
        except Exception as e:
            print("[UART] first init failed: %s" % e)
            try:
                uart = UART(UART.UART2, baudrate=115200,
                            bits=UART.EIGHTBITS,
                            parity=UART.PARITY_NONE,
                            stop=UART.STOPBITS_ONE)
                print("[UART] UART2 init OK (no pin args)")
            except Exception as e2:
                print("[UART] second init also failed: %s" % e2)

        sensor.run()
        time.sleep(0.5)

        # USER 按键 + 激光信号口初始化
        # USER 键接物理脚 53, 先用 FPIOA 重映射成 GPIO53 功能 (K230 引脚功能可重映射, 不设默认非GPIO)
        # 下拉 PULL_DOWN, 按下=高电平. 激光 GPIO33 高电平亮, value=0 默认关防上电闪.
        try:
            fpioa = FPIOA()
            fpioa.set_function(USER_BTN_PIN, FPIOA.GPIO53)
            user_btn = Pin(USER_BTN_PIN, Pin.IN, Pin.PULL_DOWN)
            laser = Pin(LASER_PIN, Pin.OUT, value=0)
            print("[GPIO] USER btn=%d(PULL_DOWN, 按下=1), laser=%d" % (USER_BTN_PIN, LASER_PIN))
        except Exception as e:
            user_btn = None
            laser = None
            print("[GPIO] init failed: %s" % e)

        # 按键去抖 + K230 本地锁定状态
        # PULL_DOWN: 松开=0, 按下=1, 故检测上升沿 (0→1) 触发
        btn_last_state = 0
        btn_last_ms = time.ticks_ms()
        btn_pressed_flag = False
        k230_lock_count = 0
        k230_lose_count = 0
        k230_locked = False

        # 帧间平滑状态变量
        prev_rect = None         # 平滑后的矩形 (x, y, w, h)
        stable_count = 0         # 连续检测到目标的帧数
        lost_count = 0           # 连续丢失目标的帧数
        is_stable = False        # 是否进入稳定跟踪状态
        frame_count = 0          # 帧计数, 用于周期性 GC

        while True:
            os.exitpoint()
            clock.tick()
            frame_count += 1

            # ---- (a) USER 按键轮询去抖: 按下 → 发 SCAN + 复位本地锁定 ----
            # PULL_DOWN: 松开=0, 按下=1. 检测上升沿 (0→1) + 50ms 去抖触发
            if user_btn is not None:
                btn_now = user_btn.value()
                now_ms = time.ticks_ms()
                if btn_now == 1 and btn_last_state == 0 and \
                   time.ticks_diff(now_ms, btn_last_ms) > 50:
                    btn_pressed_flag = True
                    btn_last_ms = now_ms
                btn_last_state = btn_now
                if btn_pressed_flag:
                    btn_pressed_flag = False
                    if uart is not None:
                        uart.write("SCAN\n")
                        print("[BTN] SCAN sent -> STM32")   # 无条件打印, 便于确认按键已发
                    else:
                        print("[BTN] pressed but UART not ready")
                    # 复位 K230 本地锁定状态, 等重新锁定再亮激光
                    k230_lock_count = 0
                    k230_lose_count = 0
                    k230_locked = False
                    if laser is not None:
                        laser.value(0)

            img = sensor.snapshot()
            frame = img.to_numpy_ref()

            # 检测用灰度图
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            blur = cv2.GaussianBlur(gray, (5, 5), 0)
            edges = cv2.Canny(blur, CANNY_LOW, CANNY_HIGH)

            contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            rect_count = 0
            best_area = 0
            best_pts = None
            best_rect = None

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < MIN_AREA or area > MAX_AREA:
                    continue

                epsilon = APPROX_RATIO * cv2.arcLength(cnt, True)
                approx = cv2.approxPolyDP(cnt, epsilon, True)

                if len(approx) != 4:
                    continue

                if not is_target_rectangle(approx, area):
                    continue

                # 选最外侧那条边: 黑色边框有宽度, Canny 会检出内/外两条边轮廓,
                # boundingRect 面积大的那个一定对应外边 (内边外接矩形被外边包住, 更小)。
                # 用 contourArea 比较会失灵 (内外边面积相近), 故改比 boundingRect 面积。
                br = cv2.boundingRect(approx)
                br_area = br[2] * br[3]
                if br_area > best_area:
                    best_area = br_area
                    best_pts = approx
                    best_rect = br

            # --- 帧间平滑逻辑 ---
            if best_pts is not None:
                # 本帧检测到了目标
                lost_count = 0
                stable_count += 1
                if stable_count >= MIN_STABLE_FRAMES:
                    is_stable = True

                # EMA 平滑矩形位置
                if is_stable:
                    best_rect = smooth_rect(best_rect, prev_rect)
                prev_rect = best_rect
            else:
                # 本帧没检测到目标
                stable_count = 0
                lost_count += 1
                if is_stable and lost_count <= MAX_LOST_FRAMES and prev_rect is not None:
                    # 短暂丢失，沿用上一次的平滑结果
                    best_rect = prev_rect
                    best_pts = None  # 没有实际轮廓点，用矩形画
                else:
                    is_stable = False
                    prev_rect = None

            # --- 绘制 ---
            if best_rect is not None:
                x, y, rw, rh = best_rect
                rect_count = 1

                if best_pts is not None:
                    # 画 approx 4 顶点连成的多边形, 贴合目标真实(旋转)轮廓
                    cv2.drawContours(frame, [best_pts], -1, (0, 255, 0), 2)
                else:
                    # 丢失暂存帧，画矩形代替
                    cv2.rectangle(frame, (x, y), (x + rw, y + rh), (0, 200, 200), 2)

                # 中心点: 有真实轮廓点时用对角线交点 (旋转/透视更准), 否则用 boundingRect 中心
                if best_pts is not None:
                    center_x, center_y = quad_diag_intersection(best_pts)
                else:
                    center_x = x + rw // 2
                    center_y = y + rh // 2
                img_center_x = FRAME_WIDTH // 2
                img_center_y = FRAME_HEIGHT // 2
                offset_x = center_x - img_center_x
                offset_y = center_y - img_center_y

                cv2.circle(frame, (center_x, center_y), 6, (0, 0, 255), -1)
                cv2.line(frame, (img_center_x, img_center_y), (center_x, center_y), (0, 255, 255), 2)
                cv2.putText(frame, "Offset:(%d,%d)" % (offset_x, offset_y),
                            (30, 145),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

                # 稳定状态指示
                status_str = "STABLE" if is_stable else "LOCK(%d/%d)" % (stable_count, MIN_STABLE_FRAMES)
                status_color = (0, 255, 0) if is_stable else (0, 165, 255)
                cv2.putText(frame, status_str, (30, 180),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, status_color, 2)

            # --- UART 发送偏移量给 STM32 ---
            # 语义: 仅当本帧真实检测到目标且已稳定追踪 → 发偏移;
            #       否则(LOCK中/短暂遮挡/彻底丢失)一律发 +0000, 不留恋过期偏移,
            #       避免 STM32 拿着旧偏移继续驱动电机。
            now_ms = time.ticks_ms()
            if time.ticks_diff(now_ms, last_uart_send) >= UART_SEND_INTERVAL:
                last_uart_send = now_ms
                if uart is not None:
                    if best_pts is not None and is_stable:
                        # 本帧真实检测到且稳定 → 发送像素偏移 (DX/DY 一起发)
                        # 中心用对角线交点, 与屏幕绘制一致
                        cx, cy = quad_diag_intersection(best_pts)
                        offset_x = cx - FRAME_WIDTH // 2
                        offset_y = cy - FRAME_HEIGHT // 2
                        # 固定10字符原子包: (符号+4位数字)×2, X+Y 同时到达
                        # 注意: %+05d 的宽度5含符号, 数字补零到4位 (|v|<1000时不超宽)
                        msg = "%+05d%+05d\n" % (offset_x, offset_y)
                        uart.write(msg)
                        if UART_DEBUG:
                            print(msg.strip())
                    else:
                        # 未稳定 / 丢失 → 持续发全0, 让 STM32 知道目标不在
                        uart.write("+0000+0000\n")
                        if UART_DEBUG:
                            print("LOST/UNLOCK")

            # --- (b)(c) K230 自判中心重合 → 拉高/拉低 GPIO33 激光 ---
            # UART 单向 (K230 TX→STM32 RX), STM32 无法反馈锁定完成, 故 K230 自己判.
            # 仅当本帧真实检测到目标且稳定时, 用对角线交点算 offset 判死区;
            # 否则计为"丢失帧". 锁定中持续亮, 连续 LOCK_LOSE_FRAMES 帧丢失 → 灭.
            if laser is not None:
                if best_pts is not None and is_stable:
                    cx, cy = quad_diag_intersection(best_pts)
                    ox = cx - FRAME_WIDTH // 2
                    oy = cy - FRAME_HEIGHT // 2
                    if abs(ox) <= LOCK_DEAD_ZONE and abs(oy) <= LOCK_DEAD_ZONE:
                        k230_lock_count += 1
                        k230_lose_count = 0
                    else:
                        k230_lose_count += 1
                        k230_lock_count = 0
                else:
                    k230_lose_count += 1
                    k230_lock_count = 0

                if k230_lock_count >= LOCK_CONFIRM_FRAMES and not k230_locked:
                    k230_locked = True
                    laser.value(1)
                    if UART_DEBUG:
                        print("[LASER] ON (locked)")
                if k230_locked and k230_lose_count >= LOCK_LOSE_FRAMES:
                    k230_locked = False
                    laser.value(0)
                    if UART_DEBUG:
                        print("[LASER] OFF (lost)")

            fps = clock.fps()
            cv2.putText(frame, "FPS: %.1f" % fps, (30, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
            cv2.putText(frame,
                        "Rectangles:%d Area:%d-%d Size>%dx%d" % (
                            rect_count, MIN_AREA, MAX_AREA, MIN_RECT_WIDTH, MIN_RECT_HEIGHT
                        ),
                        (30, 110),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)

            # 640x480 处理, 放大铺满 800x480 LCD (仅显示, 与识别无关)。
            show_np = cv2.resize(frame, (LCD_WIDTH, LCD_HEIGHT))
            show_img = image.Image(LCD_WIDTH, LCD_HEIGHT, image.RGB888,
                                   alloc=image.ALLOC_REF, data=show_np)

            Display.show_image(show_img)
            # 周期性 GC: 每帧 collect 是稳定税, 改为 GC_INTERVAL_FRAMES 帧一次。
            # 识别逻辑的中间对象 (gray/blur/edges/contours) 均为 numpy,
            # 引用计数即时回收, 无需每帧全堆扫描。
            if frame_count % GC_INTERVAL_FRAMES == 0:
                gc.collect()

    except KeyboardInterrupt:
        print("user stop")
    except BaseException as e:
        print("Exception:", e)
    finally:
        # 兜底关激光 (Ctrl+C / 异常退出时不留亮; laser 可能未初始化, 全包进 try)
        try:
            if laser is not None:
                laser.value(0)
        except Exception:
            pass
        try:
            if isinstance(sensor, Sensor):
                sensor.stop()
        except Exception:
            pass
        try:
            Display.deinit()
        except Exception:
            pass
        os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
        time.sleep_ms(100)
        MediaManager.deinit()


if __name__ == "__main__":
    main()
