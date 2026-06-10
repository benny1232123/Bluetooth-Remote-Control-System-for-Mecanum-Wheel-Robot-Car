import sensor, image, time
from pyb import UART, LED

# ======================== 传感器初始化 ========================
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(10)
sensor.set_auto_whitebal(True)
sensor.set_auto_gain(True)
sensor.set_auto_exposure(True)
sensor.skip_frames(150)
sensor.set_auto_whitebal(False)
sensor.set_auto_gain(False)
sensor.set_auto_exposure(False)
sensor.skip_frames(30)

# ======================== 参数配置 ========================
roi = (107, 0, 213, 160)

HOUGH_THRESHOLD  = 2500
MAG_FILTER       = 80
MERGE_DIST       = 15
LENS_CORR_FACTOR = 1.8

# 每 SEND_SIZE 个有效检测结果，取中值发给 MCU
SEND_SIZE = 1

# ======================== 外设初始化 ========================
uart = UART(3, 115200)
uart.init(115200, bits=8, parity=None, stop=1)

led_R = LED(1)
led_G = LED(2)
led_B = LED(3)

# ---- UART startup test ----
uart.write("OPENMV_READY\r\n")
print("UART3 init OK")


# ======================== 工具函数 ========================
def dist(ax, ay, bx, by):
    return ((ax - bx) ** 2 + (ay - by) ** 2) ** 0.5


def median_val(lst):
    s = sorted(lst)
    return s[len(s) // 2]


# ======================== 缓冲区 ========================
buf_cx = []
buf_cy = []
last_cx, last_cy = 0, 0
has_result = False
send_count = 0       # total UART sends for diagnostics
frame_count = 0      # total frames for alive message


# ======================== 主循环 ========================
clock = time.clock()

while True:
    clock.tick()

    img = sensor.snapshot().lens_corr(LENS_CORR_FACTOR)

    # --- Hough 检测 ---
    circles = img.find_circles(
        threshold=HOUGH_THRESHOLD,
        x_margin=10, y_margin=10, r_margin=10,
        r_min=8, r_max=120, r_step=2,
        roi=roi
    )
    circles = [c for c in circles if c.magnitude() >= MAG_FILTER]

    # 空间去重
    deduped = []
    for c in circles:
        dup = False
        for r in deduped:
            if dist(c.x(), c.y(), r.x(), r.y()) < MERGE_DIST:
                dup = True
                if c.magnitude() > r.magnitude():
                    deduped.remove(r)
                    deduped.append(c)
                break
        if not dup:
            deduped.append(c)

    # 取最大的圆
    best = None
    if deduped:
        best = max(deduped, key=lambda c: c.r())

    # --- 画面绘制 ---
    led_R.on()
    led_G.on()
    led_B.on()

    if best:
        img.draw_circle(best.x(), best.y(), best.r(),
                        color=(255, 0, 0), thickness=2)
        img.draw_cross(best.x(), best.y(),
                       color=(255, 255, 0), thickness=2)

        # 存入缓冲区
        buf_cx.append(best.x())
        buf_cy.append(best.y())

    # --- 攒够 SEND_SIZE 个结果，取中值发给 MCU ---
    if len(buf_cx) >= SEND_SIZE:
        med_cx = median_val(buf_cx)
        med_cy = median_val(buf_cy)

        last_cx = med_cx
        last_cy = med_cy
        has_result = True

        msg = "C%dX%dY" % (med_cx, med_cy)
        uart.write(msg)
        send_count += 1
        led_B.toggle()  # blue LED toggles on each UART send

        buf_cx = []
        buf_cy = []

    elif not best and len(buf_cx) == 0:
        # 没检测到且缓冲区为空
        if has_result:
            uart.write("C%dX%dY" % (last_cx, last_cy))
        else:
            uart.write('Q')
        send_count += 1
        led_B.toggle()

    frame_count += 1
    if frame_count % 50 == 0:
        print("ALIVE frames=%d sends=%d fps=%.1f" % (frame_count, send_count, clock.fps()))
