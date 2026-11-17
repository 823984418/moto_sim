"""Draw two schematics: observer block diagram and zero-vector double sampling."""

from PIL import Image, ImageDraw, ImageFont

FONT = r"C:\Windows\Fonts\msyh.ttc"
BG = (255, 255, 255)
FG = (30, 30, 30)
BLUE = (30, 90, 200)
RED = (210, 70, 40)
GREEN = (30, 140, 80)
GREY = (120, 120, 120)

f_title = ImageFont.truetype(FONT, 30)
f_box = ImageFont.truetype(FONT, 20)
f_small = ImageFont.truetype(FONT, 16)
f_tiny = ImageFont.truetype(FONT, 14)


def box(d, xy, text, color=BLUE):
    d.rounded_rectangle(xy, radius=14, outline=color, width=3, fill=(248, 250, 255))
    lines = text.split("\n")
    x0, y0, x1, y1 = xy
    h = len(lines) * 26
    for i, line in enumerate(lines):
        w = d.textlength(line, font=f_box)
        d.text(((x0 + x1 - w) / 2, (y0 + y1 - h) / 2 + i * 26), line, fill=FG, font=f_box)


def arrow(d, p0, p1, color=GREY, width=3):
    d.line([p0, p1], fill=color, width=width)
    # arrow head
    import math

    ang = math.atan2(p1[1] - p0[1], p1[0] - p0[0])
    L = 12
    for s in (+1, -1):
        d.line(
            [
                p1,
                (
                    p1[0] - L * math.cos(ang - s * 0.4),
                    p1[1] - L * math.sin(ang - s * 0.4),
                ),
            ],
            fill=color,
            width=width,
        )


# ---------------------------------------------------------------- block diagram
img = Image.new("RGB", (1400, 820), BG)
d = ImageDraw.Draw(img)
d.text((60, 24), "磁链圆观测器数据流（moto_sim 实现）", fill=FG, font=f_title)

box(d, (60, 110, 360, 220), "PMSM + 两电平桥\n（平均模型）")
box(d, (470, 110, 760, 220), "零矢量双采样\n i1, i2")
box(d, (850, 110, 1200, 220), "斜率 / 中点电流\n s0=(i2-i1)/Δt\n i=(i1+i2)/2")

box(d, (850, 320, 1200, 430), "相对积分\n A=∫s0dt,  B=∫idt\n u=A+R·B")
box(d, (850, 530, 1200, 650), "加窗圆拟合\n R, c, v, rho\n gamma, sigma")
box(d, (470, 530, 760, 650), "二阶 PLL")
box(d, (60, 530, 360, 650), "电流环 / 速度环\n（dq 控制）")

arrow(d, (360, 165), (470, 165))
arrow(d, (760, 165), (850, 165))
arrow(d, (1025, 220), (1025, 320))
arrow(d, (1025, 430), (1025, 530))
arrow(d, (850, 590), (760, 590))
arrow(d, (470, 590), (360, 590))
arrow(d, (210, 530), (210, 220))

d.text((770, 555), "u = A + R·B", fill=GREEN, font=f_small)
d.text((770, 615), "R, c, rho", fill=BLUE, font=f_small)
d.text((220, 360), "电压指令 / 占空比", fill=GREY, font=f_small)
d.text((370, 605), "theta, omega", fill=GREY, font=f_small)

# observer dashed box
d.rounded_rectangle((430, 60, 1240, 690), radius=20, outline=RED, width=3)
d.rectangle([445, 58, 600, 82], fill=BG)
d.text((450, 62), "磁链圆观测器", fill=RED, font=f_small)

img.save("flux_circle_block.png")


# ---------------------------------------------------------------- PWM timing
img = Image.new("RGB", (1400, 720), BG)
d = ImageDraw.Draw(img)
d.text((50, 24), "中心对齐 PWM、零矢量与双采样时序（示意）", fill=FG, font=f_title)

x0, x1 = 120, 1280
y_ax = 560
Tc = (x1 - x0) / 2.0
periods = 2
d.line([(x0, y_ax), (x1, y_ax)], fill=FG, width=2)
d.line([(x0, y_ax - 10), (x0, y_ax + 10)], fill=FG, width=2)
d.line([(x0 + Tc, y_ax - 10), (x0 + Tc, y_ax + 10)], fill=FG, width=2)
d.text((x0 + Tc / 2 - 40, y_ax + 18), "T_c", fill=FG, font=f_small)
d.text((x0 + Tc * 1.5 - 40, y_ax + 18), "T_c", fill=FG, font=f_small)

# duties (fraction of carrier high, centered)
duties = [0.24, 0.5, 0.76]
names = ["A 相", "B 相", "C 相"]
colors = [BLUE, GREEN, RED]
base_ys = [120, 230, 340]
amp = 70
dmax = max(duties)
dmin = min(duties)
low_len = (1.0 - dmax) * Tc / 2.0
high_len = dmin * Tc

# shade zero vectors first (so the phase traces draw on top)
for p in range(periods):
    c0 = x0 + p * Tc
    d.rectangle([c0, 90, c0 + low_len, 430], fill=(235, 240, 250))
    d.rectangle([c0 + Tc - low_len, 90, c0 + Tc, 430], fill=(235, 240, 250))
    h_start = c0 + (Tc - high_len) / 2.0
    d.rectangle([h_start, 90, h_start + high_len, 430], fill=(250, 236, 232))

# phase traces on top
for p in range(periods):
    cyc0 = x0 + p * Tc
    for duty, name, color, ybase in zip(duties, names, colors, base_ys):
        high = duty * Tc
        h0 = cyc0 + (Tc - high) / 2
        h1 = h0 + high
        ylow = ybase + amp
        yhigh = ybase
        d.line([(cyc0, ylow), (h0, ylow)], fill=color, width=3)
        d.line([(h0, ylow), (h0, yhigh)], fill=color, width=3)
        d.line([(h0, yhigh), (h1, yhigh)], fill=color, width=3)
        d.line([(h1, yhigh), (h1, ylow)], fill=color, width=3)
        d.line([(h1, ylow), (cyc0 + Tc, ylow)], fill=color, width=3)
        d.text((60, ybase + 20), name, fill=color, font=f_box)

# labels for the first period
d.text((x0 + 8, 430), "000", fill=BLUE, font=f_small)
d.text((x0 + (Tc - high_len) / 2 + 8, 430), "111", fill=RED, font=f_small)

# sampling detail in the first 000 interval
tau_d = 0.035 * Tc
dt = 0.05 * Tc
t1 = x0 + tau_d
t2 = t1 + dt
for t, lab in [(t1, "i1"), (t2, "i2")]:
    d.line([(t, 90), (t, 470)], fill=(120, 120, 120), width=1)
    d.ellipse([t - 5, 455, t + 5, 465], fill=FG)
    d.text((t - 10, 475), lab, fill=FG, font=f_small)
# tau_d and dt arrows
d.line([(x0, 500), (t1, 500)], fill=GREY, width=2)
d.text(((x0 + t1) / 2 - 18, 505), "tau_d", fill=GREY, font=f_small)
d.line([(t1, 525), (t2, 525)], fill=GREY, width=2)
d.text(((t1 + t2) / 2 - 12, 528), "Δt", fill=GREY, font=f_small)
# T0 arrow
d.line([(x0, 545), (x0 + low_len, 545)], fill=GREY, width=2)
d.text((x0 + low_len / 2 - 12, 548), "T0", fill=GREY, font=f_small)

d.text(
    (x0, 640),
    "零矢量 000：三相下桥臂全通，端电压 v=0，测得 s0=di/dt|v=0；"
    "111 为三相上桥臂全通，可做对称差分。",
    fill=FG,
    font=f_small,
)

img.save("pwm_zero_vector.png")

print("saved flux_circle_block.png, pwm_zero_vector.png")
