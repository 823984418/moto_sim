"""Plot the virtual-flux circle u = A + R*B in the alpha-beta plane."""

import math

import pandas as pd
from PIL import Image, ImageDraw, ImageFont

FONT_PATH = r"C:\Windows\Fonts\msyh.ttc"
W, H = 1000, 860
BG = (255, 255, 255)
FG = (30, 30, 30)
GRID = (225, 225, 225)
BLUE = (30, 90, 200)
RED = (210, 70, 40)

img = Image.new("RGB", (W, H), BG)
draw = ImageDraw.Draw(img)
font_title = ImageFont.truetype(FONT_PATH, 26)
font_label = ImageFont.truetype(FONT_PATH, 18)
font_small = ImageFont.truetype(FONT_PATH, 15)

df = pd.read_csv("trace_circle.csv")
tail = df[df.t > 3.65].copy()
cx = tail.c_x.iloc[-1]
cy = tail.c_y.iloc[-1]
rad = tail.radius.iloc[-1]

# data bounds, include the fitted circle
xs = list(tail.u_x) + [cx - rad, cx + rad]
ys = list(tail.u_y) + [cy - rad, cy + rad]
pad = 0.7
xmin, xmax = min(xs) - pad, max(xs) + pad
ymin, ymax = min(ys) - pad, max(ys) + pad
span = max(xmax - xmin, ymax - ymin)
xmid = 0.5 * (xmin + xmax)
ymid = 0.5 * (ymin + ymax)
xmin, xmax = xmid - span / 2, xmid + span / 2
ymin, ymax = ymid - span / 2, ymid + span / 2

box = (100, 110, 900, 770)
x0, y0, x1, y1 = box
draw.rectangle(box, outline=(120, 120, 120), width=1)


def tx(x):
    return x0 + (x - xmin) / (xmax - xmin) * (x1 - x0)


def ty(y):
    return y1 - (y - ymin) / (ymax - ymin) * (y1 - y0)


# grid + ticks
for i in range(7):
    v = xmin + (xmax - xmin) * i / 6
    x = tx(v)
    draw.line([(x, y0), (x, y1)], fill=GRID, width=1)
    draw.text((x - 12, y1 + 6), f"{v:.1f}", fill=FG, font=font_small)
for i in range(7):
    v = ymin + (ymax - ymin) * i / 6
    y = ty(v)
    draw.line([(x0, y), (x1, y)], fill=GRID, width=1)
    draw.text((x0 - 44, y - 8), f"{v:.1f}", fill=FG, font=font_small)

# fitted circle
circle = []
for k in range(361):
    a = 2 * math.pi * k / 360
    circle.append((tx(cx + rad * math.cos(a)), ty(cy + rad * math.sin(a))))
draw.line(circle, fill=RED, width=2)

# trajectory
step = 1
pts = [(tx(x), ty(y)) for x, y in zip(tail.u_x[::step], tail.u_y[::step])]
draw.line(pts, fill=BLUE, width=2)

# center
ccx, ccy = tx(cx), ty(cy)
draw.ellipse([ccx - 5, ccy - 5, ccx + 5, ccy + 5], fill=RED)
draw.text((ccx + 8, ccy - 20), f"圆心 c = ({cx:.2f}, {cy:.2f}) A", fill=RED, font=font_small)
draw.text((tx(cx + rad * 0.7) + 8, ty(cy + rad * 0.7)), f"半径 rho = {rad:.2f} A", fill=RED, font=font_small)

# start/end markers
sx, sy = tx(tail.u_x.iloc[0]), ty(tail.u_y.iloc[0])
ex, ey = tx(tail.u_x.iloc[-1]), ty(tail.u_y.iloc[-1])
draw.ellipse([sx - 4, sy - 4, sx + 4, sy + 4], outline=BLUE, width=2)
draw.ellipse([ex - 4, ey - 4, ex + 4, ey + 4], fill=BLUE)

draw.text((100, 30), "虚拟磁链轨迹 u = A + R·B（1 mA，5 V 本体死区，20 rad/s）", fill=FG, font=font_title)
draw.text((450, 810), "u_alpha / A", fill=FG, font=font_label)
draw.text((15, 60), "u_beta / A", fill=FG, font=font_small)
draw.line([(x0 + 15, y0 + 18), (x0 + 55, y0 + 18)], fill=BLUE, width=3)
draw.text((x0 + 62, y0 + 8), "观测轨迹 u = A + R·B", fill=FG, font=font_small)
draw.line([(x0 + 15, y0 + 44), (x0 + 55, y0 + 44)], fill=RED, width=3)
draw.text((x0 + 62, y0 + 34), "拟合圆（圆心 c，半径 rho）", fill=FG, font=font_small)

img.save("flux_circle_trajectory.png")
print("saved flux_circle_trajectory.png")
