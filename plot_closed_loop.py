"""Plot the closed-loop handover trace."""

import math

import pandas as pd
from PIL import Image, ImageDraw, ImageFont

FONT_PATH = r"C:\Windows\Fonts\msyh.ttc"
W, H = 1600, 620
BG = (255, 255, 255)
FG = (30, 30, 30)
GRID = (225, 225, 225)

font_title = ImageFont.truetype(FONT_PATH, 24)
font_panel = ImageFont.truetype(FONT_PATH, 25)
font_label = ImageFont.truetype(FONT_PATH, 18)
font_small = ImageFont.truetype(FONT_PATH, 15)

img = Image.new("RGB", (W, H), BG)
draw = ImageDraw.Draw(img)
df = pd.read_csv("trace_closed_ok.csv")
df["err_deg"] = df["err"] * 180.0 / math.pi


def panel(box, title, xlabel, xlim, ylim, series, note=None, vline=None):
    x0, y0, x1, y1 = box
    draw.rectangle(box, outline=(120, 120, 120), width=1)
    for i in range(6):
        x = x0 + (x1 - x0) * i / 5
        draw.line([(x, y0), (x, y1)], fill=GRID, width=1)
        v = xlim[0] + (xlim[1] - xlim[0]) * i / 5
        draw.text((x - 10, y1 + 5), f"{v:g}", fill=FG, font=font_small)
    for i in range(5):
        y = y0 + (y1 - y0) * i / 4
        draw.line([(x0, y), (x1, y)], fill=GRID, width=1)
        v = ylim[1] - (ylim[1] - ylim[0]) * i / 4
        draw.text((x0 - 52, y - 8), f"{v:g}", fill=FG, font=font_small)

    def tx(t):
        return x0 + (t - xlim[0]) / (xlim[1] - xlim[0]) * (x1 - x0)

    def ty(v):
        v = max(ylim[0], min(ylim[1], v))
        return y0 + (ylim[1] - v) / (ylim[1] - ylim[0]) * (y1 - y0)

    colors = [(30, 90, 200), (220, 80, 30)]
    for idx, (label, xs, ys) in enumerate(series):
        color = colors[idx % len(colors)]
        step = max(1, len(xs) // 2500)
        pts = [(tx(x), ty(y)) for x, y in zip(xs[::step], ys[::step])]
        if len(pts) > 1:
            draw.line(pts, fill=color, width=2)
        lx = x0 + 14 + idx * 260
        ly = y0 + 14
        draw.rectangle([lx - 4, ly - 12, lx + 210, ly + 12], fill=BG)
        draw.line([(lx, ly), (lx + 34, ly)], fill=color, width=3)
        draw.text((lx + 40, ly - 9), label, fill=FG, font=font_small)

    draw.text((x0, y0 - 30), title, fill=FG, font=font_panel)
    if vline is not None:
        xv = tx(vline)
        for yy in range(y0 + 4, y1, 12):
            draw.line([(xv, yy), (xv, min(yy + 6, y1))], fill=(120, 120, 120), width=2)
        draw.text((xv + 5, y0 + 4), "交班", fill=(90, 90, 90), font=font_small)
    draw.text(((x0 + x1) // 2 - 20, y1 + 26), xlabel, fill=FG, font=font_label)
    if note:
        draw.text((x0 + 10, y1 - 26), note, fill=(90, 90, 90), font=font_small)


panel(
    (90, 90, 760, 480),
    "闭环交接（3 s）：角度误差 (deg)",
    "t / s",
    (0, 8),
    (-5, 5),
    [("磁链圆观测器闭环", df.t, df.err_deg)],
    note="交接后稳态 RMS 0.86°，最大 2.22°",
    vline=3.0,
)
panel(
    (880, 90, 1550, 480),
    "闭环交接：电角速度 (rad/s)",
    "t / s",
    (0, 8),
    (15, 22),
    [("真实速度", df.t, df.true_speed), ("观测速度", df.t, df.est_speed)],
    note="稳态速度 RMS 0.072 rad/s",
    vline=3.0,
)

draw.text((90, 22), "磁链圆观测器闭环交接（速度环增益 0.02）", fill=FG, font=ImageFont.truetype(FONT_PATH, 30))

img.save("flux_circle_closed_loop.png")
print("saved flux_circle_closed_loop.png")
