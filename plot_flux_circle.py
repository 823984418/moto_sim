"""Plot the flux-circle observer traces produced by example1 --headless --csv."""

import math

import pandas as pd
from PIL import Image, ImageDraw, ImageFont

FONT_PATH = r"C:\Windows\Fonts\msyh.ttc"
W, H = 1600, 1000
BG = (255, 255, 255)
FG = (30, 30, 30)
GRID = (225, 225, 225)
COLORS = [(30, 90, 200), (220, 80, 30), (20, 150, 70), (150, 60, 180)]

font_title = ImageFont.truetype(FONT_PATH, 24)
font_panel = ImageFont.truetype(FONT_PATH, 25)
font_label = ImageFont.truetype(FONT_PATH, 18)
font_small = ImageFont.truetype(FONT_PATH, 15)

img = Image.new("RGB", (W, H), BG)
draw = ImageDraw.Draw(img)


def load(path):
    df = pd.read_csv(path)
    df["err_deg"] = df["err"] * 180.0 / math.pi
    return df


def panel(box, title, xlabel, xlim, ylim, series, note=None):
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
        draw.text((x0 - 48, y - 8), f"{v:g}", fill=FG, font=font_small)

    def tx(t):
        return x0 + (t - xlim[0]) / (xlim[1] - xlim[0]) * (x1 - x0)

    def ty(v):
        v = max(ylim[0], min(ylim[1], v))
        return y0 + (ylim[1] - v) / (ylim[1] - ylim[0]) * (y1 - y0)

    for idx, (label, xs, ys) in enumerate(series):
        color = COLORS[idx % len(COLORS)]
        step = max(1, len(xs) // 2500)
        pts = [(tx(x), ty(y)) for x, y in zip(xs[::step], ys[::step])]
        if len(pts) > 1:
            draw.line(pts, fill=color, width=2)

    draw.text((x0, y0 - 30), title, fill=FG, font=font_panel)
    draw.text(((x0 + x1) // 2 - 20, y1 + 26), xlabel, fill=FG, font=font_label)
    if note:
        draw.text((x0 + 10, y1 - 26), note, fill=(90, 90, 90), font=font_small)


m1 = load("trace_1ma.csv")
m10 = load("trace_10ma.csv")
rid = load("trace_rid.csv")
true_r = 384.032
rid["r_err_pct"] = (rid.r - true_r) / true_r * 100.0

panel(
    (90, 110, 760, 425),
    "1 mA 电流噪声：角度误差 (deg)",
    "t / s",
    (0, 8),
    (-3, 3),
    [("1 mA, R 初值=真值", m1.t, m1.err_deg)],
    note="1 mA, R 初值=真值；稳态 RMS 0.64°，最大 1.37°（启动段裁剪）",
)
panel(
    (880, 110, 1550, 425),
    "10 mA 电流噪声：角度误差 (deg)",
    "t / s",
    (0, 8),
    (-40, 40),
    [("10 mA, R 初值=真值", m10.t, m10.err_deg)],
    note="10 mA；稳态 RMS 10.9°，最大 29.5°；拟合被 σ 门限拒绝",
)
panel(
    (90, 610, 760, 925),
    "R 辨识：相对误差 (%)",
    "t / s",
    (0, 8),
    (-2, 27),
    [("估计 R", rid.t, rid.r_err_pct), ("真值", rid.t, [0.0] * len(rid))],
    note="蓝=估计，橙=真值；初值 +25%，首次拟合后 +0.10%",
)
panel(
    (880, 610, 1550, 925),
    "R 辨识过程：角度误差 (deg)",
    "t / s",
    (0, 8),
    (-20, 20),
    [("25% R 误差", rid.t, rid.err_deg)],
    note="25% R 误差；1 s 前角度被旋转，拟合后稳态 RMS 0.65°",
)

draw.text((90, 25), "PMSM 零矢量斜率磁链圆观测器：moto_sim 合成验证", fill=FG, font=ImageFont.truetype(FONT_PATH, 30))

img.save("flux_circle_report.png")
print("saved flux_circle_report.png")
