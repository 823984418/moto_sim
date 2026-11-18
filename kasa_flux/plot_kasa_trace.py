"""把 `example1 --headless --use-kasa --csv` 的轨迹画成 PNG（只依赖 Pillow）。

用法：
    python plot_kasa_trace.py trace.csv kasa_trace.png

曲线：角度误差、R、半径、可观性/可信度。参考线用 FAN 电机的真值
（R = 384.032 s^-1，radius = 4.0424 A），换电机时去掉即可。
"""

import csv
import math
import sys

from PIL import Image, ImageDraw, ImageFont

W, H = 1500, 1120
MARGIN_L, MARGIN_R, MARGIN_T = 125, 30, 60
PANELS = 4
PANEL_GAP = 66
TRUE_R = 384.032
TRUE_RADIUS = 4.0424


def load(path):
    cols = {}
    with open(path, newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for name in reader.fieldnames:
            cols[name] = []
        for row in reader:
            for name in reader.fieldnames:
                try:
                    cols[name].append(float(row[name]))
                except (TypeError, ValueError):
                    cols[name].append(float("nan"))
    return cols


def font(size):
    for name in ("C:/Windows/Fonts/arial.ttf", "C:/Windows/Fonts/segoeui.ttf"):
        try:
            return ImageFont.truetype(name, size)
        except OSError:
            pass
    return ImageFont.load_default()


F_TITLE = font(19)
F_SMALL = font(13)
F_AXIS = font(12)
PALETTE = [(30, 90, 200), (200, 60, 40), (20, 150, 80), (150, 60, 180)]


def draw_panel(draw, box, title, series, tmax, log=False, ylim=None, refs=(), xticks=True):
    x0, y0, x1, y1 = box
    draw.text((x0, y0 - 27), title, fill=(20, 20, 20), font=F_TITLE)
    draw.rectangle(box, outline=(120, 120, 120), width=1)

    vals = []
    for _, ys in series:
        if log:
            vals += [math.log10(y) for y in ys if y == y and y > 0]
        else:
            vals += [y for y in ys if y == y]
    for v, _ in refs:
        vals.append(math.log10(v) if log else v)
    if ylim is not None:
        lo, hi = ylim
    else:
        lo, hi = (min(vals), max(vals)) if vals else (0.0, 1.0)
        if log:
            lo, hi = math.floor(lo), math.ceil(hi)
            if hi - lo < 1:
                hi = lo + 1
        elif hi - lo < 1e-12:
            lo, hi = lo - 1.0, hi + 1.0
        else:
            pad = 0.08 * (hi - lo)
            lo, hi = lo - pad, hi + pad
    span = hi - lo if hi > lo else 1.0

    plot_x0, plot_x1 = x0 + 18, x1 - 14
    plot_y0, plot_y1 = y0 + 12, y1 - (24 if xticks else 10)

    def px(t):
        return plot_x0 + (plot_x1 - plot_x0) * (t / tmax if tmax > 0 else 0.0)

    def py(v):
        if log:
            v = math.log10(v) if v > 0 else float("nan")
        return plot_y1 - (plot_y1 - plot_y0) * ((v - lo) / span)

    for i in range(5):
        v = lo + span * i / 4.0
        yy = py(10 ** v if log else v)
        draw.line([(plot_x0, yy), (plot_x1, yy)], fill=(226, 226, 226))
        label = f"{10 ** v:.1e}" if log else f"{v:.4g}"
        draw.text((x0 - 13, yy - 7), label, fill=(80, 80, 80), font=F_AXIS)
    if xticks:
        for i in range(6):
            t = tmax * i / 5.0
            xx = px(t)
            draw.line([(xx, plot_y0), (xx, plot_y1)], fill=(238, 238, 238))
            draw.text((xx - 10, y1 - 22), f"{t:.1f}", fill=(80, 80, 80), font=F_AXIS)

    for v, name in refs:
        yy = py(v)
        for xx in range(int(plot_x0), int(plot_x1), 9):
            draw.line([(xx, yy), (min(xx + 4, plot_x1), yy)], fill=(150, 150, 150))
        draw.text((plot_x0 + 6, yy - 16), name, fill=(120, 120, 120), font=F_AXIS)

    for k, (name, ys) in enumerate(series):
        color = PALETTE[k % len(PALETTE)]
        draw.text((plot_x1 - 170 * (len(series) - k) + 8, y0 + 4), name,
                  fill=color, font=F_SMALL)
        pts = []
        for t, v in zip(series_time, ys):
            if v != v or (log and v <= 0):
                continue
            pts.append((px(t), py(v)))
        if len(pts) > 1:
            draw.line(pts, fill=color, width=2, joint="curve")


def main():
    global series_time
    csv_path = sys.argv[1] if len(sys.argv) > 1 else "trace.csv"
    out_path = sys.argv[2] if len(sys.argv) > 2 else "kasa_trace.png"
    title = sys.argv[3] if len(sys.argv) > 3 else csv_path
    c = load(csv_path)
    series_time = c["t"]
    tmax = max(series_time) if series_time else 1.0

    img = Image.new("RGB", (W, H), (255, 255, 255))
    d = ImageDraw.Draw(img)
    d.text((MARGIN_L, 16), f"moto_sim streaming Kasa flux observer — {title}",
           fill=(20, 20, 20), font=F_TITLE)

    plot_top = MARGIN_T + 26
    plot_h = (H - plot_top - 30 - (PANELS - 1) * PANEL_GAP) // PANELS
    boxes = []
    for i in range(PANELS):
        y0 = plot_top + i * (plot_h + PANEL_GAP)
        boxes.append((MARGIN_L, y0, W - MARGIN_R, y0 + plot_h))

    draw_panel(d, boxes[0], "angle error [deg]",
               [("err(PLL)", c["err"]), ("err_meas(circle)", c["err_meas"])],
               tmax, ylim=(-10.0, 10.0))

    draw_panel(d, boxes[1], "R = Rs/L  [s^-1]",
               [("estimated", c["r"])], tmax,
               refs=((TRUE_R, "true"),))

    draw_panel(d, boxes[2], "radius = psi_f/L  [A]",
               [("estimated", c["radius"])], tmax,
               refs=((TRUE_RADIUS, "true"),))

    draw_panel(d, boxes[3], "observability / credibility",
               [("obs", c["obs"]), ("sigma_rel", c["sigma_rel"])],
               tmax, log=True)

    img.save(out_path)
    print(f"saved {out_path}")


if __name__ == "__main__":
    main()
