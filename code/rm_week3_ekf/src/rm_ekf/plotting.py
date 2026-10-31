"""轻量绘图：PIL 手绘 + 3x 超采样抗锯齿（环境无 matplotlib）。

提供 Figure/Axes：多序列折线、网格、坐标轴、图例、标题、σ 阴影带、中文标注。
"""
import math
import os

from PIL import Image, ImageDraw, ImageFont

_FONT_CANDIDATES = [
    r"C:\Windows\Fonts\msyh.ttc",
    r"C:\Windows\Fonts\msyhl.ttc",
    r"C:\Windows\Fonts\simhei.ttf",
]

PALETTE = [
    (214, 69, 65), (43, 129, 214), (46, 168, 100), (230, 158, 34),
    (150, 84, 198), (0, 168, 190), (190, 60, 120), (110, 110, 110),
]

SS = 3  # 超采样倍数


def _font(size):
    for p in _FONT_CANDIDATES:
        if os.path.exists(p):
            try:
                return ImageFont.truetype(p, size)
            except OSError:
                continue
    return ImageFont.load_default()


def _nice_ticks(lo, hi, n=5):
    if not math.isfinite(lo) or not math.isfinite(hi) or hi <= lo:
        lo, hi = 0.0, 1.0
    raw = (hi - lo) / max(n - 1, 1)
    mag = 10 ** math.floor(math.log10(raw))
    step = 10 * mag
    for m in (1, 2, 2.5, 5, 10):
        if raw <= m * mag:
            step = m * mag
            break
    start = math.ceil(lo / step) * step
    ticks = []
    v = start
    while v <= hi + 1e-9:
        ticks.append(round(v, 10))
        v += step
    return ticks


class Axes:
    def __init__(self, fig, box, title="", xlabel="", ylabel="", ylim=None, xlim=None):
        self.fig = fig
        self.box = box
        self.title = title
        self.xlabel = xlabel
        self.ylabel = ylabel
        self.ylim = ylim
        self.xlim = xlim
        self.series = []
        self.bands = []
        self.hlines = []
        self.vlines = []

    def plot(self, xs, ys, color=None, label="", width=2.0):
        c = color if color is not None else PALETTE[len(self.series) % len(PALETTE)]
        self.series.append({"x": list(xs), "y": list(ys), "color": c,
                            "label": label, "width": width})
        return self

    def band(self, xs, lo, hi, color, alpha=42, label=""):
        self.bands.append({"x": list(xs), "lo": list(lo), "hi": list(hi),
                           "color": color, "alpha": alpha, "label": label})
        return self

    def hline(self, y, color=(120, 120, 120), width=1.0, label=""):
        self.hlines.append({"y": y, "color": color, "width": width, "label": label})
        return self

    def vline(self, x, color=(150, 150, 150), width=1.0, dash=6):
        self.vlines.append({"x": x, "color": color, "width": width, "dash": dash})
        return self

    def _scale(self):
        xs = [v for s in self.series for v in s["x"] if math.isfinite(v)]
        ys = [v for s in self.series for v in s["y"] if math.isfinite(v)]
        for b in self.bands:
            ys += [v for v in b["hi"] if math.isfinite(v)]
            ys += [v for v in b["lo"] if math.isfinite(v)]
        if not xs:
            xs, ys = [0, 1], [0, 1]
        x0, x1 = (min(xs), max(xs)) if self.xlim is None else self.xlim
        y0, y1 = (min(ys), max(ys)) if self.ylim is None else self.ylim
        if x1 - x0 < 1e-9:
            x1 = x0 + 1.0
        pad = 0.07 * (y1 - y0 if y1 > y0 else 1.0)
        return x0, x1, y0 - pad, y1 + pad

    def draw(self):
        img, d = self.fig.img, self.fig.draw
        x0, y0, x1, y1 = self.box
        d.rectangle([x0, y0, x1, y1], fill=(255, 255, 255))
        dx0, dx1, dy0, dy1 = self._scale()
        f_tick = _font(int(13 * SS))
        f_lab = _font(int(15 * SS))
        f_title = _font(int(17 * SS))
        for t in _nice_ticks(dx0, dx1, 6):
            px = x0 + (t - dx0) / (dx1 - dx0) * (x1 - x0)
            d.line([px, y0, px, y1], fill=(233, 233, 233), width=SS)
            txt = "%g" % t
            w = d.textlength(txt, font=f_tick)
            d.text((px - w / 2, y1 + 7 * SS), txt, font=f_tick, fill=(60, 60, 60))
        for t in _nice_ticks(dy0, dy1, 6):
            py = y1 - (t - dy0) / (dy1 - dy0) * (y1 - y0)
            d.line([x0, py, x1, py], fill=(233, 233, 233), width=SS)
            txt = "%g" % t
            w = d.textlength(txt, font=f_tick)
            d.text((x0 - 9 * SS - w, py - 8 * SS), txt, font=f_tick, fill=(60, 60, 60))

        def px(xv):
            return x0 + (xv - dx0) / (dx1 - dx0) * (x1 - x0)

        def py(yv):
            return y1 - (yv - dy0) / (dy1 - dy0) * (y1 - y0)

        for b in self.bands:
            pts_hi = [(px(x), py(y)) for x, y in zip(b["x"], b["hi"]) if math.isfinite(y)]
            pts_lo = [(px(x), py(y)) for x, y in zip(reversed(b["x"]), reversed(b["lo"]))
                      if math.isfinite(y)]
            if len(pts_hi) > 1 and len(pts_lo) > 1:
                layer = Image.new("RGBA", img.size, (0, 0, 0, 0))
                ld = ImageDraw.Draw(layer)
                ld.polygon(pts_hi + pts_lo, fill=tuple(b["color"]) + (b["alpha"],))
                img2 = Image.alpha_composite(img.convert("RGBA"), layer).convert("RGB")
                self.fig.img = img2
                img = img2
                self.fig.draw = ImageDraw.Draw(img2)
                d = self.fig.draw
        for h in self.hlines:
            d.line([x0, py(h["y"]), x1, py(h["y"])], fill=h["color"], width=int(h["width"] * SS))
        for v in self.vlines:
            x = px(v["x"])
            y = y0
            while y < y1:
                d.line([x, y, x, min(y + v["dash"] * SS, y1)],
                       fill=v["color"], width=int(v["width"] * SS))
                y += (v["dash"] + 5) * SS
        for s in self.series:
            pts = [(px(x), py(y)) for x, y in zip(s["x"], s["y"]) if math.isfinite(y)]
            if len(pts) > 1:
                d.line(pts, fill=s["color"], width=max(1, int(round(s["width"] * SS))),
                       joint="curve")
        d.rectangle([x0, y0, x1, y1], outline=(90, 90, 90), width=SS)
        if self.title:
            w = d.textlength(self.title, font=f_title)
            d.text(((x0 + x1 - w) / 2, y0 - 28 * SS), self.title, font=f_title, fill=(20, 20, 20))
        if self.xlabel:
            w = d.textlength(self.xlabel, font=f_lab)
            d.text(((x0 + x1 - w) / 2, y1 + 26 * SS), self.xlabel, font=f_lab, fill=(40, 40, 40))
        if self.ylabel:
            tw = int(d.textlength(self.ylabel, font=f_lab)) + 4 * SS
            tmp = Image.new("RGB", (tw, 26 * SS), (255, 255, 255))
            ImageDraw.Draw(tmp).text((0, 0), self.ylabel, font=f_lab, fill=(40, 40, 40))
            tmp = tmp.rotate(90, expand=True)
            img.paste(tmp, (int(x0 - 62 * SS), int((y0 + y1 - tmp.size[1]) / 2)))

    def legend(self, pos="tl"):
        items = [(s["color"], s["label"]) for s in self.series if s["label"]]
        items += [(b["color"], b["label"]) for b in self.bands if b["label"]]
        if not items:
            return
        img = self.fig.img
        d = ImageDraw.Draw(img)
        f = _font(int(13 * SS))
        x0, y0, x1, y1 = self.box
        lh = 21 * SS
        wmax = max(d.textlength(t, font=f) for _, t in items)
        bw = wmax + 46 * SS
        bh = lh * len(items) + 12 * SS
        if pos == "tr":
            lx, ly = x1 - bw - 14 * SS, y0 + 12 * SS
        elif pos == "br":
            lx, ly = x1 - bw - 14 * SS, y1 - bh - 12 * SS
        else:
            lx, ly = x0 + 14 * SS, y0 + 12 * SS
        layer = Image.new("RGBA", img.size, (0, 0, 0, 0))
        ld = ImageDraw.Draw(layer)
        ld.rectangle([lx, ly, lx + bw, ly + bh], fill=(255, 255, 255, 215),
                     outline=(175, 175, 175, 255))
        self.fig.img = Image.alpha_composite(img.convert("RGBA"), layer).convert("RGB")
        d = ImageDraw.Draw(self.fig.img)
        for i, (c, t) in enumerate(items):
            yy = ly + 8 * SS + i * lh
            d.line([lx + 8 * SS, yy + 8 * SS, lx + 30 * SS, yy + 8 * SS], fill=c, width=2 * SS)
            d.text((lx + 38 * SS, yy), t, font=f, fill=(30, 30, 30))
        self.fig.draw = d


class Figure:
    def __init__(self, width=1400, height=420, bg=(255, 255, 255)):
        self.size = (int(width * SS), int(height * SS))
        self.img = Image.new("RGB", self.size, bg)
        self.draw = ImageDraw.Draw(self.img)

    def axes(self, rect, **kw):
        W, H = self.size
        box = (int(rect[0] * W), int(rect[1] * H), int(rect[2] * W), int(rect[3] * H))
        return Axes(self, box, **kw)

    def text(self, xy, s, size=14, color=(30, 30, 30)):
        f = _font(int(size * SS))
        self.draw.text((xy[0] * SS, xy[1] * SS), s, font=f, fill=color)

    def save(self, path):
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        out = self.img.resize((self.size[0] // SS, self.size[1] // SS), Image.LANCZOS)
        out.save(path)
        return path
