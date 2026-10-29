"""Intersecțiile caroiajului 3x3 (4x4 = 16 puncte), cu precizie sub-pixel, pornind de la colțurile exterioare."""
import cv2
import numpy as np

UV = np.float32([[0, 0], [3, 0], [3, 3], [0, 3]])


def _bilinear(g, pts):
    x, y = pts[..., 0], pts[..., 1]
    x0, y0 = np.floor(x).astype(int), np.floor(y).astype(int)
    h, w = g.shape
    x0 = np.clip(x0, 0, w - 2); y0 = np.clip(y0, 0, h - 2)
    fx, fy = x - x0, y - y0
    return (g[y0, x0] * (1 - fx) * (1 - fy) + g[y0, x0 + 1] * fx * (1 - fy)
            + g[y0 + 1, x0] * (1 - fx) * fy + g[y0 + 1, x0 + 1] * fx * fy)


def intersections(img_bgr, corners_px, half=10.0):
    g = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY).astype(np.float64)
    g = cv2.GaussianBlur(g, (3, 3), 0)
    H = cv2.getPerspectiveTransform(UV, np.float32(corners_px))
    proj = lambda uv: cv2.perspectiveTransform(np.float64(uv).reshape(-1, 1, 2), H).reshape(-1, 2)
    s = np.arange(-half, half + 0.01, 0.5)
    fits = {}
    for vertical in (True, False):
        for k in range(4):
            ts = np.linspace(0.06, 2.94, 60)
            ts = ts[np.abs(ts - np.round(ts)) > 0.12]
            uv = np.stack([np.full_like(ts, k), ts], 1) if vertical else np.stack([ts, np.full_like(ts, k)], 1)
            uv2 = uv + (np.array([0, 0.01]) if vertical else np.array([0.01, 0]))
            p, p2 = proj(uv), proj(uv2)
            d = (p2 - p); d /= np.linalg.norm(d, axis=1, keepdims=True)
            n = np.stack([-d[:, 1], d[:, 0]], 1)
            samp = p[:, None, :] + s[None, :, None] * n[:, None, :]
            prof = _bilinear(g, samp)
            base = np.median(np.concatenate([prof[:, :4], prof[:, -4:]], 1), axis=1, keepdims=True)
            dark = np.clip(base - prof, 0, None)
            peak = dark.max(1)
            ok = peak > 25
            w = np.where(dark >= 0.3 * peak[:, None], dark, 0)
            off = (w * s).sum(1) / np.maximum(w.sum(1), 1e-9)
            pts = (p + off[:, None] * n)[ok]
            if len(pts) < 15:
                raise ValueError(f"linia {'u' if vertical else 'v'}={k}: doar {len(pts)} puncte")
            a, b = (0, 1) if vertical else (1, 0)  # vertical: x = f(y); orizontal: y = f(x)
            coef = np.polyfit(pts[:, b], pts[:, a], 2)
            res = pts[:, a] - np.polyval(coef, pts[:, b])
            keep = np.abs(res) < max(3 * res.std(), 0.5)
            coef = np.polyfit(pts[keep, b], pts[keep, a], 2)
            res = pts[keep, a] - np.polyval(coef, pts[keep, b])
            fits[(vertical, k)] = (coef, float(np.sqrt((res ** 2).mean())))
    out = np.zeros((4, 4, 2))  # [j = linia v, i = coloana u]
    for j in range(4):
        for i in range(4):
            x, y = proj([[i, j]])[0]
            cv_, ch = fits[(True, i)][0], fits[(False, j)][0]
            for _ in range(20):
                x = np.polyval(cv_, y); y = np.polyval(ch, x)
            out[j, i] = (x, y)
    rms = max(f[1] for f in fits.values())
    return out, rms
