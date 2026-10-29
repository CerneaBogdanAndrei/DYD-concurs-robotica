"""ÎNCERCARE NEREUȘITĂ (2026-10-29), păstrată ca referință: rezidual 7 px, soluții nefizice. Rezolvare constrânsă: foaia orizontală (doar x, y, rotire, înălțime), camera fixă pe segmentul servo 4, unghiuri citite."""
import json, sys
import numpy as np

L = [104.5, 82.85, 82.85, 183.85]
COL, ROW = 45.0, 43.0
GRID = np.array([[COL * i, ROW * j, 0.0] for j in range(4) for i in range(4)])
views = json.load(open(sys.argv[1]))
OBSV = [np.array(v["pts"]).reshape(-1, 2) for v in views]
Q = [np.array(v["read"][:5], float) for v in views]

def rodr(r):
    th = np.linalg.norm(r)
    if th < 1e-12: return np.eye(3)
    k = r / th; K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * K @ K

def link_frame(s, d=(0, 0, 0, 0), Ls=L):
    L1, L2, L3, _ = Ls
    th = np.radians(s[0] - 90 + d[0])
    a2 = np.radians(s[1] + d[1]); a3 = a2 + np.radians(s[2] + d[2] - 90); a4 = a3 + np.radians(s[3] + d[3] - 90)
    rw = L2 * np.cos(a2) + L3 * np.cos(a3); zw = L1 + L2 * np.sin(a2) + L3 * np.sin(a3)
    W = np.array([rw * np.cos(th), rw * np.sin(th), zw])
    ez = np.array([np.cos(a4) * np.cos(th), np.cos(a4) * np.sin(th), np.sin(a4)])
    ey = np.array([-np.sin(th), np.cos(th), 0.0]); ex = np.cross(ey, ez)
    return np.column_stack([ex, ey, ez]), W

# p: f, cx, cy, k1, k2 | X rot(3) t(3) | grid x, y, yaw, z | d1..d4
NAMES = "f cx cy k1 k2 rx ry rz tx ty tz gx gy gyaw gz d1 d2 d3 d4".split()
def project(p):
    f, cx, cy, k1, k2 = p[:5]
    Rx, tx = rodr(p[5:8]), p[8:11]
    gx, gy, gyaw, gz = p[11:15]
    c, s = np.cos(gyaw), np.sin(gyaw)
    Rg = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]) @ np.diag([1, -1, -1.0])  # foaia: coloane x, linii y, z în jos
    Pw = GRID @ Rg.T + np.array([gx, gy, gz])
    out = []
    for q in Q:
        RF, W = link_frame(q, p[15:19])
        Rc = RF @ Rx; tc = RF @ tx + W
        pc = (Pw - tc) @ Rc
        x, y = pc[:, 0] / pc[:, 2], pc[:, 1] / pc[:, 2]
        r2 = x * x + y * y; fd = 1 + k1 * r2 + k2 * r2 * r2
        out.append(np.column_stack([f * x * fd + cx, f * y * fd + cy]))
    return out

def residuals(p):
    return np.concatenate([(a - b).ravel() for a, b in zip(project(p), OBSV)])

def lm(p, free, iters=300):
    p = p.copy(); lam = 1e-3
    r = residuals(p); cost = r @ r
    for _ in range(iters):
        J = np.zeros((len(r), len(free)))
        for c, i in enumerate(free):
            h = 1e-6 * max(1.0, abs(p[i])); pp = p.copy(); pp[i] += h
            J[:, c] = (residuals(pp) - r) / h
        A = J.T @ J; g = J.T @ r
        while True:
            step = np.linalg.solve(A + lam * np.diag(np.diag(A) + 1e-9), -g)
            pn = p.copy(); pn[free] += step
            rn = residuals(pn); cn = rn @ rn
            if cn < cost:
                p, r, cost, lam = pn, rn, cn, lam / 3; break
            lam *= 5
            if lam > 1e12: break
        if lam > 1e12 or np.abs(step).max() < 1e-9: break
    rms = np.sqrt(cost / (len(r) / 2))
    try: sd = np.sqrt(np.diag(np.linalg.inv(J.T @ J)) * rms ** 2)
    except np.linalg.LinAlgError: sd = np.full(len(free), np.nan)
    return p, rms, dict(zip([NAMES[i] for i in free], sd))

if __name__ == "__main__":
    # start: camera privește de-a lungul ghearei, imaginea „sus” = departe; foaia ~ la 250-350 mm în față
    Rx0 = np.column_stack([[0, -1, 0], [1, 0, 0], [0, 0, 1]]).astype(float)
    import cv2
    best = None
    for f0 in (600, 700, 800):
        K = np.array([[f0, 0, 320], [0, f0, 240], [0, 0, 1.0]])
        ok, rv, tv = cv2.solvePnP(GRID, OBSV[0], K, None)
        Rcg = cv2.Rodrigues(rv)[0]
        for tx0 in ([-40, 0, 40], [40, 0, 40], [-50, 0, -10], [0, 0, 80], [0, 40, 40], [0, -40, 40]):
            RF, W = link_frame(Q[0])
            Rbc, tbc = RF @ Rx0, RF @ np.array(tx0, float) + W
            o = Rbc @ tv.ravel() + tbc
            xa = Rbc @ Rcg[:, 0]
            p0 = np.array([f0, 320, 240, 0, 0, *cv2.Rodrigues(Rx0)[0].ravel(), *tx0, o[0], o[1], np.arctan2(xa[1], xa[0]), o[2], 0, 0, 0, 0], float)
            p1, e1, _ = lm(p0, [5, 6, 7, 8, 9, 10, 11, 12, 13, 14])
            p2, e2, sd = lm(p1, [0, 3, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14])
            if best is None or e2 < best[1]: best = (p2, e2, sd)
    p, e, sd = best
    print(f"A) foaia orizontală, înălțimea liberă, centrul optic fix: RMS {e:.2f} px")
    print("   " + "  ".join(f"{k} {p[NAMES.index(k)] if k not in ('gyaw',) else np.degrees(p[13]):.3g}±{v if k!='gyaw' else np.degrees(v):.2g}" for k, v in sd.items()))
    pB, eB, sdB = lm(p, [0, 1, 2, 3, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14])
    print(f"B) + centrul optic liber: RMS {eB:.2f} px; f {pB[0]:.0f} c ({pB[1]:.0f},{pB[2]:.0f}) k1 {pB[3]:.3f}; masa z {pB[14]:.1f}±{sdB['gz']:.1f}")
    pC, eC, sdC = lm(pB, [0, 1, 2, 3, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 16, 17, 18])
    print(f"C) + zero-urile servo 2-4: RMS {eC:.2f} px; d2 {pC[16]:+.1f}±{sdC['d2']:.1f} d3 {pC[17]:+.1f}±{sdC['d3']:.1f} d4 {pC[18]:+.1f}±{sdC['d4']:.1f}; masa z {pC[14]:.1f}")
    for name, pp in (("A", p), ("B", pB), ("C", pC)):
        errs = [np.sqrt(((a - b) ** 2).sum(1).mean()) for a, b in zip(project(pp), OBSV)]
        c, s = np.cos(pp[13]), np.sin(pp[13])
        cell = lambda r, cc: np.array([pp[11], pp[12]]) + np.array([[c, -s], [s, c]]) @ np.array([COL * (cc + .5), -ROW * (r + .5)])
        print(f"{name}: camera t={np.round(pp[8:11],1)} mm; foaia: rotire {np.degrees(pp[13]):.1f} grd, masa z {pp[14]:.1f}; "
              f"r0c0 {np.round(cell(0,0),1)} r1c1 {np.round(cell(1,1),1)} r2c2 {np.round(cell(2,2),1)}; RMS/vedere " + " ".join(f"{x:.1f}" for x in errs))
    np.save(sys.argv[2] + "/p2.npy", np.stack([p, pB, pC]))
