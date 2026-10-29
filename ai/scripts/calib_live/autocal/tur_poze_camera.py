"""Turul de poze: brațul se mișcă, previzualizarea analizează caroiajul, se salvează cele 16 intersecții."""
import json, sys, time, math, urllib.request
from datetime import datetime
import cv2, numpy as np
sys.path.insert(0, sys.argv[1]); from detect import intersections
OUT = sys.argv[1] + "/data"; import os; os.makedirs(OUT, exist_ok=True)
SVC, PREV = "http://127.0.0.1:8765", "http://127.0.0.1:8080"
OBS = str(__import__("pathlib").Path(__file__).resolve().parents[3] / "out/preview/latest_observation.json")
L1, L2, L3, L4 = 104.5, 82.85, 82.85, 183.85

def post(m, body=None):
    r = urllib.request.Request(f"{SVC}/{m}", data=json.dumps(body or {}).encode(), method="POST")
    return json.load(urllib.request.urlopen(r, timeout=30))

def tip_z(s):
    a2 = math.radians(s[1]); a3 = math.radians(s[1] + s[2] - 90); a4 = math.radians(s[1] + s[2] + s[3] - 180)
    return L1 + L2 * math.sin(a2) + L3 * math.sin(a3) + L4 * math.sin(a4)

POSES = [
    [90, 90, 29, 3, 90], [84, 90, 29, 3, 90], [96, 90, 29, 3, 90], [90, 90, 29, 10, 90], [90, 90, 29, 16, 90],
    [90, 86, 29, 8, 90], [90, 94, 29, 3, 90], [90, 90, 35, 3, 90], [90, 90, 24, 10, 90], [90, 90, 29, 3, 78],
    [90, 90, 29, 3, 102], [85, 92, 32, 6, 82], [95, 88, 26, 9, 98], [87, 94, 24, 12, 95], [93, 86, 33, 4, 85],
    [90, 96, 35, 0, 90], [80, 92, 30, 8, 90], [100, 92, 30, 8, 90],
]
post("beep", {"n": 3}); time.sleep(1.5)
results = []
for k, s in enumerate(POSES):
    z = tip_z(s)
    if z < 45:
        print(f"{k:2d} {s} sar: vârful ar fi la z={z:.0f} mm"); continue
    post("move_joints", {"angles": s, "time_ms": 1500})
    time.sleep(1.2)
    t0 = time.time()
    rd = post("read_joints")["result"]
    obs = None
    for _ in range(40):
        time.sleep(0.25)
        d = json.load(open(OBS))
        ts = datetime.strptime(d["captured_at"], "%Y-%m-%dT%H:%M:%SZ").replace(tzinfo=__import__("datetime").timezone.utc).timestamp()
        if ts >= t0:
            obs = d; break
    if obs is None or not obs["actionable"]:
        print(f"{k:2d} {s} citit {rd}: caroiajul nu e complet ({obs and obs['reasons']})"); continue
    img = cv2.imdecode(np.frombuffer(urllib.request.urlopen(f"{PREV}/snapshot.jpg", timeout=10).read(), np.uint8), 1)
    try:
        pts, rms = intersections(img, obs["geometry"]["corners_px"])
    except ValueError as e:
        print(f"{k:2d} {s}: detectare eșuată: {e}"); continue
    cv2.imwrite(f"{OUT}/pose_{k:02d}.png", img)
    results.append({"k": k, "cmd": s, "read": rd, "pts": pts.tolist(), "line_rms": rms, "tip_z": z})
    print(f"{k:2d} {s} citit {rd}: 16 puncte, linii {rms:.2f} px, vârf z~{z:.0f} mm")
json.dump(results, open(f"{OUT}/views.json", "w"), indent=1)
post("go_pose", {"name": "observe_small", "speed": "slow"})
print("vederi bune:", len(results))
