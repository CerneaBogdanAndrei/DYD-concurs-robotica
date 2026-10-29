"""Monitor pentru poza de observare: citește impulsurile servo 1..5 (doar citire) și starea viziunii; bip când e bine."""
import json, subprocess, sys, time, urllib.request

LIMITS = {1: (900, 3100), 2: (900, 3100), 3: (900, 3100), 4: (900, 3100), 5: (380, 3700)}
MARGIN = 40  # impuls; puțin în interiorul intervalului, ca o mică mișcare să nu-l scoată afară

def pulse(i):
    reg = 0x30 + i
    subprocess.run(["i2cset", "-y", "1", "0x15", hex(reg), "0x00"], check=True, capture_output=True)
    time.sleep(0.01)
    raw = int(subprocess.run(["i2cget", "-y", "1", "0x15", hex(reg), "w"], check=True, capture_output=True, text=True).stdout, 16)
    return ((raw & 0xFF) << 8) | (raw >> 8)

def vision():
    try:
        s = json.loads(urllib.request.urlopen("http://127.0.0.1:8080/status.json", timeout=2).read())
        a = s.get("analysis") or {}
        return bool(a.get("actionable")), ",".join(a.get("reasons") or [])
    except Exception as exc:
        return False, str(exc)

end = time.time() + float(sys.argv[1] if len(sys.argv) > 1 else 300)
good_streak, last_beep = 0, 0.0
while time.time() < end:
    try:
        p = {i: pulse(i) for i in LIMITS}
    except Exception as exc:
        print("citire i2c eșuată:", exc, flush=True); time.sleep(0.5); continue
    bad = [i for i, v in p.items() if not (LIMITS[i][0] + MARGIN <= v <= LIMITS[i][1] - MARGIN)]
    ok_v, why = vision() if "--fara-viziune" not in sys.argv else (True, "")
    good_streak = good_streak + 1 if (not bad and ok_v) else 0
    print("{} impulsuri {} | în afara: {} | viziune {} {}".format(time.strftime("%H:%M:%S"), p, bad or "-", "OK" if ok_v else "NU", why), flush=True)
    if good_streak >= 2 and time.time() - last_beep > 2.0:
        subprocess.run(["i2cset", "-y", "1", "0x15", "0x06", "1"], capture_output=True)
        last_beep = time.time()
    time.sleep(0.3)
