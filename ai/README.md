# brain — subsistemul de nivel înalt pentru Yahboom DOFBOT (CRI 2026)

Percepție, memorie persistentă, planificare simbolică și orchestrare. Mișcarea brațului o face serviciul
colegilor (TypeScript, HTTP local, `../docs/CONTRACT_BRAT.md`). Brain-ul îl apelează prin
`brain/adapters/arm/team_arm.py`.

## Stare (2026-10-29)

| Parte | Stare |
|---|---|
| Etapa 0: viziune offline (grilă după linii, ArUco opțional, celule, culoare, yaw, memorie, selftest) | livrată, testată pe date sintetice ([`docs/stage0.md`](docs/stage0.md)) |
| Etapa 2: config v2 (3 șabloane, braț, proximitate, timpi, planificare), `MockArm`, `TeamArm`, `SimWorld`, randare cu paralaxă | implementate; teste automate încă lipsă |
| Etapa 2: planificare S1/S2/S3, orchestrare (FSM), `brain sim run`, `brain run` | **lipsă**: nicio sarcină nu rulează încă cap-coadă |
| Previzualizarea camerei în browser (`brain camera preview`, MJPEG + analiză adnotată) | implementată, testată cu cadre sintetice |
| Etapa 3: scripturi pentru Pi, plan de test live | **lipsă**: mai jos sunt pașii manuali provizorii |

Nimic din brain nu a fost testat pe braț sau pe camera reală.

## Pe laptop (Windows)

```powershell
cd ai
python -m venv .venv
.venv\Scripts\python -m pip install -e ".[dev]"
$env:PYTHONUTF8=1                                   # diacritice corecte în consolă

.venv\Scripts\python -m pytest -q                   # ~45 s
.venv\Scripts\python -m brain config validate
.venv\Scripts\python -m brain selftest stage0 --seed 42 --scenes 200 --out out/selftest   # ~75 s, exit 0 = trecut
.venv\Scripts\python -m brain sim generate --seed 3 --count 2 --out out/synth
.venv\Scripts\python -m brain vision analyze --frames "out/synth/scene_3/frame_*.png" --location medium --out out/an
```

Pe Linux / Pi, înlocuiți `.venv\Scripts\python` cu `.venv/bin/python`.

### Serviciul de mișcare în simulare + `TeamArm`

Din rădăcina repo-ului (Node ≥ 20):

```bash
npm install && npm run sim          # server SIMULAT pe http://127.0.0.1:8765, cu sim/calib.json
```

Apoi, din `ai/`, într-o altă consolă:

```bash
.venv/bin/python -c "from brain.adapters.arm.team_arm import TeamArm; a = TeamArm(); print(a.connect()); print(a.mode)"
.venv/bin/python -c "from brain.adapters.arm.team_arm import TeamArm; a = TeamArm(); a.connect(); print(a.pick('source', (1, 1), yaw_deg=10)); print(a.place('dest', (1, 1))); a.home()"
```

`npm run sim` nu modelează scena și răspunde instantaneu: verifică doar contractul, nu durata mișcărilor.

## Pe Raspberry Pi (pași manuali provizorii, până la scripturile din Etapa 3)

### 1. Copierea proiectului

Cu internet pe Pi:

```bash
git clone https://github.com/CerneaBogdanAndrei/DYD-concurs-robotica.git
cd DYD-concurs-robotica
git checkout <branch-ul publicat>          # sau main, după îmbinare
```

Fără internet: pe laptop rulați `git archive --format=zip -o dyd.zip HEAD` în rădăcina repo-ului, apoi
`scp dyd.zip pi@<ip-pi>:~` și pe Pi `unzip dyd.zip -d DYD-concurs-robotica`.

### 2. Diagnostic (trimiteți-ne rezultatul)

```bash
python3 --version; cat /etc/os-release; uname -m
node -v; npm -v
ls /dev/video*; i2cdetect -y 1             # placa brațului trebuie să apară la 0x15
df -h ~; vcgencmd measure_temp
systemctl list-units --type=service | grep -i -E "yahboom|jupyter|ros|dofbot"
```

### 3. Python (brain)

Brain-ul cere **Python ≥ 3.8**. Pe Python 3.7 (de ex. Raspberry Pi OS Buster) trebuie instalat întâi un Python mai nou.

```bash
cd ~/DYD-concurs-robotica/ai
python3 -m venv .venv
.venv/bin/python -m pip install -U pip
.venv/bin/python -m pip install -e ".[dev]"
```

- `uname -m` = `aarch64`: `opencv-contrib-python` are wheel-uri pe PyPI.
- `armv7l` (32 de biți): wheel-urile vin de pe piwheels (configurat implicit în Raspberry Pi OS). Dacă pip
  începe să compileze OpenCV (durează ore), întrerupeți cu Ctrl+C și folosiți OpenCV din apt:

  ```bash
  sudo apt install -y python3-opencv python3-numpy python3-yaml
  rm -rf .venv && python3 -m venv --system-site-packages .venv
  .venv/bin/python -m pip install --no-deps -e . && .venv/bin/python -m pip install pytest
  ```

  Fără modulul `aruco` merge doar metoda principală (linii), cea folosită implicit.

Verificare:

```bash
.venv/bin/python -m pytest -q
.venv/bin/python -m brain config validate
.venv/bin/python -m brain selftest stage0 --seed 42 --scenes 50 --out out/selftest   # notați durata
```

### 4. Camera (fără braț): ce vede și ce detectează

Previzualizare live în browser (Pi-ul nu are ecran):

```bash
.venv/bin/python -m brain camera preview --location medium      # Ctrl+C oprește
```

Comanda afișează adresa, de ex. `http://192.168.1.50:8080/`. Deschideți-o în browserul laptopului (aceeași rețea).
Pagina arată fluxul live, ultima analiză adnotată (grila găsită, culoarea, fața și yaw-ul fiecărei celule,
refăcută la fiecare secundă) și rezumatul în text. Adrese directe, utile și pentru `curl`:

| Adresă | Conținut |
|---|---|
| `/stream.mjpg` | flux live MJPEG |
| `/snapshot.jpg` | ultimul cadru |
| `/annotated.jpg` | ultima analiză adnotată |
| `/status.json` | fps, motivele de respingere, câte o linie per celulă |

Ultimele imagini se salvează și pe disc în `out/preview/` (`latest_raw.jpg`, `latest_annotated.jpg`,
`latest_observation.json`). Portul și frecvența se schimbă în `config/system.yaml`, secțiunea `preview`.
Cu `host: 0.0.0.0`, oricine din aceeași rețea poate vedea camera (doar citire).

Ca să le vadă și Claude Code de pe laptop: `curl -o cam.jpg http://<ip-pi>:8080/annotated.jpg`, apoi îi cereți
să citească `cam.jpg` (sau îi dați direct adresa Pi-ului).

Cadre pentru analiză offline:

```bash
.venv/bin/python -m brain camera capture --count 5 --out out/live
.venv/bin/python -m brain vision analyze --frames "out/live/frame_*.png" --location medium --out out/live_an
```

Fără cameră, previzualizarea merge și pe fișiere:
`python -m brain camera preview --frames "out/synth/scene_3/frame_*.png"`.
Pragurile HSV și dimensiunile celulelor sunt NECALIBRATE. Camera USB nu a fost încă testată pe hardware.
Previzualizarea ține camera ocupată: opriți-o înainte de orice altă comandă care folosește camera.

### 5. Serviciul de mișcare pe braț

Pașii completi sunt în `../README.md` (I2C activat, `build-essential`, serviciile Yahboom oprite). Pe scurt:

```bash
cd ~/DYD-concurs-robotica
npm install                 # npm ≥ 12 poate bloca compilarea i2c-bus: npm install-scripts approve i2c-bus
npm run build && npm test
npm run server              # braț REAL; lăsați-l pornit
```

Calibrați cu numele folosite de brain (`config/system.yaml`):

```bash
npm run calib -- poza observe_medium          # la fel pentru small și large
npm run calib -- celule medium                # r0c0, r0c2, r2c0, r2c2
npm run calib -- zona zone_red                # S3: zone_red, zone_yellow, zone_green, zone_blue
npm run calib -- test medium 1 1
```

### 6. Brain → serviciu

Doar citire de stare, fără mișcare. Trebuie să afișeze `sim: False` și `mode: hardware`:

```bash
.venv/bin/python -c "from brain.adapters.arm.team_arm import TeamArm; a = TeamArm(); print(a.connect()); print(a.mode)"
```

Prima mișcare se face cu mâna pe întrerupătorul de alimentare, după calibrare:

```bash
.venv/bin/python -c "from brain.adapters.arm.team_arm import TeamArm; a = TeamArm(); a.connect(); a.go_pose('observe_medium', 'slow')"
```

Cu `camera preview` pornit într-o altă consolă, vedeți în browser ce vede camera din această poză.
Rularea sarcinilor (`brain run`) nu există încă.

## Documente

- [`docs/step1-analysis.md`](docs/step1-analysis.md): analiza tehnică și arhitectura
- [`docs/stage0.md`](docs/stage0.md): Etapa 0 (viziune offline)
- [`docs/questions-for-kinematics-team.md`](docs/questions-for-kinematics-team.md): constatări și întrebări pentru serviciul de mișcare
