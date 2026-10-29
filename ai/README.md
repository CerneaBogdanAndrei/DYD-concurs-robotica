# brain — subsistemul de nivel înalt pentru Yahboom DOFBOT (CRI 2026)

Percepție, memorie persistentă, planificare simbolică și orchestrare. Mișcarea brațului o face serviciul
colegilor (TypeScript, HTTP local, `../docs/CONTRACT_BRAT.md`). Brain-ul îl apelează prin
`brain/adapters/arm/team_arm.py`.

## Stare (2026-10-29, Etapa 2)

| Parte | Stare |
|---|---|
| Etapa 0: viziune offline (grilă după linii, ArUco opțional, celule, culoare, yaw, memorie, selftest) | livrată, testată pe date sintetice ([`docs/stage0.md`](docs/stage0.md)) |
| Etapa 2: planificare S1/S2/S3 (Greedy), orchestrare (FSM), `brain sim run`, `brain run`, `report timings`, `calib corners` | implementate, testate în simulare ([`docs/stage2.md`](docs/stage2.md)) |
| Integrare cu serviciul colegilor | testată cu serviciul lor pornit în simulare (`--sim`), pe Pi |
| Previzualizarea camerei în browser (`brain camera preview`, MJPEG + analiză adnotată) | implementată, testată cu cadre sintetice; integrată și în `brain run --preview` |
| Pagina de lucru: ținta S1 din poză sau editată, planul, execuția din pagină (oprită implicit, cu cod) | implementată, testată fără hardware (cadre sintetice, `MockArm`) |
| Gheara printre cuburi: spațiile măsurate de cameră, deschiderea și offset-ul calculate, regulile A/B/C și în serviciu | implementată, testată în simulare; serviciul o aplică doar după `npm run calib -- gheara` |
| Etapa 3: scripturi pentru Pi, plan de test live | **lipsă**: mai jos sunt pașii manuali provizorii |

Foaia folosită la probă: `small` (caroiaj 3×3, celulă 45 × 43 mm, măsurată), cubul de **29 mm** (măsurat).
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

### Sarcinile în simulare (Etapa 2)

```bash
python -m brain sim run --task tower --example --gif out/s2.gif                    # exemplul S2, viziunea reală pe camera sintetică
python -m brain sim run --task tower --order red,yellow,green --target medium:r0c0
python -m brain sim run --task reproduce --example --buffers 2                       # exemplul S1 (cere un punct tampon)
python -m brain sim run --task sort --seed 5 --faults pick:grasp_fail@2
python -m brain sim run --task all --random 20 --perception oracle --buffers 2       # 60 de scenarii, rata de succes
python -m brain report timings --journal out/sim_run/timings.jsonl
```

`--perception oracle` sare peste randare și viziune (rapid); implicit e `render` (camera sintetică + viziunea reală).
Timpii din simulare sunt SIMULAȚI. Detalii, rezultate și limitări: [`docs/stage2.md`](docs/stage2.md).

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
.venv/bin/python -m brain camera preview          # șablonul = primul din planning.active_locations; Ctrl+C oprește
```

Comanda afișează adresa, de ex. `http://192.168.230.100:8080/`. Deschideți-o în browser (laptop sau telefon, în
aceeași rețea). Pagina nu încarcă nimic de pe internet (merge și cu Pi-ul offline), are temă luminoasă/întunecată
după sistem și o singură coloană pe telefon. Se actualizează singură la ~1 s. Panourile:

- **Live**: fluxul camerei, cu conturul fiecărui cub din ultima analiză, laturile lui (mm) și cel mai mic spațiu
  până la un vecin. Culoarea conturului e starea prinderii: verde = pe centru, galben = cu offset, portocaliu =
  degetele încap pe o singură axă, roșu = pe niciuna. Dedesubt: cadre/s, rezoluția, modul analizei, ora ei.
- **Ce vede brain-ul**: imaginea adnotată (spațiile până la vecini în mm, verzi dacă încape degetul, roșii dacă
  nu) și caroiajul: culoarea, încrederea, fața (plină/pictogramă), laturile, „gol” sau „?”. Când grila nu e
  găsită, motivele apar în clar (de ex. „caroiajul atinge marginea imaginii”).
- **Stare**: șablonul activ; serviciul de mișcare (simulare/braț real, pozele și locurile calibrate, ce lipsește,
  dacă gheara e calibrată în mm). Dacă serviciul nu răspunde, pagina scrie asta.
- **Ținta (S1)**: „Încarcă poza țintă” (fișier sau camera telefonului). Poza e analizată cu aceeași detecție, dar
  fără obiectivul camerei de pe braț și fără nadir/înălțime (vederea e tratată ca de sus): fotografiați foaia de
  sus, toată, cu robotul în partea de JOS a pozei (ca din camera brațului). Rezultatul apare ca un caroiaj
  editabil: un click trece prin gol → roșu → galben → verde → albastru; celulele nesigure sunt marcate; butoanele
  ⟲/⟳ rotesc caroiajul. Se poate porni și de la un caroiaj gol sau de la „Copiază ce vede camera”.
  „Confirmă ținta” o salvează în `out/preview/target/target.yaml` (plus poza originală), bună și pentru
  `brain run --task reproduce --target-file out/preview/target/target.yaml`.
- **Plan**: „Calculează planul” = de la ultima observație la ținta confirmată, cu cuburile luate de pe șabloanele
  active și din punctele tampon. Lista de mutări („roșu din r2c0 → r0c1”), deschiderea ghearei la fiecare, durata
  estimată, ce nu se poate face și de ce. Masa deja ca ținta = „nimic de făcut”. **Planul nu mișcă brațul.**
- **Execuție**: există doar cu `preview.allow_execute: true` în `config/system.yaml` (implicit `false`: butonul
  lipsește, iar ruta răspunde 403). Atunci consola Pi-ului afișează la pornire un cod de 8 caractere, introdus o
  dată în pagină (după 10 coduri greșite execuția se blochează până la repornire). Aceleași verificări ca
  `brain run --execute`: `status.sim` trebuie să fie `false` (sau `--service-sim` pentru teste), locurile și pozele
  calibrate, bugetul de timp. O singură execuție odată; „Oprește” = după comanda curentă, fără `home`. Pagina arată
  mutările cu rezultatul lor și, la final, verificarea vizuală față de țintă. Totul ajunge în `out/preview/runs/`.

Adrese directe, utile și pentru `curl`:

| Adresă | Conținut |
|---|---|
| `GET /stream.mjpg` | flux live MJPEG (cu conturul cuburilor; `--no-overlay` îl scoate) |
| `GET /snapshot.jpg`, `GET /annotated.jpg` | ultimul cadru, ultima analiză adnotată |
| `GET /status.json` | fps, rezoluția, analiza (per celulă: culoare, încredere, laturi, spații, prindere), `workbench` |
| `POST /target/image` | poza țintei: corpul = octeții imaginii, `Content-Type: image/*`, maximum 12 MB |
| `GET /target`, `POST /target` | ținta curentă; ținta editată, JSON `{"location": "small", "rows": [[...] x3]}`, valori `red/yellow/green/blue/empty` |
| `GET /target/annotated.jpg` | poza țintei, cu analiza |
| `POST /plan` | planul (JSON) |
| `GET /execute`, `POST /execute`, `POST /execute/token`, `POST /execute/stop` | execuția (codul în antetul `X-Brain-Token`) |

```bash
curl -X POST -H "Content-Type: image/jpeg" --data-binary @tinta.jpg http://<ip-pi>:8080/target/image
curl -X POST http://<ip-pi>:8080/plan
```

Ultimele imagini se salvează și pe disc în `out/preview/` (`latest_raw.jpg`, `latest_annotated.jpg`,
`latest_observation.json`). Portul și frecvența se schimbă în `config/system.yaml`, secțiunea `preview`.
Cu `host: 0.0.0.0`, oricine din aceeași rețea poate vedea camera și poate schimba ținta; doar execuția cere codul.

Ca să le vadă și Claude Code de pe laptop: `curl -o cam.jpg http://<ip-pi>:8080/annotated.jpg`, apoi îi cereți
să citească `cam.jpg` (sau îi dați direct adresa Pi-ului).

Fluxul merge la **30 fps** (maximul camerei: 640×480, MJPG); analiza rulează într-un proces separat, deci nu îl
încetinește. 60 fps nu se poate cu această cameră.

**Camera oblică și obiectivul de 110°: două calibrări care cresc mult precizia**

1. Distorsiunea obiectivului (o singură dată, cu o tablă de șah tipărită, plată, de ex. 10×7 pătrate de 25 mm =
   9×6 colțuri interioare): 10–20 de poze din unghiuri diferite (±30–45°), la distanțe diferite și cu tabla și în
   colțurile imaginii, apoi:
   ```bash
   .venv/bin/python -m brain camera capture --count 1 --out out/tabla/p01     # repetat pentru fiecare poză
   .venv/bin/python -m brain calib camera --frames "out/tabla/*/frame_0.png" --pattern 9x6 --square-mm 25 --write
   ```
   Scrie `config/lens.yaml`; toată viziunea îndreaptă apoi cadrele. O calibrare cu eroare RMS peste 1 px e refuzată.
2. Poziția camerei în poza de observare (`config/system.yaml`, la șablonul activ): `camera_height_mm` = înălțimea
   camerei peste masă, măsurată cu rigla; `nadir_uv` = punctul de pe masă de sub cameră, în celule (`u` = coloană,
   `v` = rând; poate fi în afara caroiajului). Cu ele, un cub e atribuit celulei după bază, nu după fața de sus
   (care, văzută oblic, pare împinsă spre celula de mai departe), iar fețele laterale sunt ignorate.

Cadre pentru analiză offline:

```bash
.venv/bin/python -m brain camera capture --count 5 --out out/live
.venv/bin/python -m brain vision analyze --frames "out/live/frame_*.png" --location small --out out/live_an
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
npm run calib -- gheara 135                   # gheara: doar servo 6 la unghi; măsurați deschiderea INTERIOARĂ (mm)
npm run calib -- gheara 110                   # repetați pentru 3-4 unghiuri care acoperă 29..45 mm
npm run calib -- gheara 90
npm run calib -- gheara                       # tabelul; cu cel puțin 2 puncte, regulile A și C sunt active
npm run calib -- poza observe_small           # foaia folosită: small
npm run calib -- celule small                 # r0c0, r0c2, r2c0, r2c2
npm run calib -- zona zone_red                # S3: zone_red, zone_yellow, zone_green, zone_blue
npm run calib -- zona buffer_1                # opțional: puncte tampon (S1), trecute și în planning.buffer_points
npm run calib -- test small 1 1
```

### 6. Brain → serviciu

Doar citire de stare, fără mișcare. Trebuie să afișeze `sim: False` și `mode: hardware`:

```bash
.venv/bin/python -c "from brain.adapters.arm.team_arm import TeamArm; a = TeamArm(); print(a.connect()); print(a.mode)"
```

Prima mișcare se face cu mâna pe întrerupătorul de alimentare, după calibrare:

```bash
.venv/bin/python -c "from brain.adapters.arm.team_arm import TeamArm; a = TeamArm(); a.connect(); a.go_pose('observe_small', 'slow')"
```

Cu `camera preview` pornit într-o altă consolă, vedeți în browser ce vede camera din această poză.

### 7. Sarcinile pe braț (`brain run`)

Fără `--execute`, brain-ul doar observă (camera din poziția curentă) și afișează planul; brațul nu se mișcă:

```bash
.venv/bin/python -m brain run --task tower --order red,yellow,green,blue,red --target small:r1c1 --preview
.venv/bin/python -m brain run --task memorize --target-out out/s1_target.yaml        # S1, faza 1: memorează aranjamentul
.venv/bin/python -m brain run --task reproduce --target-file out/s1_target.yaml
.venv/bin/python -m brain run --task sort
```

Cu `--execute` brațul se mișcă, după o confirmare în consolă (scrieți `da`). Serviciul trebuie să raporteze
`sim: false`, iar `small`, `observe_small` (și zonele, la S3) trebuie să fie calibrate. Ctrl+C oprește după
comanda curentă, fără `home`. Rezultatele (raport, durate, memorie, log) ajung în `out/run-<data>/`.
Duratele măsurate devin valori pentru `config/system.yaml` cu `python -m brain report timings --journal out/run-*/timings.jsonl`.

## Documente

- [`docs/step1-analysis.md`](docs/step1-analysis.md): analiza tehnică și arhitectura
- [`docs/stage0.md`](docs/stage0.md): Etapa 0 (viziune offline)
- [`docs/stage2.md`](docs/stage2.md): Etapa 2 (planificare, orchestrare, simularea sarcinilor)
- [`docs/sesiune-2026-10-29-pi.md`](docs/sesiune-2026-10-29-pi.md): jurnalul sesiunii pe Pi (instalare, braț, calibrare, ce urmează)
- [`docs/calibrare-2026-10-29.md`](docs/calibrare-2026-10-29.md): calibrarea pe braț (unghiuri, teste, potrivirea L1/L4)
- [`docs/questions-for-kinematics-team.md`](docs/questions-for-kinematics-team.md): constatări și întrebări pentru serviciul de mișcare
