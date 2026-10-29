# Prompt pentru sesiunea următoare — proiectul „Brain” (CRI 2026)

> **Actualizare 2026-10-29 seara (au prioritate față de restul textului):** senzorul de proximitate (HC-SR04) a fost
> ELIMINAT din cod și din planuri; brațul e recalibrat pe foaia MICĂ (medium nu încăpea în cameră); previzualizarea
> merge la 30 fps (MJPG); planificarea penalizează vecinii ocupați (`adjacent_cube_penalty_deg`). Problema principală
> pe braț: gheara coboară deschisă la `GRIP_OPEN` = 60 și lovește cuburile alăturate (constatările 12–13 din
> `questions-for-kinematics-team.md`). Etapa 2 e implementată (vezi `stage2.md`, `sesiune-2026-10-29-pi.md`).

Continuă proiectul „Brain” pentru brațul Yahboom DOFBOT (concursul CRI 2026). Răspunde-mi în română.

## 0. Reguli de lucru (obligatorii)

- Codul tău stă DOAR în `ai/` din `C:\Users\theda\PycharmProjects\DYD-concurs-robotica` (pachetul Python `brain`).
  Restul repo-ului (TypeScript: `src/`, `test/`, `clients/`, `referinta-python/`, `sim/`, `scripts/`, `docs/CONTRACT_BRAT.md`,
  `package.json`, `.gitignore` din rădăcină) e al colegilor: îl citești și îl rulezi, dar NU îl modifici. Orice bug sau
  lipsă la ei o notezi în `ai/docs/questions-for-kinematics-team.md` (secțiunea „Constatări”).
- Livrare pe etape: la finalul unei etape te oprești, raportezi cifre reale și aștepți confirmarea mea explicită.
- Cod: identificatori în engleză; docstring-uri, comentarii și mesaje în română. `cv2.putText` doar ASCII.
  Densitatea comentariilor ca în codul existent.
- Python 3.8+: `from __future__ import annotations`, `typing.List/Dict/Optional`, fără `match`, fără `X | Y` la rulare,
  fără `dict | dict`, fără `str.removeprefix`. `tests/test_architecture.py` verifică sintaxa 3.8 și că nimic nu importă
  `Arm_Lib/smbus/serial/RPi/spidev/gpiozero/pigpio` în afara adaptoarelor desemnate (`HARDWARE_ALLOWED`; dacă adaugi un
  adaptor HC-SR04 pe GPIO, îl treci explicit acolo).
- Nu inventa valori fizice: tot ce nu e măsurat se marchează NECALIBRAT / `calibrated: false`.
- Nu declara nimic verificat pe hardware fără test pe hardware. Nu declara teste trecute fără să le rulezi; raportează cifrele.
- Git: lucrezi pe branch-ul `brain-etapa2`; la final îl aduci în `main` prin fast-forward și faci push la amândouă
  (am cerut eu asta). Înainte de orice `git pull`: backup zip pentru `ai/` (fără `.venv`, `out/`, `__pycache__`) în
  `C:\Users\theda\PycharmProjects\DYD-backups\`, apoi `git fetch` și verifică dacă se poate face fast-forward.
  **Repo-ul e PUBLIC** (API-ul GitHub răspunde 200 fără autentificare): nu urca secrete, adrese de e-mail sau date personale.
- Consola Windows: `$env:PYTHONUTF8=1`. Commit-urile se termină cu linia `Co-Authored-By` cerută de sistem.

## 1. Mediul

- Windows 11, Python 3.12, `ai/.venv` (numpy 2.5.4, opencv-contrib-python 5.0.0, PyYAML 6.0.3, pytest 9.2.0, uv). Pachet instalat editabil.
- Node 24.18, npm 12.0.1. `gh` NU e instalat (nici în PowerShell, nici în Git Bash). Push-ul cere autentificare prin
  fereastra „Connect to GitHub” (Git Credential Manager), care așteaptă utilizatorul: rulează push-ul în fundal și anunță-mă.
- Comenzi (din `ai/`):
  ```
  .venv\Scripts\python -m pytest -q                                   # 226 teste, ~40 s
  .venv\Scripts\python -m brain config validate
  .venv\Scripts\python -m brain selftest stage0 --seed 42 --scenes 200 --out out/selftest          # medium implicit
  .venv\Scripts\python -m brain selftest stage0 --seed 42 --scenes 200 --difficulty hard --out out/selftest_hard
  .venv\Scripts\python -m brain selftest stage0 --seed 42 --scenes 200 --location small --out out/selftest_small
  .venv\Scripts\python -m brain sim generate --seed 3 --count 2 --out out/synth
  .venv\Scripts\python -m brain vision analyze --frames "out/synth/scene_3/frame_*.png" --location medium --out out/an
  .venv\Scripts\python -m brain camera preview --frames "out/synth/scene_3/frame_*.png" --host 127.0.0.1 --port 8081 --duration 25
  ```
- Serviciul colegilor în simulare (din rădăcina repo-ului): `npm install; npm run build; node dist/src/cli/server.js --sim --port 8765`
  (opțional `--calib <fișier>` pentru alte locuri decât `sim/calib.json`). Testele lor: `node --test "dist/test/**/*.test.js"` (53/53).
- Python 3.8 + OpenCV 4.6 (într-un folder temporar din afara proiectului):
  ```
  .venv\Scripts\python -m uv venv --python 3.8 <TEMP>\venv38
  .venv\Scripts\python -m uv pip install --python <TEMP>\venv38\Scripts\python.exe numpy==1.24.4 opencv-contrib-python==4.6.0.66 PyYAML==6.0.1 pytest==8.3.3
  $env:PYTHONPATH=(Get-Location).Path; <TEMP>\venv38\Scripts\python.exe -m pytest -q -p no:cacheprovider     # din ai/
  ```
- Sonde de diagnostic: `ai/scripts/probes/` (vezi README-ul de acolo).

## 2. Starea reală la 2026-10-29 (commit `e65eaf5` + commit-ul cu acest prompt, pe `main` și `brain-etapa2`)

### Ce există și e testat
- **Etapa 0, viziune offline:** detecția grilei după linii (ArUco opțional), redresare cu margine de 0,3 celule, analiza
  celulelor (ocupare, culoare din inel, yaw, față plină/pictogramă, cuburi lipite, blob-uri tăiate), `uv_top`, SceneState
  schema 2 (cu zone) și scriere atomică, generator sintetic, CLI, selftest.
- **Corecție nouă de detecție** (`grid_lines.py`, `pipeline.py`): a doua trecere de rafinare, plus cea mai bună potrivire per
  grilă (reziduurile la sub 10% unul de altul sunt egale; decide scorul de structură, apoi ordinea de generare). Teste de
  regresie: `test_best_fit_wins_among_candidates_of_same_grid` (hard 4200187, small 4200228).
- **Config v2** (`system.yaml` schema 2): cube; locations small/medium/large (cell_mm 34/40/48 NECALIBRAT, observe_pose,
  camera_height_mm, nadir_uv, expected_corners_px); arm; proximity (HC-SR04 presupus); timing (NEMĂSURAT); planning
  (`max_tower_level: 4`, pus pentru turnul de 5); preview; `sort_map.yaml` cu zone_red/yellow/green/blue (`max_level: 3`).
- **Previzualizare cameră:** `brain camera preview` (MJPEG `/stream.mjpg`, `/snapshot.jpg`, `/annotated.jpg`, `/status.json`,
  imagini salvate în `out/preview/`) și `brain camera capture`. Testată doar pe cadre sintetice (~9,5 fps pe laptop).
  Teste: `tests/integration/test_preview.py`.
- **Teste:** 226/226 pe Python 3.12 și 226/226 pe Python 3.8 + numpy 1.24.4 + OpenCV 4.6.

### Implementat, dar fără teste automate
`MockArm` (SimWorld, FaultSpec, SimDurations, log), `TeamArm` (HTTP stdlib, inversarea semnului yaw, maparea erorilor),
`validation.py`, `SystemClock`/`ManualClock`, `SimulatedProximity` (HC-SR04), `SyntheticCamera`/`FileCamera`/`LoopingFileCamera`,
`UsbCamera` (netestată pe hardware), `SimWorld`, `render_location` + `NeighborGrid`, `domain/plan.py` (acțiuni, Move, Plan,
`correct_parallax`), `domain/proximity.py`.

### Lipsește complet
`brain/planning/`, `brain/orchestration/`, renderer 2D + GIF (Pillow nu e încă în `pyproject.toml`), comenzile
`brain sim run`, `brain run`, `brain report timings`, `brain calib corners`, `docs/stage2.md`, scripturile pentru Pi, planul de test live.

### Cifre măsurate (selftest seed 42, 200 de scene; date sintetice, NU validează camera reală)

| Rulare | Detecție | Geometrii greșite | Ocupare | Culoare | Yaw ≤ 5° | Verdict |
|---|---|---|---|---|---|---|
| medium nominal | 100% | 0 | 100% | 100% | 100% (mediană 0,16°) | trece toate pragurile |
| medium hard | 95,0% | 0 | 100% | 100% | 99,8% | doar raportat (Etapa 0: 91,4%) |
| small nominal | 85% | 1 (seed 4200295, 5,6 px, doar 3 linii măsurabile) | 97,5% | 100% | 93,9% | pică; nevalidat anterior |
| large nominal | 87,1% | 0 | 99,8% | 100% | 100% | toate eșecurile sunt scene ArUco (`marker_centers_mm` definite doar pentru grila de 40 mm); liniile: 100% |

Sonda cu șabloane vecine (`probe_neighbors.py 12`): 34/36 scene corecte (small seed 0: colț 4,1 px; small seed 5: respinsă
sigur). Eroarea uv după corecția de paralaxă: mediană 0,16–0,27 mm; p95 2,1 mm (small) / 1,3 mm (medium) / 0,7 mm (large);
max 4,4 mm (small).

## 3. Serviciul de mișcare al colegilor (verificat pe cod și în simulare)

TypeScript/Node, `http://127.0.0.1:8765`, `POST /<metodă>` cu JSON, răspuns `{"result": ...}`; `GET` doar la `status` și `read_joints`.
Metode: `home`, `go_pose{name, speed fast|slow}` („home” există mereu), `move_joints{angles 5|6, time_ms 0..10000}`,
`open_gripper`, `close_gripper`, `pick{location, cell [r,c] 0..2 | null pentru zonă, yaw_deg, level 0..8, uv [u,v] 0..3}`,
`place{location, cell, level, yaw_deg}`, `read_joints` (null = citire eșuată), `beep{n 1..10}` (bipuri de 100 ms),
`status{sim, poses, sites (nume → grid|point), joints (comandate), grip, xyz}`.
Erori `{"error":{code,message}}`: `argument`/`limite` 400, `ruta` 404, `metoda` 405 (nedocumentat), `calibrare` 409, `hardware` 500.
Convenții: level k = k·30 mm; yaw la caroiaj = în imaginea redresată, de la axa coloanelor spre r crescător (OPUS convenției
noastre → `TeamArm` inversează semnul; confirmat pe servo 5); yaw la ZONĂ = în sistemul robotului (x în față);
`uv` = centrul bazei cubului, corectat de paralaxă (centrul celulei (r,c) = (c+0,5, r+0,5)); dacă `uv` e dat, `cell` e ignorat.
`pick`: deschide gheara, tranzit la ≥ 70 mm, coboară la 45 mm deasupra, apoi în 3 pași până la jumătatea cubului, strânge,
urcă la 45 mm. `place`: la fel, eliberează cu 3 mm mai sus. Calibrare: `npm run calib -- poza NUME | celule NUME | zona NUME | test NUME r c`.
`sim/calib.json`: `source`, `dest`, `observe_source`, `observe_dest`, `zone_red/yellow/blue/green`.

**Rezultate în simulare cu `TeamArm`:** toate metodele merg. Erorile se mapează corect (`calibrare` → `CalibrationMissing`,
`argument` → `InvalidCommand`), iar refuzurile locale merg (level 9, uv 3,5, unghi 200, beep 300 ms, `open_gripper(full=False)`).
**Accesibilitate** (lungimi NECALIBRATE din `config.ts`, cu `home` înainte): niveluri 0..6 în toate celulele și zonele, 7..8 nu.
Turnul de 5 e posibil în simulare. `npm run sim` răspunde instantaneu: nu dă timpi.

**Probleme găsite la ei** (notate în `questions-for-kinematics-team.md`):
1. `pick`/`place` pot întoarce `false` DUPĂ ce brațul s-a mișcat (de ex. `place source [2,2] level 7` după level 6: gheara urcă
   de la z=243 la 263, apoi „inaccesibil”).
2. Tranzitul rămâne la înălțimea curentă: după o poziție înaltă, celulele depărtate dau `false` până la `home`/`go_pose`.
3. npm 12 blochează compilarea `i2c-bus` (`npm install-scripts approve i2c-bus`).
4. Numele locurilor diferă de ale noastre (small/medium/large, observe_small/medium/large).
5. Sistemul de referință al yaw-ului la zone e neconfirmat.
6. Codul 405 nu e documentat. Clientul lor Python nu prinde `socket.timeout` pe Python < 3.10.

**Consecințe pentru brain (de implementat):** după orice `false` sau eroare de comunicație → `go_pose(observe_*)` înainte de
reîncercare; tratează `false` ca „brațul poate fi în altă poziție, dar nu a coborât”; `TeamArm` NU trebuie să inverseze yaw-ul la
zone (citește tipul locului din `status.sites` la `connect()`); pentru testele cu 3 șabloane, un `calib.json` propriu în
`ai/tests/fixtures/` pornit cu `--calib`.

## 4. Proba (informații de la mine)

- TREI șabloane 3×3 (mic, mediu, mare) pe aceeași foaie A4, care nu ocupă toată foaia. Brațul se adaptează după unde sunt cuburile.
- **S1:** memorează aranjamentul de pe un șablon, apoi pune pe fiecare poziție cubul potrivit, ales din TOATE cuburile prezente
  (posibil de pe alt șablon). Ținta poate veni din observație SAU din fișier YAML/JSON (per celulă `top` și opțional `bottom`).
  Fața de jos nu se vede: `bottom` cerut și neverificabil → raportat explicit „neverificabil”.
- **S2:** turn într-o ordine dată la start (`--order red,yellow,green,blue,red`, cu repetiții), în celula indicată de juriu.
- **S3:** hartă cu pătrate marcate prin FORME (nu culori); fiecare cub pe pătratul culorii lui.
- Limită: 5 minute per sarcină. Senzor de proximitate 2–400 cm (aproape sigur HC-SR04, neconfirmat). Timpi: se măsoară la rulări.
- **Exemple (doar teste fixe; la juriu totul e aleatoriu).** Imaginile s1/s2 NU există: `ai/docs/examples/Tema_CRI_2026.pdf` e gol.
  - S1 exemplu: R1 = galben, verde, albastru; R2 = roșu, galben, verde; R3 = albastru, roșu, galben; „jos” = aceeași culoare
    ca fața de sus; toate 9 celulele ocupate.
  - S2 exemplu: de jos în sus roșu, galben, verde, albastru, roșu (2 cuburi roșii, niveluri 0..4).

## 5. Decizii confirmate

SceneState unic + diff după verificare. FSM minimă OBSERVE → PLAN → EXECUTE → VERIFY → DONE/ERROR, cu tabel explicit de tranziții.
Memorie: scriere atomică, fără jurnal. Grila după linii; ArUco doar opțional. `yaw_control` dezactivat. Prioritate: S2 și S3
cap-coadă în simulare, apoi S1, apoi Pi + test live. Prefer simplu și funcțional pe braț, nu complet și netestat.

## 6. ETAPA 2 — ce mai e de făcut (în ordine)

1. **Teste pentru modulele existente:**
   - suita de contract comună `MockArm` + `TeamArm`, cu un server HTTP fals (`ThreadingHTTPServer` pe `127.0.0.1:0`), care
     imită exact răspunsurile din §3: `false` + `reason`, codurile de eroare, `null` la `read_joints`, timeout;
   - opțional, un test marcat `service`, sărit dacă lipsește Node, care pornește serviciul real în simulare cu un
     `calib.json` pentru small/medium/large;
   - `SimWorld`: prindere în/în afara toleranței, nivel greșit, coliziune, cădere, stive;
   - `SimulatedProximity`: BELOW_RANGE cu cub ținut, distanțe ≈30+s / ≈60+s, defecte injectate;
   - randare + percepție pe cele 3 dimensiuni, cu vecini și precizie uv (din `scripts/probes/probe_neighbors.py`);
   - `correct_parallax`, `validation.py`, `ManualClock`.
2. **Corecturi în `TeamArm`:**
   - yaw-ul nu se inversează la zone;
   - `false` se tratează ca stare de poziție incertă;
   - opțional, o eroare `limite` primită în timpul unui `pick`/`place` nu mai înseamnă „brațul nu s-a mișcat”.
3. **`brain/planning`:**
   - Lumea de planificare, construită din SceneState + config:
     - sloturi = celulele locațiilor + zonele din `sort_map`;
     - stive, din care doar vârful e manipulabil;
     - filtru de încredere (`planning.min_confidence`).
   - Costul, din `arm.calib_json`:
     - model de grilă O + c·ecol + r·erow (formula din `src/grid.ts`) și puncte pentru zone;
     - cost = max(|Δ unghi bază|, `radial_deg_per_mm`·|Δr|), plus penalizare lângă stive ≥ 2;
     - fără `calib.json`: metrică substitut (Manhattan + `location_switch_deg`), marcată `cost_is_substitute`.
   - Obiectivele:
     - **turn (S2):** repetiții de culoare, lungime 1..`max_tower_level`+1, cub de bază deja corect păstrat;
     - **reproducere (S1):** țintă din observație SAU din fișier; alege din tot ce e pe masă; `match_top_face`
       prefer/require/ignore; `bottom` opțional raportat „neverificabil”;
     - **sortare (S3):** fiecare culoare pe zona ei, stivuire până la `sort_map.max_level`, cuburile deja corecte ignorate,
       culorile necunoscute tratate după `unknown_color_policy`.
   - Greedy:
     - cuburile deja fixate rămân pe loc;
     - cuburile care blochează se mută pe celule libere din `staging_locations`;
     - nicio atribuire dublă;
     - motive clare pentru obiectivele neîndeplinite: „lipsește 1 cub roșu”, „prea puține cuburi”, „nivel peste limită”;
     - explicație și durată estimată (din `timing`).
   - La pick: uv = `correct_parallax(uv_top, (level+1)·30, camera_height_mm, nadir_uv)`, limitat la [0,3]. La place: yaw 0.
4. **`brain/orchestration`:**
   - **ArmSession:**
     - verifică modul (`status.sim` vs. `--execute`/hardware) și validează comenzile;
     - măsoară durata fiecărei comenzi (jurnal de durate) și loghează tot.
   - **LocationObserver:**
     - pașii: `go_pose`, apoi `settle_s`, captură, pipeline;
     - reîncercări până la `max_observe_attempts`;
     - publică imaginile în `PreviewHub`, ca să se vadă live în browser în timpul sarcinii (camera nu poate fi deschisă de
       două procese).
   - **Îmbinarea** observației cu memoria (nivelurile de sub vârf, invizibile, vin din memorie).
   - **Verificarea prin proximitate** după pick/place:
     - BELOW_RANGE = cub ținut; ≈30+s = cub sub gheară; ≈60+s = nimic;
     - cu `s` necunoscut → doar ținut/neținut;
     - contradicție cu viziunea → reobservare.
   - **FSM**, cu tabel de tranziții și recuperare:
     - pick ratat → reobservare, maximum `max_pick_attempts`;
     - cub căzut → replanificare;
     - timeout sau eroare de comunicație → ERROR, fără `home` automat;
     - observație neacționabilă → reîncercare, apoi ERROR;
     - buget depășit → nicio mutare nouă.
   - **Restul:** TimeBudget pe Clock injectabil, loguri JSON Lines, raport JSON per rulare, jurnal de durate pentru
     `report timings`.
5. **Renderer 2D schematic + GIF** (Pillow, adăugat în `pyproject.toml`; pe Pi, opțional).
6. **CLI:**
   - `brain sim run --task tower|sort|reproduce`, cu opțiunile:
     - `--order`, `--target`, `--target-file`, `--faults`;
     - `--gif`, `--dry-run`, `--seed`;
     - `--random` pentru scenarii aleatorii.
   - `brain run --arm team --camera usb|files`:
     - implicit doar planifică; mișcă brațul numai cu `--execute`;
     - confirmare în consolă înainte de prima mișcare;
     - Ctrl+C oprește după comanda curentă și nu trimite `home`.
   - `brain report timings`.
   - `brain calib corners --location X --frames ...` (scrie `expected_corners_px`).
7. **Teste cap-coadă în simulare:**
   - exemplele S1 și S2 din §4, ca teste fixe;
   - ≥ 50 de scenarii aleatorii cu seed:
     - turnuri 1–5 cu repetiții;
     - aranjări 3×3 complete și parțiale;
     - sortări;
   - cazuri speciale:
     - cub care blochează;
     - culoare lipsă / prea puține cuburi;
     - sortare cu cuburi deja corecte;
     - reproducere de pe small pe large;
   - robustețe:
     - erori injectate;
     - buget depășit;
     - contradicție de proximitate;
     - determinism;
   - raportează rata de succes și timpul simulat per sarcină.
8. **Restanțe de percepție (după ce primesc dimensiunile reale):**
   - selftest pe small/large fără scenele ArUco, sau `marker_offset_mm` relativ la grilă;
   - robustețea pe small (respingeri `grid_line_unsupported` corecte, ocupare 97,5%, yaw 93,9%);
   - geometria acceptată cu doar 3 linii măsurabile (4200295).
9. **Documentație și final de etapă:**
   - `docs/stage2.md`: structură, comenzi, rezultate măsurate, limitări, „implementat / simulat / nevalidat pe hardware”;
   - actualizează `README.md`, `questions-for-kinematics-team.md` (multe răspunsuri sunt în `CONTRACT_BRAT.md`) și
     `step1-analysis.md` (senzorul EXISTĂ, mișcarea trece prin serviciu, trei șabloane, exemplele S1/S2);
   - testele pe Python 3.8, apoi commit, push, fast-forward `main` și oprire.

## 7. ETAPA 3 — Raspberry Pi și test live (doar după confirmarea mea pentru Etapa 2)

1. **`scripts/pi_doctor.sh`:** raportează `python3 --version`, `/etc/os-release`, `uname -m`, Node/npm, `/dev/video*`,
   `i2cdetect -y 1` (0x15), serviciile Yahboom care țin ocupată magistrala sau camera, spațiul liber și temperatura.
2. **`scripts/pi_install.sh`:**
   - venv;
   - wheel aarch64, piwheels pe armv7 sau `apt install python3-opencv` ca rezervă (cu `--system-site-packages`);
   - extra `[pi]` în `pyproject`;
   - varianta offline (wheels copiate de pe laptop);
   - Python 3.7 → instalează unul nou.
3. **Rulare fără ecran:** previzualizarea există deja (`brain camera preview`). Rămâne de integrat în `brain run`
   (hub comun) și de salvat un istoric opțional de imagini.
4. **Serviciul colegilor pe Pi:**
   - pașii exacți pentru `npm run server` pe braț real (I2C, aprobarea `i2c-bus`, oprirea serviciilor Yahboom);
   - `brain run` verifică `status.sim == false` înainte de `--execute`.
5. **`docs/live-test-plan.md`** + comenzi CLI dedicate. Fiecare pas are criteriu de trecere și „ce faci dacă pică”:
   1. camera: expunere/WB raportate, imagine salvată;
   2. calibrare HSV în lumina sălii (unealtă nouă, cu statistici HSV per celulă din previzualizare);
   3. `calib corners` pentru fiecare șablon;
   4. `vision analyze` live, comparat cu ce văd eu;
   5. braț: status, home, `go_pose observe_*` fără cuburi;
   6. un singur pick + place cu `--execute`, la viteză redusă;
   7. sarcinile: S2 cu 2 niveluri, apoi 5; S3; S1;
   8. `report timings` și actualizarea `timing`.
6. **Siguranță:** `--execute` obligatoriu, confirmare înainte de prima mișcare, Ctrl+C fără `home` automat, log complet.
7. **`scripts/make_bundle`:** arhivă cu `ai/` (fără `.venv`, `out`, cache) + config.
8. **Senzorul HC-SR04:**
   - adaptor GPIO real (trecut în `HARDWARE_ALLOWED`);
   - divizor 5 V → 3,3 V pe ECHO;
   - de clarificat cine îl citește: brain-ul sau serviciul.
9. **README, secțiunea „Ziua concursului”:** ordinea exactă a comenzilor, de la pornirea Pi-ului până la fiecare sarcină,
   cu parametrii juriului (ordinea turnului, celula țintă, aranjarea).

## 8. Ce aș mai face (propuneri, de discutat)

- Un test de integrare care rulează `brain sim run` împotriva serviciului lor real (`npm run sim`) cu 3 șabloane. Prinde
  nepotrivirile de contract înainte de Pi.
- După fiecare mutare, `go_pose(observe_*)`: ocolește problemele 1–2 de la serviciu și oricum e necesar pentru verificare.
- O comandă `brain doctor` (Python) care verifică într-un pas: config, cameră, serviciul (`status`), spațiul pe disc.
- Statistici HSV per celulă în `/status.json`, pentru calibrarea culorilor în sală fără ecran.
- `expected_corners_px` din `calib corners`, pentru alegerea robustă a șablonului când se văd mai multe pe foaie.
- Pentru S1 cu cuburi în afara șabloanelor: serviciul acceptă doar celule și puncte de zonă. Ori le cerem pick la xy arbitrar,
  ori calibrăm o zonă mai mare.

## 9. Întrebări deschise (pune-le în raportul final)

- Dimensiunile reale ale celulelor celor 3 șabloane și grosimea liniilor; numele locurilor din `calib.json`.
- Înălțimea camerei în fiecare poză de observare; pozele `observe_*` învățate cu `npm run calib -- poza`.
- S1: ce înseamnă „jos: <culoare>”? E doar informativ sau se punctează?
- S2: lungimea maximă a turnului cerută la juriu și înălțimea maximă accesibilă pe brațul real.
- Harta S3: imagine sau fișier; corespondența formă → culoare; e permisă stivuirea pe punctul unei zone?
- Senzorul: model exact, pini GPIO, divizor pe ECHO, montaj (coaxial pe gheară?), cine îl citește.
- Python/OS/arhitectura de pe Pi (rezultatul diagnosticului din `ai/README.md`); versiunea de Node/npm de pe Pi.

## 10. Prima acțiune

1. `git fetch`, `git status`, backup, `git pull --ff-only` pe `brain-etapa2` (sau `main`).
2. `pytest -q`, `config validate`, selftest pe medium; confirmă 226/226 și cifrele din §2.
3. Începe cu §6.1 (teste) și §6.2 (corecturi `TeamArm`), apoi §6.3–§6.7. Raportează-mi la final de Etapă 2 și oprește-te.
