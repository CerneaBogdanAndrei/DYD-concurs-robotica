# Pasul 1 — Validare și proiectare (subsistemul Brain, CRI 2026)

Data: 2026-10-29. Documentul conține analiza inițială, actualizată cu deciziile confirmate de echipă.

## 0'. Actualizări din Etapa 2 (2026-10-29) — au prioritate față de tabelul de mai jos

| Subiect | Acum |
|---|---|
| Senzor de proximitate | **Există** (2–400 cm, aproape sigur HC-SR04, neconfirmat). Simulat (`SimulatedProximity`) și folosit opțional la verificarea pick/place (`orchestration/proximity_check.py`); pe robot e dezactivat (`proximity.enabled: false`) până la montare și calibrare. |
| Mișcarea | Prin serviciul colegilor (TypeScript, HTTP local, `docs/CONTRACT_BRAT.md`), cu extensiile `level` și `uv`. Brain-ul nu are adaptor Arm_Lib. |
| Caroiaje | Echipa a primit trei foi A4 (câte un caroiaj 3×3: mic, mediu, mare) și a ales foaia `medium` (celulă 4,3 cm, măsurată). La probă se folosește doar ea (`planning.active_locations`); `small`/`large` rămân pentru testele de viziune. |
| Mutări intermediare | Celulele libere ale caroiajului, apoi punctele tampon opționale (`planning.buffer_points`, calibrate ca zonele). |
| Exemple | S1: R1 = galben, verde, albastru; R2 = roșu, galben, verde; R3 = albastru, roșu, galben (fața de jos = fața de sus, neverificabilă). S2: de jos în sus roșu, galben, verde, albastru, roșu. Ambele sunt teste fixe în `tests/integration/test_sim_tasks.py`. |
| Stive | Până la 9 cuburi pe stivă (nivelurile 0..8 ale serviciului); turnul S2 e limitat de `planning.max_tower_level` (4 → 5 cuburi). |
| Detalii | `docs/stage2.md`. |

## 0. Decizii confirmate (2026-10-29)

| Subiect | Decizie |
|---|---|
| Senzor de proximitate | **Nu există.** Proximitatea e scoasă complet: porturi, fuziune, praguri, simulator. |
| SafetyGate | Rămâne doar pentru validarea comenzilor: limite de unghiuri, locații, niveluri, buget de timp. |
| Python pe Pi | Necunoscut. Codul țintește **Python 3.8+**. |
| Caroiaje S1 | 2 caroiaje (`source`, `dest`), fiecare cu poziție de observare proprie. |
| Detecția grilei | **Metoda principală: linii/contur.** ArUco e opțional, activat din config (nu se știe dacă regulamentul permite markeri). |
| Modul B | După reset, cuburile sunt în zona `staging`, care are poziție de observare proprie. Ținta = caroiajul `dest`. Configurabil. |
| Cuburi | Până la 9, culorile se pot repeta. Cuburile cu aceeași culoare (și aceeași față sus) sunt interschimbabile simbolic. |
| Stare a lumii | **O singură stare (`SceneState`)** + `diff` după verificare. Fără tipurile separate Observed/Planned/Confirmed. |
| ArmInterface | Doar contractul de bază. Fără extensii opționale deocamdată. |
| Memorie | Scriere atomică (tmp + fsync + replace). Fără jurnal și fără carantină. |
| Mașina de stări | Minimă: `OBSERVE → PLAN → EXECUTE → VERIFY → DONE/ERROR`, cu tabel explicit de tranziții. |
| Teach-in | Se învață colțurile + centrul fiecărui caroiaj. Celelalte celule se interpolează în spațiul articular, apoi se validează individual; orice celulă poate fi suprascrisă. Același mecanism pentru nivelurile turnului (offset de înălțime). |
| Control yaw | `yaw_control: disabled` implicit, până la calibrare. |
| Pachet și structură | Pachetul `brain`; algoritmii de viziune în `perception/`; `SceneState` inclus în Etapa 0. |
| Dependențe | `numpy`, `opencv-contrib-python`, `PyYAML`, `pytest` (+ `Pillow` din Etapa 2). Fără pydantic. |
| Prioritate | **S2 și S3 cap-coadă în simulare cât mai repede**, apoi S1. Un sistem simplu care merge pe braț e preferat unuia complet, dar netestat. |

## 1. API-ul Yahboom `Arm_Lib`

**Proveniența informațiilor.** Sursa oficială `Arm_Lib` nu e publică pe GitHub. Yahboom o distribuie în `Dofbot.zip` (`0.py_install/setup.py`) și o preinstalează în imaginea de sistem. Surse secundare folosite:

- tutoriale Yahboom și ZetaBank (semnături);
- reimplementarea Go a protocolului I2C, `viam-yahboom-dofbot` (adresă, registre, conversii);
- un raport de diagnostic pentru DOFBOT SE, `dofbot-se-tools` (harta registrelor, bug-uri).

**Nimic de aici nu e confirmat pe dispozitivul nostru.**

| Primitivă | Ce indică sursele secundare | Status |
|---|---|---|
| `Arm_Device()` | Varianta I2C: adresa `0x15`. Constructorul deschide probabil `SMBus(1)` imediat. | De verificat. Importul cere `smbus` (pică pe Windows). |
| `Arm_serial_servo_write(id, angle, time)` | Registrul `0x10+id`, payload `[posH,posL,timeH,timeL]`. `time` e pe 16 biți (max. 65535). | Unitatea ms e probabilă (un tutorial se contrazice). Trebuie cronometrat. |
| `Arm_serial_servo_write6(s1..s6, time)` | Două tranzacții separate: `0x1e` (timp), apoi `0x1d` (12 octeți de poziții). | **Neatomic**: dacă a doua tranzacție pică, timpul e scris, pozițiile nu. |
| `Arm_serial_servo_read(id)` | `int` în grade sau `None` (eroare / poziție 0 / în afara domeniului). Rezoluție 1°. | `None` trebuie tratat explicit. |
| `Arm_Buzzer_On(delay=0xff)` / `Arm_Buzzer_Off()` | `delay` în unități de 100 ms (20 → 2 s, observat pe SE). Domeniu documentat 1–50. 255 („continuu”) pare respins. | De verificat. `beep(duration_ms)` se cuantizează la 100 ms. |
| `Arm_serial_set_torque(enable)` | 0 = off, 1 = on. Întoarce `None`. | Nu confirmă că s-a aplicat. |

**Consecințe pentru design:**

1. **Servo 5 are domeniul 0–270°, nu 0–180°.** Contractul `move_joints` păstrează `[0, 180]` (submulțime sigură); limitele reale per articulație stau în config, marcate `calibrated: false`.
2. **Servourile 2, 3 și 4 sunt inversate în interiorul bibliotecii** (`180 - angle`). `poses.yaml` stochează **exclusiv unghiuri în convenția `Arm_Lib`**, exact ce citește teach-in-ul. Nu facem conversii proprii.
3. **Biblioteca pare să ascundă excepțiile** (`except` gol + `print`): confirmat pe varianta serială, probabil și pe I2C. Un `write` nu ne spune dacă a reușit → adaptorul verifică prin read-back după timpul de mișcare (toleranță ≥ 2°). Un driver propriu peste aceleași registre rămâne opțiune pentru Etapa 3, numai cu acord explicit.
4. **Citirile sunt nesigure**: pe varianta serială există răspunsuri decalate, atribuite servo-ului greșit; pe I2C apar `None` intermitente. Citim cu retry și verificare de plauzibilitate. O citire nu dovedește niciodată că un cub a fost prins.
5. **Există două variante de `Arm_Lib`** (I2C pentru Pi + shield, UART pentru SE/Pro). Trebuie verificat ce e instalat.
6. Imaginea Yahboom pornește **servicii de fabrică** care țin magistrala ocupată. Oprirea lor intră în checklistul pre-start.

**Introspecție fără mișcare și fără import** (de rulat pe Pi):

```bash
cat "$(python3 -c "import importlib.util as u;print(u.find_spec('Arm_Lib').origin)")" > arm_lib_src.txt
i2cdetect -y 1          # ar trebui să apară 0x15
python3 --version; cat /etc/os-release
```

## 2. Riscuri și ambiguități

| # | Problemă | Rezolvare |
|---|---|---|
| R1 | Fără IK: pick/place pe celulă înseamnă poziții învățate `(location, cell, level, phase)`. | Teach-in pe colțuri + centru, interpolare în spațiul articular, validare per celulă, suprascriere posibilă (decis). |
| R2 | Fără senzor de prindere. | `True` de la pick/place = „secvența a rulat fără eroare detectată”, nu succes fizic. Confirmarea vine din reobservare cu camera și `diff` pe `SceneState`. |
| R3 | Comenzile sunt blocante, contractul nu are anulare. | Oprirea e posibilă doar între comenzi. Nu e un stop de urgență. |
| R4 | Convențiile `location`, `(row,col)`, `level`, semnul yaw nu sunt stabilite între echipe. | Document de convenții (vezi `questions-for-kinematics-team.md`). |
| R5 | S1: două caroiaje fără markeri distinctivi. | Se disting prin poziția de observare (decis). |
| R6 | Modul B. | Cuburile sunt în `staging`, ținta e `dest` (decis, configurabil). |
| R7 | Culori repetate (până la 9 cuburi). | Interschimbabilitate simbolică; `cube_id` doar când culoarea e unică în scenă. |
| R8 | Fața de sus: plină, sau albă cu pictogramă și bordură de 2–3 mm (≈5–7 px). | Clasificare pe inelul exterior; `top_face: full/icon/unknown`. Recunoașterea pictogramei nu e cerută. |
| R9 | Paralaxă: fața de sus e la 30 mm deasupra mesei. | Poziție de observare cât mai verticală; atribuirea cubului la celulă după centroid; generatorul sintetic simulează paralaxa. |
| R10 | Repetabilitatea poziției de observare (joc servouri ~1–2°). | Omografia se recalculează la fiecare captură. |
| R11 | Greedy pe distanța unghiulară. | Pozițiile învățate dau unghiurile → `cost = max_i w_i·|Δθ_i|` (write6 sincron) + constantă pick/place. Fără poziții: Manhattan în celule, marcat `substitute=True`. |
| R12 | `speed` nu e viteză fizică. | `time_ms = clamp(max|Δθ| / rate_speed, min, max)`, Δθ față de ultima poziție comandată. |
| R14 | Python vechi pe Pi. | Cod compatibil Python 3.8+; testul `test_py38_syntax.py` verifică sintaxa. |
| R15 | API-ul ArUco s-a schimbat în OpenCV 4.7. | Wrapper compatibil cu ambele API-uri. |
| R16 | Controalele camerei diferă între V4L2 și DirectShow. | Setare + citire înapoi + raport; compensare software pe albul mesei (balans de alb deja în Etapa 0). |
| R17 | `[-45°, +45°]` închis la ambele capete. | Folosim `[-45, 45)`. |
| R18 | S3: „cuburi dispersate” — fără IK, brațul poate prinde doar în poziții învățate. | Presupunere de confirmat: cuburile S3 stau pe celulele unui caroiaj cunoscut. |

(R13 — senzorul de proximitate — eliminat: nu există senzor.)

## 3. Camera eye-in-hand și limitele estimării yaw

- **O omografie acoperă un singur plan dintr-o singură poziție.** Nu combinăm imagini din poziții diferite. Orice captură folosită pentru geometrie trebuie să conțină grila (sau markerii, dacă sunt activați).
- **Paralaxa nu strică unghiul.** Între planul feței de sus și planul mesei, transformarea văzută prin aceeași cameră e o omotetie (fixează dreapta de la infinit). Yaw-ul măsurat în imaginea redresată pe masă e corect și pentru fața de sus; se schimbă doar poziția și scara. Fețele laterale vizibile oblic pot strica conturul → yaw cu încredere redusă.
- **Simetria la 90°**: yaw observabil doar modulo 90°, în `[-45, 45)`. Aproape de ±45° raportăm încredere scăzută, nu o valoare falsă.
- **Orientarea grilei (fără markeri)**: o grilă 3×3 cu linii e simetrică la rotații de 90°. Orientarea se fixează prin poziția de observare (camera vede grila mereu la fel, cu rotație sub ±45°) plus `orientation_quarter_turns` per locație în config.
- **Controlul yaw depinde de celulă**: rotația bazei schimbă orientarea gripperului față de grilă. `roll = roll_învățat + k·yaw`, cu `k` și semnul calibrate. Până atunci `yaw_control: disabled`.
- **Verificarea vizuală costă o deplasare** la poziția de observare; intră în bugetul de timp.
- **Câmpul vizual**: grila are ≈122 mm; dacă o poziție nu o prinde toată, se definesc mai multe poziții.

## 4. Coordonate logice vs. cinematice

| Spațiu | Unitate | Cine îl folosește |
|---|---|---|
| imagine brută | px | camera, detecția grilei |
| imagine redresată | px (300×300, 100 px/celulă; centrele liniilor la 0/100/200/300) | analiza celulelor |
| grilă logică | `GridCoord(row,col)`, `level` ∈ 0..3 | memorie, planificare |
| locație | `source`, `dest`, `staging`, `sort:<nume>`, `observe:<nume>` | planificare ↔ braț |
| masă (nominal) | mm, originea la centrul liniei din colțul (0,0) | generator sintetic și estimări, **niciodată comenzi** |
| articulații | grade, convenția `Arm_Lib`, ID 1..6 | adaptoarele de braț + modelul de cost |

Brain-ul comandă brațul **doar** prin `(location, cell, level, yaw_deg)`. Traducerea în articulații o face adaptorul (`YahboomArm`: poziții învățate + interpolare; `TeamArm`: IK-ul colegilor).

Convenții: `(0,0)` = colțul stânga-sus în imaginea redresată, `row` crește în jos, `col` spre dreapta (corespondența fizică se fixează prin `orientation_quarter_turns`). Yaw pozitiv = sens antiorar văzut de sus.

## 5. Arhitectura (Ports & Adapters, simplificată)

```
cli ─► orchestration (FSM minimă · Executor · SafetyGate · TimeBudget)
          ├─ perception  (cadre → GridObservation)
          ├─ planning    (SceneState + țintă → Plan, funcții pure)
          └─ memory      (SceneState ↔ JSON atomic)
                 domain (tipuri pure, fără I/O)      ports (Protocol)
  adapters/arm:     MockArm · YahboomArm · TeamArm  (unicul acces la /dev/i2c-1 e în YahboomArm)
  adapters/vision:  OpenCvCamera · FileCamera · SyntheticCamera
  simulation:       SimWorld (starea fizică simulată, comună pentru MockArm și SyntheticCamera)
```

```
ai/
├─ pyproject.toml
├─ config/        system.yaml · colors.yaml  (Etapa 0); poses.yaml · sort_map.yaml (Etapele 2–3)
├─ brain/
│  ├─ domain/         grid, cube, observation, scene_state, errors
│  ├─ configuration/  loader + validatoare (fișier / cheie / valoare)
│  ├─ perception/     photometry, grid_lines, grid_aruco, geometry, color, cells, pipeline, annotate
│  ├─ memory/         codec, store (atomic)
│  ├─ simulation/     scene_gen (+ sim_world, renderer în Etapa 2)
│  ├─ cli/            main, selftest
│  ├─ ports/ · planning/ · orchestration/ · adapters/   (din Etapa 1–2)
├─ tests/ · scripts/ · docs/
```

**Decizii-cheie:**
- `SceneState` e singura stare a lumii. După fiecare operație: reobservare → `diff(așteptat, observat)` → actualizare.
- `ArmSession` (Etapa 2) împachetează adaptorul: modul `simulation`/`hardware` vine din config; validează comenzile; consultă `SafetyGate`; scrie log.
- `Clock` injectabil pentru testele de buget de timp.

## 6. Contractele dintre module

| Între | Contract | Garanție |
|---|---|---|
| cameră → percepție | listă de cadre BGR `uint8` (implicit 5) | aceeași rezoluție, aceeași poziție de observare |
| percepție → planificare | `GridObservation(location, cells[9], actionable, reasons, geometry, quality)` | `actionable=False` ⇒ fără celule, fără manipulare |
| `CellObservation` | ocupare (occupied/empty/unknown) + încredere, culoare + încredere, `yaw_deg` sau `None` + încredere, `top_face`, motive | necunoscut rămâne `unknown`/`None` |
| memorie | `save(SceneState, overwrite=…)`, `load()` | scriere tmp + fsync + `os.replace`; refuză suprascrierea accidentală și suprascrierea unui fișier invalid |
| planificare | `plan_task{1,2,3}(state, target, cfg) → Plan` | funcții pure, fără I/O |
| orchestrare → braț | `ArmSession.execute(action) → ActionOutcome(command_accepted, duration_s, error)` | `command_accepted` ≠ succes fizic |

**Semantica `ArmInterface`:** metode blocante; `InvalidCommand` (respinsă înainte de trimitere), `ArmCommunicationError`/`ArmTimeout` (stare fizică necunoscută → reobservare obligatorie), `CapabilityUnavailable` (TeamArm neimplementat, niciodată `True` fals). După eroare de comunicație: fără retry automat pentru pick/place, fără `home()` automat.

**Mașina de stări:** `OBSERVE → PLAN → EXECUTE → VERIFY → (PLAN | DONE)`, plus `ERROR`. Tranzițiile sunt într-un tabel explicit; orice altă tranziție aruncă excepție. `TimeBudget` refuză o operație nouă dacă timpul estimat + rezerva depășește timpul rămas.

## 7. Întrebări pentru echipa cinematică

Mutate în [`questions-for-kinematics-team.md`](questions-for-kinematics-team.md).

## 8. Ordinea de implementare

**Etapa 0** (viziune offline): schelet + test de graniță a importurilor → `domain` + `configuration` → generator sintetic cu seed → fotometrie → geometrie (linii/contur, ArUco opțional) → analiza celulelor → `GridObservation` + JPG adnotat → `SceneState` v1 + stocare atomică + `diff` → CLI (`sim generate`, `vision analyze`, `selftest stage0`).

**Apoi, conform priorității:** Etapa 2 pentru S2 și S3 (SimWorld, MockArm, teste de contract, planificatoare S2/S3, FSM, renderer) → S1 în simulare → Etapa 1 (viziune live) → Etapa 3 (YahboomArm, teach-in) → Etapa 4 (TeamArm).

## 9. Criterii de acceptanță pentru Etapa 0

Doar pe date sintetice; nu spun nimic despre camera reală.

- `pip install -e ".[dev]"` și `pytest` trec fără `Arm_Lib`, `smbus`, cameră sau rețea.
- `python -m brain selftest stage0 --seed 42 --scenes 200`: rezultate identice la rulări repetate în același mediu; cod de ieșire ≠ 0 dacă pică vreun prag.
- Scene nominale: detecție grilă ≥ 99%; ocupare ≥ 99%; culoare ≥ 98% pe celulele ocupate; eroare yaw ≤ 5° (mod 90°) în ≥ 95% din cazurile cu |yaw| ≤ 40°; cel mult 1% din răspunsurile cu încredere > 0,8 sunt greșite.
- Scene negative: pipeline-ul întoarce motivul și `actionable=False`, fără stări de celule.
- Culori ambigue → `unknown` sau încredere scăzută.
- JPG adnotat pentru fiecare scenă; erorile de config numesc fișierul, cheia și valoarea; testul de graniță a importurilor trece.

## Surse

- [viam-yahboom-dofbot — reimplementare Go a protocolului I2C](https://github.com/HipsterBrown/viam-yahboom-dofbot)
- [dofbot-se-tools — raport de diagnostic și bug-uri `Arm_Lib`](https://github.com/Prof3ssa/dofbot-se-tools)
- [Yahboom — Servo calibration (torque)](https://www.yahboom.net/public/upload/upload-html/1699441487/Servo%20calibration.html)
- [Yahboom — Read the current position of the servo](http://www.yahboom.net/public/upload/upload-html/1715841863/Read%20the%20current%20position%20of%20the%20servo.html)
- [Yahboom — Driver library and communication](https://www.yahboom.net/public/upload/upload-html/1699255422/8.Driver%20library%20and%20communication.html)
- [ZetaBank — Code Explanation (write6, write6_array)](https://zeta-edu-basic.readthedocs.io/en/latest/Day1/2.robot_arm/2.explanation.html)
