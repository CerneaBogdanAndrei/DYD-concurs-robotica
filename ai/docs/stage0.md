# Etapa 0 — Viziune offline (livrare)

Data: 2026-10-29. Totul din această etapă rulează fără braț, fără cameră și fără `Arm_Lib`.

## 1. Structura și responsabilitățile modulelor

```
ai/
├─ pyproject.toml              pachetul `brain` (Python ≥ 3.8), comanda `brain`
├─ config/
│  ├─ system.yaml              grilă (NECALIBRAT), locații, praguri de viziune, calea memoriei
│  └─ colors.yaml              intervale HSV per culoare (NECALIBRAT)
├─ brain/
│  ├─ domain/                  tipuri pure, fără I/O și fără OpenCV
│  │  ├─ grid.py               GridCoord, iterare row-major
│  │  ├─ cube.py               Color, TopFace, Occupancy, convenția yaw [-45, 45)
│  │  ├─ observation.py        CellObservation, GridObservation (neacționabilă ⇒ fără celule)
│  │  ├─ scene_state.py        SceneState (stive de cuburi), diff_locations/diff_scenes, ID-uri unice
│  │  └─ errors.py             ierarhia de excepții (ConfigError numește fișier/cheie/valoare)
│  ├─ configuration/           citire YAML + validare strictă (chei necunoscute respinse)
│  ├─ perception/
│  │  ├─ photometry.py         calitatea cadrelor, mediana pe 5 cadre, balans de alb software
│  │  ├─ grid_lines.py         detecția grilei după linii (metoda principală)
│  │  ├─ grid_aruco.py         detecția opțională prin ArUco (API OpenCV vechi și nou)
│  │  ├─ geometry.py           ordonare colțuri, validarea patrulaterului, omografie, verificarea liniilor
│  │  ├─ color.py              vot pe nuanță cu încredere și motive
│  │  ├─ cells.py              prezență, culoare (inel exterior), yaw, tipul feței, separarea cuburilor lipite
│  │  ├─ pipeline.py           VisionPipeline: cadre → GridObservation
│  │  ├─ annotate.py           JPG adnotat (imaginea originală + imaginea redresată)
│  │  └─ image_io.py           citire/scriere compatibilă cu căi Unicode pe Windows
│  ├─ memory/                  codec JSON versionat + SceneStore (scriere atomică, protecție la suprascriere)
│  ├─ simulation/scene_gen.py  generator determinist de scene sintetice + adevăr de referință
│  ├─ cli/                     `python -m brain ...` și selftest-ul Etapei 0
│  └─ logging_setup.py         loguri JSON Lines
├─ tests/                      183 de teste (unit, integrare, arhitectură)
└─ docs/                       analiza, întrebările pentru echipa cinematică, acest document
```

## 2. Dependențe

| Pachet | Cerință | Testat |
|---|---|---|
| Python | ≥ 3.8 | 3.12.10 și 3.8.20 |
| numpy | ≥ 1.19 | 2.5.4 și 1.24.4 |
| opencv-contrib-python | ≥ 4.5 | 5.0.0 și 4.6.0 (API ArUco vechi) |
| PyYAML | ≥ 5.3 | 6.0.3 și 6.0.1 |
| pytest (dezvoltare) | ≥ 7 | 9.2.0 și 8.3.3 |

Pe Raspberry Pi nu a fost testat încă: versiunea de Python/OS e necunoscută. Pe Pi nu instalați `opencv-python` și `opencv-contrib-python` simultan (intră în conflict).

## 3. Instalare

```bash
cd ai
python -m venv .venv
# Windows
.venv\Scripts\python -m pip install -e ".[dev]"
# Linux / Raspberry Pi
.venv/bin/python -m pip install -e ".[dev]"
```

În consola Windows, pentru diacritice corecte: `$env:PYTHONUTF8=1` (PowerShell) sau `set PYTHONUTF8=1` (cmd).

## 4. Comenzi (rulate din `ai/`)

```bash
python -m brain config validate
python -m brain sim generate --seed 42 --count 3 --out out/synthetic            # PNG + truth.json
python -m brain sim generate --seed 7 --kind occluded_corner --out out/synthetic
python -m brain vision analyze --frames "out/synthetic/scene_42/frame_*.png" --location source --out out/analysis
python -m brain vision analyze --frames "out/synthetic/scene_42/frame_*.png" --location source --state-out data/memory/scene_state.json
python -m brain memory show --path data/memory/scene_state.json
python -m brain selftest stage0 --seed 42 --scenes 200 --out out/selftest      # comanda unică de validare
python -m brain --log-file out/logs/brain.jsonl vision analyze ...              # loguri JSON Lines
```

Tipuri de scene (`--kind`): `positive`, `blank`, `no_grid`, `overexposed`, `underexposed`, `grid_out_of_frame`, `degenerate`, `occluded_corner`, `ambiguous_color`, `missing_marker` (cere `--aruco`).

Coduri de ieșire:

| Cod | Semnificație |
|---|---|
| 0 | succes |
| 1 | selftest picat |
| 2 | eroare de configurare sau de intrare (mesajul numește fișierul, cheia și valoarea) |
| 3 | observație neacționabilă |

## 5. Teste

```bash
python -m pytest            # 183 de teste, ~30 s
```

| Grup | Ce verifică |
|---|---|
| `tests/unit` | domeniul, configurarea (erori cu fișier/cheie/valoare), memoria (atomicitate, corupere, versiune incompatibilă, eșec la `os.replace`), fotometria, geometria, culoarea (inclusiv cazuri ambigue), celulele, generatorul (determinism) |
| `tests/integration` | pipeline pe scene pozitive și negative, ArUco, colț acoperit, rotirea indexării, selftest determinist, CLI cap-coadă |
| `tests/test_architecture.py` | niciun modul nu importă `Arm_Lib`/`smbus`/`serial`/`RPi`; importul tuturor modulelor nu încarcă biblioteci hardware; straturile pure nu importă OpenCV/numpy; sintaxa e compatibilă cu Python 3.8 |

## 6. Rezultate măsurate (2026-10-29, PC de dezvoltare, date sintetice)

`python -m brain selftest stage0 --seed 42 --scenes 200`: 140 de scene pozitive (120 cu linii, 20 cu ArUco) și 60 negative sau speciale.

| Metrică | Prag | seed 42 (200) | seed 7 (400) | `hard`, seed 42 (doar raportat) |
|---|---|---|---|---|
| Detecție grilă | ≥ 99% | 100% | 100% | 91,4% (respingeri sigure) |
| Geometrie greșită acceptată | 0 | 0 | 0 | 0 |
| Ocupare | ≥ 99% | 100% | 100% | 100% |
| Culoare | ≥ 98% | 100% | 100% | 100% |
| Tipul feței | ≥ 95% | 100% | 100% | — |
| Yaw ≤ 5° (pentru \|yaw\| ≤ 40°) | ≥ 95% | 100% (mediană 0,17°, p95 0,49°) | 100% | 99,8% |
| Erori cu încredere > 0,8 | ≤ 1% | 0% | 0% | 0% |
| Scene negative respinse | 100% | 100% | 100% | 100% |
| Timp de analiză (5 cadre 640×480) | — | mediană 162 ms | — | — |

Același seed produce același raport (testat automat). **Aceste cifre nu spun nimic despre camera reală.**

## 7. Cum funcționează (pe scurt)

**Detecția grilei (linii):**
1. Masca pixelilor întunecați față de media locală și necolorați.
2. Patrulater grosier: înfășurătoarea convexă a liniilor, cu colțurile teșite refăcute prin prelungirea laturilor.
3. Redresare cu margine; în profilele de proiecție se caută 4 linii echidistante pe fiecare axă (este tolerată o linie lipsă).
4. Corecție globală de omografie: fiecare pixel de linie dă o ecuație liniară; rezolvare prin cele mai mici pătrate cu ponderi Huber, 3 iterații.
5. Omografia finală din cele 16 intersecții.

**Validarea geometriei:**
- convexitate, arie, distanța față de marginea imaginii, raportul laturilor, unghiurile, numărul de condiționare normalizat;
- **suport pe fiecare linie**: oriunde ambele părți ale unei linii sunt masă vizibilă, trebuie să existe tuș. O grilă estimată greșit are întotdeauna o linie care trece prin masa goală.

**Celulele:**
- regiunile colorate se calculează separat pe clase de nuanță și se atribuie celulei după centroid (rezistent la paralaxă);
- cuburile lipite se separă după nucleele transformatei de distanță;
- culoarea se votează pe inelul exterior al feței;
- yaw = orientarea dominantă a conturului (media circulară a lui 4θ), cu concentrarea ca încredere;
- o celulă „fără culoare” e declarată goală doar dacă nu are muchii, obiecte întunecate sau culoare slabă. Altfel e `unknown`: un cub decolorat nu e raportat ca absent.

## 8. Coduri de motive (`reasons`)

| Cod | Semnificație |
|---|---|
| `too_few_valid_frames`, `overexposed`, `underexposed`, `low_contrast` | calitate insuficientă a imaginii |
| `grid_not_found`, `grid_lines_not_found` | grila nu a fost găsită |
| `markers_missing:<id>`, `markers_duplicate:<id>` | ArUco: markeri lipsă sau duplicați |
| `quad_not_convex`, `grid_too_small`, `grid_touches_border`, `perspective_too_strong`, `aspect_out_of_range`, `angle_out_of_range`, `homography_ill_conditioned` | geometrie neplauzibilă |
| `grid_line_unsupported`, `grid_structure_weak`, `grid_structure_unverifiable` | liniile nu confirmă geometria |
| `edges_without_color`, `dark_object_without_color`, `faint_color_object`, `ambiguous_colored_area`, `roi_occluded` | celulă `unknown`: posibil obiect necolorat |
| `ambiguous_hue`, `insufficient_color_pixels`, `low_saturation` | culoare nesigură |
| `off_center`, `merged_blob_separated`, `split_blob`, `oversized_blob`, `blob_clipped_by_image`, `yaw_unreliable` | încredere redusă pentru poziție sau yaw |

## 9. Implementat / simulat / nevalidat

- **Implementat și testat offline:**
  - detecția grilei după linii și prin ArUco;
  - validarea geometriei;
  - analiza celulelor;
  - agregarea cadrelor și balansul de alb software;
  - `SceneState` cu scriere atomică și `diff`;
  - JPG adnotat, CLI, loguri JSON;
  - selftest reproductibil.
- **Simulat:** scenele sintetice. Modelul e aproximativ:
  - masa e randată ortografic și proiectată printr-o omografie aleatoare;
  - paralaxa e aproximată;
  - nu există distorsiune de lentilă, reflexii, texturi reale ale mesei sau cuburi reale.
- **Nevalidat** (necesită hardware sau informații externe):
  - camera reală (expunere, balans de alb, distorsiunea lentilei);
  - culorile reale ale cuburilor și pictogramele;
  - dimensiunile reale ale grilei;
  - performanța pe Raspberry Pi;
  - markerii ArUco tipăriți;
  - dacă regulamentul permite markeri.

## 10. Limitări cunoscute

1. **Distorsiunea lentilei nu e corectată.** Omografia presupune o cameră ideală. Camerele USB au distorsiune radială; calibrarea intrinsecă intră în Etapa 1. Reziduul ajustării liniilor (`line_residual_cells`) va arăta dacă e necesară.
2. **Orientarea unei grile fără markeri e ambiguă la 90°.** Se fixează prin poziția de observare (rotație sub ±45°) și prin `orientation_quarter_turns`.
3. **Ocluzie mare + paralaxă puternică** (o latură întreagă de contur acoperită) → grila e respinsă. Pe scenele `hard`, 8,6% din cele pozitive sunt respinse. Respingerea e sigură, dar costă o reobservare; remediul practic este o poziție de observare cât mai verticală, cu grila mare în cadru.
4. **Un obiect alb/gri, fără muchii în interiorul celulei, nu se distinge de masă.**
5. Yaw-ul e observabil doar modulo 90°; pictograma nu e recunoscută.
6. **Toate pragurile sunt stabilite pe date sintetice** și sunt marcate `calibrated: false`. Pe camera reală trebuie recalibrate.

## 11. Pașii următori (conform priorității: S2 și S3 în simulare)

**Etapa 2, varianta redusă pentru S2/S3:**
- `ports/arm.py` (contractul `ArmInterface`);
- `SimWorld` + `MockArm` cu injectare de erori;
- `tests/test_arm_contract.py`;
- planificatoarele S2 (turn) și S3 (sortare Greedy);
- mașina de stări minimă;
- bugetul de timp cu `Clock` simulat;
- `poses.yaml` și `sort_map.yaml` cu exemple marcate necalibrate;
- renderer 2D / GIF.

Informații necesare înainte:
- S3: cuburile „dispersate” stau pe celulele unui caroiaj cunoscut? Fără IK, brațul poate prinde doar în poziții învățate.
- Unde sunt zonele de sortare și zona tampon (`staging`), și câte locuri are fiecare?
- Estimări de timp pentru `pick`/`place` (sau le lăsăm ca valori configurabile, marcate neestimate).
