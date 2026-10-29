# Etapa 2 — planificare, orchestrare, simularea sarcinilor (CRI 2026)

Data: 2026-10-29. Lucrat pe Raspberry Pi (Debian 13, aarch64, Python 3.13.5, Node 22.23.4), branch `brain-etapa2`.
**Nimic din această etapă nu a fost testat pe braț sau pe camera reală.** Tot ce urmează e verificat în simulare
și împotriva serviciului colegilor pornit cu `--sim`.

## Ce s-a schimbat față de prompt

- **O singură foaie.** Echipa a primit trei foi A4, fiecare cu un singur caroiaj 3×3, și a ales foaia `medium`
  (`docs/examples/sablon_3x3_masuratori.jpeg`): celulă **4,3 cm (măsurat)**; margini sus 1 cm, stânga 4,7 cm,
  dreapta 2,8 cm, sub caroiaj 15,9 cm. Pe lățime suma dă 20,4 cm, nu 21 cm: o nepotrivire de ~6 mm, de remăsurat.
  Config: `locations.medium.cell_mm: 43.0`, `planning.active_locations: [medium]`. `small`/`large` rămân definite
  doar pentru testele de viziune.
- **Puncte tampon** (`planning.buffer_points`, implicit goale). Cu 9 cuburi pe 9 celule (exemplul S1 amestecat)
  nu există nicio celulă liberă: o permutare nu se poate rearanja fără un loc în afara caroiajului. Un punct tampon
  se calibrează ca o zonă (`npm run calib -- zona buffer_1`) și se trece în config. Planificatorul îl folosește doar
  când nu e nicio celulă liberă.
- `MAX_STACK_HEIGHT` 4 → 9 (nivelurile 0..8 ale serviciului; turnul S2 are 5 cuburi).

## Structura nouă

| Modul | Rol |
|---|---|
| `brain/planning/world.py` | Lumea de planificare din `SceneState`: sloturi (celule ale șabloanelor active, puncte tampon, zone S3), stive, filtrul de încredere (`planning.min_confidence`), celule blocate (ocupare necunoscută / înălțime incertă). |
| `brain/planning/cost.py` | Costul în grade: din `calib.json` al serviciului (modelul de caroiaj din `src/grid.ts`, potrivit prin cele mai mici pătrate, și punctele zonelor) = max(\|Δ bază\|, `radial_deg_per_mm`·\|Δ rază\|) + penalizare lângă stive ≥ 2; fără `calib.json`: Manhattan + `location_switch_deg`, marcat `cost_is_substitute`. |
| `brain/planning/goals.py` | Obiectivele S1/S2/S3; citirea ordinii (`red,yellow,…` sau în română), a celulei (`medium:r1c1`), a țintei S1 din YAML/JSON (`rows` sau `cells`, cu `bottom`/`face` opționale) sau din observație (`target_from_location`). |
| `brain/planning/planner.py` | Greedy: turn (prefix corect păstrat, celula țintă eliberată, dezgropare), reproducere (umple, golește, mutări directe, `match_top_face` prefer/require/ignore, fața de jos raportată „neverificabil”), sortare (stivuire pe zone până la `max_level`, `unknown_color_policy`). Motive explicite pentru ce nu se poate. Explorare: dacă o culoare lipsește, dar sub alte cuburi sunt niveluri ascunse, dezvelește întâi. |
| `brain/orchestration/session.py` | `ArmSession` (verificarea modului simulare/braț real, cronometrarea fiecărei comenzi, jurnal de durate, oprire între comenzi), `TimeBudget`. |
| `brain/orchestration/observer.py` | `LocationObserver`: `go_pose` → `settle_s` → captură → viziune, reîncercări, publicare în `PreviewHub`. |
| `brain/orchestration/memory_merge.py` | Îmbinarea observației (doar vârfurile) cu memoria (nivelurile ascunse); efectul așteptat al unei mutări; estimarea înălțimii din mărimea aparentă a feței de sus. |
| `brain/orchestration/proximity_check.py` | Verificarea prin senzor după pick/place: ținut (BELOW_RANGE) / cub sub gheară (≈30+s) / nimic (≈60+s); cu `s` necunoscut doar ținut/neținut. |
| `brain/orchestration/fsm.py` | FSM-ul `OBSERVE → PLAN → EXECUTE → VERIFY → … → DONE/ERROR`, tabel explicit de tranziții (`TRANSITIONS`), recuperare, raport. |
| `brain/adapters/arm/tee_arm.py` | `TeeArm`: comenzile merg și la serviciul colegilor (pornit cu `--sim`), și la `MockArm` (lumea simulată). |
| `brain/simulation/scenario.py` | Scenarii (exemplele S1/S2, aleatoare), percepția „oracol”, camera sintetică pentru o lume, verificarea finală pe adevăr. |
| `brain/simulation/schematic.py` | Schema 2D (vedere de sus) și animația GIF (Pillow, opțional; fără el PNG-uri). |
| `brain/cli/sim_run.py`, `run.py`, `tools.py` | `brain sim run`, `brain run`, `brain report timings`, `brain calib corners`. |

## Comenzi

```bash
python -m brain sim run --task tower --example --gif out/s2.gif            # exemplul S2, viziunea reală (camera sintetică)
python -m brain sim run --task tower --order red,yellow,green --target medium:r0c0 --dry-run
python -m brain sim run --task reproduce --example --buffers 2              # exemplul S1
python -m brain sim run --task reproduce --target-file tinta.yaml --buffers 1
python -m brain sim run --task sort --seed 5 --proximity --faults pick:grasp_fail@2,place:drop@4
python -m brain sim run --task all --random 20 --perception oracle --buffers 2
python -m brain sim run --task tower --example --service-url http://127.0.0.1:8765   # prin serviciul colegilor (npm run sim)

python -m brain run --task tower --order red,yellow,green,blue,red --target medium:r1c1 [--preview] [--execute]
python -m brain run --task memorize --target-out out/s1_target.yaml
python -m brain run --task reproduce --target-file out/s1_target.yaml [--execute]
python -m brain run --task sort [--execute]
python -m brain report timings --journal out/run-*/timings.jsonl
python -m brain calib corners --location medium --frames "out/live/frame_*.png" --write
```

Erori injectate (`--faults`): `metodă:tip@apel` sau `metodă:tip%probabilitate`, tipuri `timeout`, `comm_error`,
`unreachable`, `grasp_fail`, `drop`, `calibration`. Coduri de ieșire: 0 succes, 1 obiectiv neîndeplinit, 2 eroare
de intrare, 3 observație neacționabilă, 4 oprit (eroare de braț, Ctrl+C, refuz de siguranță).

## Bucla de execuție

1. **OBSERVE**: fiecare șablon activ (poza lui de observare). Celulele fără istoric care par de 2 cuburi (după
   mărimea feței de sus) primesc un nivel ascuns de culoare necunoscută; mai înalte → „înălțime incertă” (neatinse).
2. **PLAN**: planul complet din memorie; se execută doar prima mutare. Fără mutări: DONE (obiectiv îndeplinit sau
   motivele). Mutarea nu începe dacă nu se termină (estimat) în buget, cu rezerva păstrată.
3. **EXECUTE**: `pick` (uv corectat de paralaxă, yaw observat) → (senzor: ținut?) → `place` (yaw 0) → (senzor).
4. **VERIFY**: reobservarea locațiilor atinse (toate, dacă ținta e o zonă: un cub căzut poate ateriza oriunde),
   îmbinare cu memoria, comparație cu starea așteptată: pick ratat (reîncercare până la `max_pick_attempts`), cub
   căzut, celulă greșită. Memoria se scrie atomic după fiecare pas.

Recuperare: `False` / `MotionAborted` → locul e ocolit și brațul trece prin `go_pose(observe_*)`; place eșuat cu cubul
în gheară → cubul e pus înapoi; timeout / eroare de comunicație → ERROR fără `home`; observație neacționabilă →
reîncercări, apoi ERROR; dacă o reobservare de verificare eșuează după o mutare raportată reușită, memoria păstrează
starea așteptată, cu avertisment („neverificat vizual”).

Protecții adăugate după ce simularea cu viziunea reală le-a cerut (date sintetice, camera la 250 mm):
- fața unei stive înalte e mărită de paralaxă (×1,56 la 3 cuburi, ×1,9 la 4) și se revarsă în celulele vecine:
  apare ca fragment (`size_ratio` 0,3..0,9) sau degradează culoarea vecinului. O celulă neatinsă de ultima mutare,
  vecină cu o stivă înaltă și văzută degradat, își păstrează memoria; un fragment nu e luat drept cub;
- înălțimea unei stive necunoscute se estimează din `size_ratio`: 1 cub → ~1,15, 2 cuburi → ~2,13 (separabile);
  3 cuburi → 1,9..2,5 (nesigur, deci „înălțime incertă”). Pe robot depinde de `camera_height_mm` (NECALIBRAT).

## Rezultate măsurate

Toate pe Raspberry Pi. Timpii sarcinilor sunt SIMULAȚI (`SimDurations`: pick/place 4,5 s, go_pose 1,2 s), nu măsurați.

**Teste automate.** Python 3.8.20 + numpy 1.24.4 + OpenCV 4.6.0.66 (venv temporar): **402 trecute, 1 sărit**
(293 s), cu configurația finală. Python 3.13.5 + numpy 2.5.4 + OpenCV 5.0.0: suita completă a trecut cu
`line_width_mm` 1,0 (402); rularea finală, cu 0,8, a fost întreruptă la ~95% fără niciun eșec până acolo.
Testele noi: contractul MockArm/TeamArm cu un server HTTP fals (`test_arm_contract.py`), SimWorld/senzor/ceas/
paralaxă/validare (`test_simulation.py`), planificator (`test_planning.py`), orchestrare (`test_orchestration.py`),
randare + viziune pe cele 3 dimensiuni (`test_render_perception.py`), sarcini cap-coadă cu 60 de scenarii aleatoare
(`test_sim_tasks.py`), `brain run`/`memorize`/`calib corners` (`test_run_cli.py`) și serviciul real al colegilor
pornit cu `--sim` (`test_team_service.py`, marcaj `service`, sărit fără Node).

**Selftest viziune** (seed 42, 200 de scene sintetice; nu validează camera reală):

| Rulare | Detecție | Geometrii greșite | Ocupare | Culoare | Yaw ≤ 5° | Analiză (mediană) | Verdict |
|---|---|---|---|---|---|---|---|
| medium nominal (43 mm, linii 0,8 mm) | 100% | 0 | 100% | 100% | 100% | 923 ms | trece toate pragurile |
| medium hard | 93,6% | 1 (colț acoperit, seed 4200282, 28 px) | 100% | 100% | 99,6% | 955 ms | doar raportat; `occluded_corner_safe` 91,7% |
| small nominal (34 mm) | 82,1% | 1 (seed 4200295, 5,6 px) | 97,5% | 100% | 94,2% | 1371 ms | pică (ca înainte; foaia nu e folosită) |
| large nominal (48 mm) | 85,7% | 0 | 100% | 100% | 100% | 1325 ms | eșecurile sunt scenele ArUco (markerii sunt definiți pentru grila de 43 mm) |

Geometria greșită de pe `medium hard` apare doar cu linii subțiri: aceeași scenă dă 28 px cu linii de 1,0 mm,
22 px cu 1,5 mm și 1,3 px cu 2,5 mm. Liniile reale măsurate în poză au ~0,8 mm, deci riscul e real (restanță de
percepție, §6.8).

**Exemplele din regulament, cu camera sintetică și viziunea reală** (`out/etapa2/*.gif`):

| Sarcină | Mutări | Timp simulat | Rezultat |
|---|---|---|---|
| S2: turn roșu-galben-verde-albastru-roșu pe medium r1c1 | 5 | 55,8 s | reușit |
| S1: aranjamentul 3×3 amestecat pe același caroiaj, 2 puncte tampon | 9 | 99,0 s | reușit (fără puncte tampon: imposibil, raportat explicit) |
| S3: seed 7, senzor de proximitate, prindere ratată + cub căzut injectate | 13 | 145,5 s | reușit |

**Scenarii aleatoare** (`out/etapa2/random_*`, cu 2 puncte tampon):

| Percepție | Scenarii | Turn | Reproducere | Sortare | Timp simulat median / max |
|---|---|---|---|---|---|
| oracol (adevărul simulat) | 60 (seed 1000..1019) | 20/20 | 20/20 | 15/20 (cele 5 = mai multe cuburi de o culoare decât încap pe zonă; 100% din cele fezabile) | turn 40 / 56 s, reproducere 88 / 110 s, sortare 83 / 121 s |
| render (viziunea reală) | 15 (seed 2000..2004) | 4/5 (100% din fezabile) | 4/5 | 1/5 (33% din fezabile) | turn 34 / 56 s, reproducere 56 / 99 s, sortare 88 / 131 s |

Niciun scenariu nu depășește 300 s. Cauzele eșecurilor cu viziunea reală (nerezolvate):
- o stivă de 2 cuburi existentă de la start, într-o celulă de colț, nu e recunoscută din mărimea feței → brain-ul o
  crede de 1 cub, `pick` la nivelul 0 lovește stiva, iar memoria rămâne apoi cu înălțimea greșită (sort_2001: două
  prinderi „în gol”, locul e ocolit). La probă cuburile pornesc probabil direct pe masă, dar trebuie confirmat;
- culori nedecise (cuburi lăsate pe loc cu `unknown_color_policy: leave`) și o celulă cu culoarea greșită
  (reproduce_2004: verde în loc de galben) — viziune, nerecalibrată.

**Prin serviciul real al colegilor** (`--sim`, `TeeArm`, `calib_brain.json` de test): S2 și o sortare trec cap-coadă;
toate celulele lui `medium`, cele 4 zone și 2 puncte tampon acceptă `place` pe nivelurile 0..6.

## Limitări și riscuri (implementat / simulat / nevalidat pe hardware)

| Ce | Stare |
|---|---|
| Planificare S1/S2/S3, FSM, memorie, recuperare | implementat; testat în simulare (oracol și viziune reală pe camera sintetică) |
| Contractul cu serviciul colegilor | testat cu serviciul real pornit cu `--sim` (`calib_brain.json` de test) |
| Brațul real, camera reală, timpii | **nevalidat**: nicio mișcare reală, niciun cadru real |
| Pozele `observe_medium`, celulele, zonele, punctele tampon | **necalibrate** (de învățat cu `npm run calib`) |
| `camera_height_mm`, `nadir_uv`, `line_width_mm`, culorile HSV, latura cubului | NECALIBRATE; afectează paralaxa, estimarea înălțimii și culoarea |
| Senzorul de proximitate | doar simulat; `proximity.enabled: false`; `sensor_above_jaw_mm` necunoscut |
| Zonele S3 | camera nu le vede: conținutul lor vine din memorie (și din senzor, dacă e activ). Cuburile deja puse pe zone înainte de start trebuie date brain-ului (în simulare, prin memoria de pornire) |
| Cuburi în afara caroiajului (pe restul foii) | nu sunt văzute și nu pot fi luate: serviciul acceptă doar celule și puncte calibrate |
| Turnuri înalte | verificarea vizuală e mai slabă pe celulele vecine (vezi protecțiile de mai sus); stivele de ≥ 3 cuburi pe care brain-ul nu le-a construit nu pot fi măsurate sigur |
| Stive existente la start | estimarea înălțimii din imagine ratează uneori 2 cuburi în colțurile caroiajului (vezi rezultatele cu viziunea reală) |
| Linii subțiri (0,8 mm) | pe scenele „hard” cu un colț acoperit, viziunea a acceptat o dată o geometrie greșită |
| `TeeArm` | doar pentru teste; refuză brațul real (`status.sim` trebuie să fie true) |

## Ce urmează (Etapa 3, după confirmare)

`scripts/pi_doctor.sh`, `scripts/pi_install.sh`, planul de test live (`docs/live-test-plan.md`), calibrarea pozelor și
a celulelor pe braț, `calib corners` și HSV în lumina sălii, adaptorul HC-SR04 (dacă îl citește brain-ul), secțiunea
„Ziua concursului” din README.
