# Contract: creier ↔ serviciul de mișcare

Creierul (Python: viziune, memorie, planificare) decide **ce** se mută. Serviciul de mișcare (acest repo, TypeScript pe Raspberry Pi) decide **cum**: unde e fiecare celulă în spațiul robotului, cinematica inversă, apropierea de sus și coborârea în pași.

Cele două procese comunică prin HTTP local, cu JSON. În `dofbot_brain`, `TeamArm` este clientul din [`clients/python/team_arm.py`](../clients/python/team_arm.py): îl copiați în `motion/team_arm.py` și gata.

## Pornire

| Unde | Comandă | Ce face |
|---|---|---|
| Pi, la concurs | `npm run server` | braț real prin I2C, calibrarea din `calib.json` |
| laptop / Windows | `npm run sim` | placă simulată, `sim/calib.json`, fără hardware |

Adresa implicită este `http://127.0.0.1:8765`. Opțiuni: `--port`, `--host`, `--calib FIȘIER`, `--verbose` (afișează fiecare comandă trimisă servo-urilor).

Modul simulat răspunde instantaneu și nu modelează scena. Scena simulată rămâne în `MockArm`-ul vostru. `npm run sim` servește la testele de contract și la integrare, nu la cronometrare.

## Convenții

- **`cell` = `[r, c]`, cu valori 0..2.** `r` este linia, `c` coloana. Celula `[0, 0]` este cea învățată ca `r0c0` la calibrare. Stabiliți cu juriul care colț al caroiajului este R1C1, apoi învățați-l ca `r0c0`.
- **`location`** este numele unui loc calibrat. `GET /status` le listează. Sunt de două feluri:
  - *caroiaj* (`source`, `dest`, …): `cell` este obligatoriu;
  - *zonă* (`zone_red`, …): un singur punct, iar `cell` lipsește sau e `null`.
- **`level`**: 0 înseamnă cub pe masă; nivelul k se află cu k × 29 mm mai sus (`CUBE_MM`, latura măsurată a cubului).
- **`yaw_deg`**: rotația cubului, în grade.
  - Pentru un caroiaj, se măsoară în imaginea top-down rectificată, față de axa coloanelor (spre `c` crescător), pozitiv spre `r` crescător. Cubul e simetric la 90°, așa că orice valoare e acceptată; recomandat [-45, 45].
  - Pentru o zonă, se măsoară în sistemul robotului (x spre față).
- **`uv`** (opțional, la `pick` și la `place`): poziția exactă a centrului cubului în coordonate de celulă continue, `[u, v]` între 0 și 3 (`u` de-a lungul coloanelor; centrul celulei `[r, c]` este `[c + 0.5, r + 0.5]`), corectată de paralaxă. La `pick`, `uv` poate ieși din caroiaj cu până la 3 celule (`PICK_OUTSIDE_CELLS`: cuburi scoase lângă caroiaj); acolo înălțimea primește corecțiile `z_out` din `calib.json`. Dacă e dată, înlocuiește centrul celulei. Dați-o când viziunea măsoară cubul deplasat în celulă; la `place`, când se stivuiește peste un cub care nu e în centrul celulei.
- **`grip_mm`** (opțional, doar la `pick`) și **`release_mm`** (opțional, doar la `place`): deschiderea INTERIOARĂ a ghearei, în mm, la abordare și la eliberare. Trebuie să fie mai mare decât cubul (`CUBE_MM`) și cel mult `GRIP_MAX_OPEN_MM` (60). Cer gheara calibrată în mm (`npm run calib -- gheara UNGHI`, vezi mai jos); altfel cererea e refuzată cu `calibrare`. Lipsă: cu gheara calibrată se folosește cubul + 2 × `APPROACH_MARGIN_MM` (abordare), respectiv cubul + 2 × `RELEASE_MARGIN_MM` (eliberare); necalibrată, `GRIP_OPEN`, ca înainte.
- Toate apelurile sunt **blocante**: răspunsul vine după ce mișcarea s-a terminat. Cererile sunt executate strict una câte una.

## Metode

Toate sunt `POST /<nume>`, cu corp JSON (poate lipsi când nu sunt parametri). Răspunsul are forma `{"result": ...}`.

| Metodă | Corp | `result` | Semantică |
|---|---|---|---|
| `home` | – | `null` | brațul drept în sus (servo 1–5 la 90°), gheara neschimbată, 1,2 s |
| `go_pose` | `name`, `speed?` (`"fast"` \| `"slow"`) | `null` | poză învățată (`npm run calib -- poza NUME`); `"home"` există mereu. `slow` = timp dublu |
| `move_joints` | `angles` (5 sau 6), `time_ms` | `null` | 5 unghiuri = servo 1–5 cu gheara neschimbată; 6 = inclusiv gheara. Grade, convenția Arm_Lib |
| `open_gripper` / `close_gripper` | – | `null` | `GRIP_OPEN` / `GRIP_CLOSED` din config |
| `pick` | `location`, `cell?`, `yaw_deg?`, `level?`, `uv?`, `grip_mm?` | `true` / `false` | vezi mai jos |
| `place` | `location`, `cell?`, `level?`, `yaw_deg?`, `uv?`, `release_mm?` | `true` / `false` | vezi mai jos |
| `read_joints` | – | `[6 × număr \| null]` | unghiurile citite; `null` = citire eșuată după 6 încercări. Merge și cu `GET` |
| `beep` | `n?` (1..10) | `null` | n bipuri de 100 ms |
| `status` | – | obiect | `sim`, `poses`, `sites` (nume → `"grid"` / `"point"`), `joints`, `grip`, `xyz`, `cube_mm`, `grip_calibrated` (gheara se poate comanda în mm), `grip_range_mm` (`[min, max]` măsurat sau `null`). Merge și cu `GET` |

**`pick`**: dacă brațul e sub 70 mm (`TRANSIT_MIN_Z`), urcă întâi pe verticală. Apoi pre-închide gheara la deschiderea de abordare (`grip_mm`; regula A: gheara nu călătorește larg deschisă printre cuburi). Merge deasupra cubului (cel puțin 70 mm), coboară la 45 mm deasupra lui (`HOVER_DZ`), apoi în 6 pași aproape verticali (`VERTICAL_STEPS`) până la jumătatea cubului. Strânge gheara și urcă înapoi la 45 mm. Starea finală: cubul e în gheară, deasupra locului. Serviciul **nu** verifică dacă a prins cubul; verificarea se face din viziune. Când un vecin e prea aproape, creierul mută centrul ghearei spre spațiul mai mare prin `uv` (regula B).

**`place`**: aceeași apropiere. Coboară cu 3 mm mai sus decât poziția teoretică (`PLACE_GAP`), deschide gheara doar la `release_mm` (regula C) și urcă strict vertical, în 6 pași, până la cel puțin 70 mm. Abia acolo brațul se poate mișca pe XY sau gheara se poate redeschide.

**Calibrarea ghearei** (o dată, cu șublerul): `npm run calib -- gheara UNGHI` pune doar gheara la unghiul dat și cere deschiderea interioară măsurată, în mm; punctele se țin în `calib.json` (`"gripper": [[unghi, mm], ...]`). Măsurați cel puțin 2 unghiuri care acoperă 29..45 mm (de ex. 135, 110, 90, 60); `npm run calib -- gheara` afișează tabelul.

**`false`** înseamnă punct inaccesibil, cu motivul în `reason`: `{"result": false, "reason": "punct inaccesibil x=… y=… z=…"}`. Accesibilitatea se verifică înainte de orice mișcare, deci de regulă brațul nu s-a clintit. Creierul poate sări cubul, ca în referință.

## Erori

Răspunsul are forma `{"error": {"code": "...", "message": "..."}}`. `TeamArm` o transformă în `ArmError(code, message)`.

| HTTP | `code` | Când |
|---|---|---|
| 400 | `argument` | parametru lipsă sau greșit (`cell` în afara 0..2, `cell` la zonă, JSON invalid) |
| 400 | `limite` | unghi în afara limitelor de siguranță (`JOINT_MIN` / `JOINT_MAX`; gheara max 150) |
| 404 | `ruta` | metodă necunoscută |
| 409 | `calibrare` | poza sau locul nu e învățat; mesajul spune ce comandă trebuie rulată |
| 500 | `hardware` | eroare I2C sau altă eroare neprevăzută |

Pe partea clientului mai apare `conexiune`, când serviciul nu răspunde. Timeout-ul recomandat este 60 s. Un `pick` sau `place` real durează câteva secunde; referința Python estima ~9–10 s pentru o prindere plus o plasare.

## Calibrare (partea de mișcare)

Calibrarea se face în sala de concurs, cu caroiajele fixate. Fișierul rezultat este `calib.json`, pe Pi. Serverul îl recitește singur când se schimbă.

```bash
npm run calib -- poza observe_source        # camera vede tot caroiajul sursă
npm run calib -- poza observe_dest
npm run calib -- celule source              # gheara pe r0c0, r0c2, r2c0, r2c2
npm run calib -- celule dest
npm run calib -- zona zone_red              # câte o zonă pentru Sarcina 3, dacă e cazul
npm run calib -- test source 1 1            # verificare: gheara coboară centrată pe celulă
```

Numele pozelor și ale locurilor sunt libere, dar trebuie să coincidă cu cele folosite de creier. Propunere: `observe_source`, `observe_dest`, `observe_<zonă>`, `source`, `dest`, `zone_<culoare>`.

Colțurile caroiajului în imagine (omografia) și pragurile HSV sunt ale creierului. După `go_pose("observe_…")`, așteptați ~0,6 s (`SETTLE_S` în referință) înainte de captură, ca brațul să se liniștească.

## Exemplu

```bash
curl -s -X POST localhost:8765/go_pose -d '{"name":"observe_source"}'
curl -s -X POST localhost:8765/pick    -d '{"location":"source","cell":[1,2],"yaw_deg":-12.5}'
curl -s -X POST localhost:8765/place   -d '{"location":"dest","cell":[1,2]}'
curl -s -X POST localhost:8765/place   -d '{"location":"source","cell":[0,1],"level":2}'
curl -s localhost:8765/status
```
