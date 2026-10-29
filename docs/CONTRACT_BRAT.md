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
- **`level`**: 0 înseamnă cub pe masă; nivelul k se află cu k × 30 mm mai sus (`CUBE_MM`).
- **`yaw_deg`**: rotația cubului, în grade.
  - Pentru un caroiaj, se măsoară în imaginea top-down rectificată, față de axa coloanelor (spre `c` crescător), pozitiv spre `r` crescător. Cubul e simetric la 90°, așa că orice valoare e acceptată; recomandat [-45, 45].
  - Pentru o zonă, se măsoară în sistemul robotului (x spre față).
- **`uv`** (opțional, doar la `pick`): poziția exactă a centrului cubului în coordonate de celulă continue, `[u, v]` între 0 și 3 (`u` de-a lungul coloanelor; centrul celulei `[r, c]` este `[c + 0.5, r + 0.5]`), corectată de paralaxă. Dacă e dată, înlocuiește centrul celulei. Dați-o când viziunea măsoară cubul deplasat în celulă.
- Toate apelurile sunt **blocante**: răspunsul vine după ce mișcarea s-a terminat. Cererile sunt executate strict una câte una.

## Metode

Toate sunt `POST /<nume>`, cu corp JSON (poate lipsi când nu sunt parametri). Răspunsul are forma `{"result": ...}`.

| Metodă | Corp | `result` | Semantică |
|---|---|---|---|
| `home` | – | `null` | brațul drept în sus (servo 1–5 la 90°), gheara neschimbată, 1,2 s |
| `go_pose` | `name`, `speed?` (`"fast"` \| `"slow"`) | `null` | poză învățată (`npm run calib -- poza NUME`); `"home"` există mereu. `slow` = timp dublu |
| `move_joints` | `angles` (5 sau 6), `time_ms` | `null` | 5 unghiuri = servo 1–5 cu gheara neschimbată; 6 = inclusiv gheara. Grade, convenția Arm_Lib |
| `open_gripper` / `close_gripper` | – | `null` | `GRIP_OPEN` / `GRIP_CLOSED` din config |
| `pick` | `location`, `cell?`, `yaw_deg?`, `level?`, `uv?` | `true` / `false` | vezi mai jos |
| `place` | `location`, `cell?`, `level?`, `yaw_deg?` | `true` / `false` | vezi mai jos |
| `read_joints` | – | `[6 × număr \| null]` | unghiurile citite; `null` = citire eșuată după 6 încercări. Merge și cu `GET` |
| `beep` | `n?` (1..10) | `null` | n bipuri de 100 ms |
| `status` | – | obiect | `sim`, `poses`, `sites` (nume → `"grid"` / `"point"`), `joints`, `grip`, `xyz`. Merge și cu `GET` |

**`pick`**: deschide gheara dacă nu e deschisă. Urcă la cel puțin 70 mm (`TRANSIT_MIN_Z`), merge deasupra cubului și coboară până la 45 mm deasupra lui (`HOVER_DZ`), apoi în 3 pași aproape verticali până la jumătatea cubului. Strânge gheara și urcă înapoi la 45 mm. Starea finală: cubul e în gheară, deasupra locului. Serviciul **nu** verifică dacă a prins cubul; verificarea se face din viziune.

**`place`**: aceeași apropiere. Coboară cu 3 mm mai sus decât poziția teoretică (`PLACE_GAP`), deschide gheara și urcă la 45 mm.

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
