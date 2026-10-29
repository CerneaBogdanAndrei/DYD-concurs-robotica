# DYD: serviciul de mișcare pentru Dofbot

Concursul de roboți industriali (CRI 2026): braț Yahboom Dofbot cu Raspberry Pi 4 și cameră pe încheietură.

Acest repo este **partea de mișcare**, în TypeScript (Node.js) pe Raspberry Pi. Controlează brațul direct prin I2C, fără Python, și expune `ArmInterface`-ul creierului printr-un server HTTP local. **Creierul** (viziune, memorie, planificarea sarcinilor S1/S2/S3) e scris separat, în Python, de colegi. Contractul dintre cele două este în [docs/CONTRACT_BRAT.md](docs/CONTRACT_BRAT.md).

```
 creier (Python)                    serviciu de mișcare (acest repo)
 ┌──────────────────┐  HTTP/JSON    ┌───────────────────────────────────────────┐
 │ viziune, memorie │ ────────────▶ │ server.ts → service.ts → robot.ts → I2C   │
 │ planner S1/S2/S3 │  pick/place   │   (location+cell → xyz, IK, pick/place)    │
 │ TeamArm (client) │ ◀──────────── │   calib.json: poze + caroiaje + zone       │
 └──────────────────┘               └───────────────────────────────────────────┘
```

Logica de mișcare e portată din soluția de referință, [referinta-python/](referinta-python/):

- cinematica directă și inversă, cu aceeași convenție de unghiuri și `PITCH_CANDIDATES` -90..0;
- pick/place cu apropiere de sus, coborâre în pași și înălțime minimă de tranzit;
- modelul de caroiaj potrivit prin cele mai mici pătrate din celule învățate cu mâna;
- calibrarea prin învățare.

## Instalare pe Raspberry Pi

1. **Node.js ≥ 20.** Verificați cu `node -v`. Dacă lipsește sau e mai vechi, instalați-l, de exemplu prin [nvm](https://github.com/nvm-sh/nvm): `nvm install 22`.
2. **I2C activat**, iar utilizatorul trebuie să aibă acces la el:
   ```bash
   sudo raspi-config nonint do_i2c 0
   sudo usermod -aG i2c $USER            # apoi logout/login
   i2cdetect -y 1                        # trebuie să apară 15 (placa Yahboom)
   ```
3. **Unelte de compilare** pentru `i2c-bus`, care e modul nativ: `sudo apt install -y build-essential python3`.
4. În folderul proiectului:
   ```bash
   npm install
   npm run build
   npm test
   ```
5. **Opriți serviciile Yahboom** care folosesc brațul (Jupyter, ROS `dofbot`, aplicația). Altfel două programe vorbesc simultan pe I2C.

## Verificarea protocolului față de Arm_Lib

Protocolul I2C din [src/arm/protocol.ts](src/arm/protocol.ts) (registre, puls, inversarea servo-urilor 2/3/4, servo 5 pe 0..270) e reprodus după `Arm_Lib` publicat de Yahboom. Semnăturile și intervalele de unghi sunt confirmate de documentația Yahboom. **Registrele trebuie confirmate pe Pi**, cu fișierul real:

```bash
pip3 show -f Arm_Lib                               # unde e Arm_Lib.py
npm run compara:armlib -- codare --armlib <folderul cu Arm_Lib.py>
npm run compara:armlib -- citire                   # brațul pornit, nu se mișcă
```

- `codare` rulează `Arm_Lib.py` cu un `smbus` falsificat și compară octet cu octet 716 cazuri: scriere pe 6 servo-uri și pe unul, cuplu, buzzer, decodarea citirii. Nu atinge brațul. Merge și pe laptop, dacă aveți `Arm_Lib.py` copiat.
- `citire` compară unghiurile citite prin TS cu cele citite prin `Arm_Lib`.

Dacă `codare` raportează diferențe, corectați `protocol.ts` după `Arm_Lib.py` și rulați din nou.

## Calibrare (în sala de concurs, ~10 minute)

Caroiajele trebuie fixate cu bandă, la **14–28 cm** de axa bazei. Mai aproape de 14 cm, gheara lungă nu poate coborî de sus. Dacă se mută un caroiaj, refaceți-i calibrarea.

```bash
npm run calib -- poza observe_source       # brațul cu mâna până camera vede tot caroiajul -> Enter
npm run calib -- poza observe_dest
npm run calib -- celule source             # gheara deschisă, verticală, în jurul unui cub pe r0c0, r0c2, r2c0, r2c2
npm run calib -- celule dest
npm run calib -- zona zone_red             # Sarcina 3: câte o zonă pe culoare, dacă se sortează în afara caroiajelor
npm run calib -- test source 1 1           # verificare: gheara trebuie să coboare centrată pe celulă
npm run calib -- citeste                   # unghiuri + poziție în timp real, cu cuplul oprit
```

**Celulele sunt 0..2.** Stabiliți cu juriul care colț e R1C1 și învățați-l ca `r0c0`.

Cum citiți rezultatul de la `celule`:

- `z` ar trebui să fie ~15 mm (jumătate din cub). Dacă e mult diferit, ajustați `L1` / `L4` în [src/config.ts](src/config.ts).
- Eroarea de potrivire ar trebui să fie sub ~3 mm. Altfel reînvățați celula respectivă. Puteți adăuga celule: `--celule r0c0 r0c2 r2c0 r2c2 r1c1`.
- Pasul coloanelor și al liniilor trebuie să fie egal cu latura celulei, măsurată cu rigla.
- La `test`, dacă gheara e rotită față de liniile caroiajului, puneți `WRIST_SIGN = -1`.

Colțurile caroiajului în imagine și pragurile HSV se calibrează pe partea creierului.

## În ziua probei

```bash
npm run build               # o singură dată, după orice modificare
npm run server              # lăsați-l pornit; creierul îl folosește la toate sarcinile
```

Serverul afișează fiecare `pick`/`place` cu durata lui. Oprirea se face cu Ctrl+C. Calibrarea poate fi refăcută cu serverul pornit, pentru că `calib.json` e recitit automat.

## Fără robot (Windows / laptop)

```bash
npm install                 # i2c-bus nu se compilează pe Windows; e opțional, deci e în regulă
npm run sim                 # server pe placă simulată, cu sim/calib.json (source, dest, zone_*)
python clients/python/team_arm.py     # probă rapidă a clientului creierului
```

## Reglaje fine ([src/config.ts](src/config.ts))

| Problemă | Ce modificați |
|---|---|
| Gheara nu strânge destul / cubul alunecă | `GRIP_CLOSED` mai mare (max ~150) |
| Gheara lovește cuburile vecine la coborâre | `GRIP_OPEN` mai mic (doar puțin peste lățimea cubului) |
| Prinde prea jos / prea sus | `GRASP_DZ` (± mm) |
| Turnul se dărâmă la eliberare | `PLACE_GAP` mai mic (1–2 mm) și `T_FINE` mai mare |
| Mișcări prea lente / smucite | `T_MOVE`, `T_FINE`, `FINE_STEPS` |
| Brațul atinge cuburi la deplasări laterale | `TRANSIT_MIN_Z` mai mare |
| Gheara rotită față de cub | `WRIST_SIGN = -1` |

După orice modificare, rulați `npm run build`.

## Scripturi npm

| Script | Ce face |
|---|---|
| `build` | compilează TypeScript în `dist/` |
| `test` | build + toate testele (`node:test`) |
| `server` | serviciul de mișcare pe brațul real |
| `sim` | build + serviciul pe placa simulată |
| `calib` | calibrarea prin învățare (vezi mai sus) |
| `compara:armlib` | build + compararea protocolului cu `Arm_Lib` |

## Structură

| Fișier | Rol |
|---|---|
| [src/arm/protocol.ts](src/arm/protocol.ts) | protocolul plăcii (registre, unghi ↔ puls), funcții pure |
| [src/arm/arm.ts](src/arm/arm.ts) | `DofbotArm`: brațul peste o magistrală SMBus, operații serializate |
| [src/arm/i2c.ts](src/arm/i2c.ts) / [sim-board.ts](src/arm/sim-board.ts) | magistrala reală (`i2c-bus`, bus 1, 0x15) / placa simulată |
| [src/kinematics.ts](src/kinematics.ts) | FK/IK, `wristForYaw` (din `dofbot.py`) |
| [src/robot.ts](src/robot.ts) | pick/place, tranzit, coborâre în pași, limite de unghi (din `dofbot.py`) |
| [src/grid.ts](src/grid.ts) | modelul caroiajului, potrivire prin cele mai mici pătrate (din `grid.py`) |
| [src/calib.ts](src/calib.ts) | `calib.json`: poze cu nume, caroiaje, zone |
| [src/service.ts](src/service.ts) | `ArmInterface`: location + cell + level → mișcare |
| [src/server.ts](src/server.ts) | serverul HTTP |
| [src/cli/](src/cli/) | `server`, `calib` |
| [scripts/compara-armlib.ts](scripts/compara-armlib.ts) | comparare cu `Arm_Lib` |
| [clients/python/team_arm.py](clients/python/team_arm.py) | clientul pentru creier (`TeamArm`) |
| [src/config.ts](src/config.ts) | geometrie, gheară, timpi, limite |

## Diferențe față de referința Python

- Celulele sunt indexate **0..2** (ca în `ArmInterface`), nu 1..3.
- Planificarea sarcinilor (`run.py`) și viziunea (`vision.py`) **nu** sunt portate: ele sunt ale creierului.
- Calibrarea nu mai face click pe colțuri. Pozele de observare au nume, iar `zona` învață un singur punct pentru sortare.
- Erorile I2C sunt aruncate, nu doar afișate. O comandă în afara limitelor e refuzată, nu ignorată în tăcere cum face `Arm_Lib`.
- Limite de unghi per servo (`JOINT_MIN` / `JOINT_MAX`) și opțiunile `level` / `uv` la `pick`.
