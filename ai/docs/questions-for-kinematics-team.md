# Întrebări pentru echipa cinematică — integrare cu Brain (CRI 2026)

## Constatări din serviciul de mișcare (2026-10-29, commit `7ebca0d`, rulat cu `npm run sim`)

Testat de pe laptop (Windows, Node 24.18, npm 12.0.1) cu `TeamArm` din brain. Testele voastre: 53/53 trec.
Nimic de aici nu e testat pe braț. Multe întrebări de mai jos au primit deja răspuns în `docs/CONTRACT_BRAT.md`.

1. **`pick`/`place` pot întoarce `false` după ce brațul s-a mișcat.** `pitchFor` verifică doar punctul de jos
   și cel de apropiere, nu și tranzitul. `approach()` poate urca în pași (`line`) și abia apoi aruncă `Unreachable`.
   Reproducere: `home`; `place source [2,2] level 6` → `true` (gheara la z=243); `place source [2,2] level 7`
   → `false`, dar `status.xyz` devine z=263 și unghiurile se schimbă. Contractul spune „de regulă brațul nu
   s-a clintit”. Propunere: verificați tot traseul înainte de prima comandă către servo-uri, sau documentați
   că `false` poate lăsa brațul în altă poziție (fără să fi coborât).
2. **Tranzitul rămâne la înălțimea curentă.** `approach()` folosește `zT = max(zHover, TRANSIT_MIN_Z, z curent)`.
   După o poziție înaltă, o țintă mai depărtată de bază devine „inaccesibilă” la acea înălțime, deși e accesibilă
   de jos. Starea persistă până la `home`/`go_pose` (care resetează `curXyz`). Continuarea reproducerii de mai sus:
   `place dest [0,0]` → `false` („punct inaccesibil x=220 y=-140 z=263”); după `go_pose observe_dest` → `true`.
   Propunere: dacă tranzitul la z curent e inaccesibil, încercați înălțimi descrescătoare până la
   `max(zHover, TRANSIT_MIN_Z)`. Până atunci, brain-ul trimite `go_pose(observe_*)` între mutări și după orice `false`.
3. **npm 12 blochează compilarea `i2c-bus`.** `npm install` afișează „install scripts blocked … i2c-bus@5.2.3
   (install: node-gyp rebuild)”. Dacă pe Pi rulează npm ≥ 12, modulul nativ nu se compilează, iar `npm run server`
   pică cu „Pachetul i2c-bus nu e disponibil … rulează `npm install`” (sfat care nu ajută). npm sugerează
   `npm install-scripts approve i2c-bus`. Ce versiune de Node/npm e pe Pi?
4. **Numele locurilor.** `sim/calib.json` are `source`, `dest`, `observe_source`, `observe_dest`. Proba are trei
   șabloane pe aceeași foaie A4. Brain-ul folosește `small`, `medium`, `large`, `observe_small|medium|large` și
   `zone_red|yellow|green|blue`. Propunem aceste nume la calibrare (`npm run calib -- celule small` etc.).
5. **`yaw_deg` la zone** e în sistemul robotului (contractul). E un sistem drept (z în sus, yaw pozitiv antiorar
   văzut de sus)? Brain-ul nu vede zonele cu camera și trimite deocamdată `yaw_deg = 0` la zone.
6. **Accesibilitate pe niveluri (simulare, lungimi NECALIBRATE din `config.ts`):** cu `home` înainte de fiecare apel,
   `place` reușește pe nivelurile 0..6 în toate celulele `source`/`dest` și în toate zonele din `sim/calib.json`,
   dar nu și pe nivelurile 7..8. `pick` de pe nivelul 7 reușește în 2/9 celule. Un turn de 5 cuburi (nivelurile 0..4) e
   accesibil în simulare. Pe braț trebuie reverificat după calibrare.
7. **Mărunțișuri:** răspunsul 405 (`code: "metoda"`) nu apare în tabelul de erori. În `clients/python/team_arm.py`,
   `except TimeoutError` nu prinde `socket.timeout` pe Python < 3.10, deci un timeout ar ieși ca excepție brută
   (brain-ul are adaptorul propriu, nu e afectat).

## Constatări din Etapa 2 (2026-10-29, testate pe Raspberry Pi cu `node dist/src/cli/server.js --sim`)

Rulat pe Pi (Debian 13, aarch64, Node 22.23.4, npm 10.9.9): `npm run build` și testele voastre, 53/53.
`i2c-bus` e compilat (npm 10 nu blochează scripturile de instalare, deci constatarea 3 nu apare pe Pi).
Brain-ul rulează sarcinile cap-coadă prin serviciul vostru (`TeeArm`: fiecare comandă merge la serviciu și în
lumea simulată a brain-ului), cu `ai/tests/fixtures/calib_brain.json` (poziții de TEST, nu măsurători).

8. **Foaia aleasă.** Echipa a primit trei foi A4, fiecare cu un singur caroiaj 3×3, și a ales foaia `medium`
   (celulă 4,3 cm, măsurată: `ai/docs/examples/sablon_3x3_masuratori.jpeg`). Brain-ul folosește doar locurile:
   `medium` (caroiaj), poza `observe_medium`, zonele `zone_red|yellow|green|blue` (S3) și, opțional, punctele
   tampon `buffer_1`, `buffer_2` (S1: cu 9 cuburi pe 9 celule, o permutare cere un loc liber în afara caroiajului).
   Vă rugăm să calibrați exact aceste nume: `npm run calib -- celule medium`, `poza observe_medium`, `zona zone_red`, …,
   `zona buffer_1`. Brain-ul verifică la pornire `status.sites`/`status.poses` și refuză să miște brațul dacă lipsesc.
9. **`limite` în timpul unui `pick`/`place`.** Contractul spune că `false` și erorile 400 lasă brațul pe loc, dar
   `JointLimitError` poate apărea și pe traseu (după primii pași). Brain-ul tratează acum `limite` primit la
   `pick`/`place` ca „brațul s-ar fi putut mișca” (`MotionAborted`): trece prin `go_pose(observe_medium)` și reobservă.
   Confirmați dacă se poate întâmpla, sau dacă toate unghiurile sunt verificate înainte de prima comandă.
10. **Yaw la zone.** Brain-ul trimite acum `yaw_deg` neschimbat la zone (sistemul robotului) și cu semnul inversat
    doar la caroiaje (convenția voastră). La zone trimite deocamdată 0.
11. **Accesibilitate cu `calib_brain.json`.** Toate cele 9 celule ale lui `medium`, cele 4 zone și cele 2 puncte tampon
    acceptă `place` pe nivelurile 0..6 (cu `home` înainte). Turnul S2 de 5 cuburi (nivelurile 0..4) trece prin serviciu.
    Pe braț trebuie reverificat după calibrare.

### Răspunsuri deja date în `docs/CONTRACT_BRAT.md` (la întrebările de mai jos)

- Forma API-ului (1): serviciu HTTP local, `POST /<metodă>`; brain-ul are adaptorul `TeamArm`. Cererile se execută
  una câte una (o coadă).
- Semantica `pick`/`place` (2): blocante; `true` = secvența a rulat (fără confirmarea prinderii); `false` = inaccesibil,
  cu `reason`; timeout recomandat 60 s; erori `argument`/`limite`/`ruta`/`calibrare`/`hardware`.
- Convenții (3): `cell` 0..2, `level` k = k·30 mm, `uv` 0..3 corectat de paralaxă; yaw la caroiaj în imaginea redresată
  (semn opus convenției brain-ului), la zonă în sistemul robotului.
- Unghiuri (4): convenția Arm_Lib; servo 5 are 0..270°.
- Calibrare (5): `npm run calib` (poze, celule, zone); IK propriu, potrivirea caroiajului prin cele mai mici pătrate.
- Viteză (6): `fast` / `slow` (timp dublu); brain-ul trimite `medium` ca `slow`.
- I2C (7): serviciul e singurul proprietar al magistralei.
- Gripper (11): `GRIP_OPEN` 60°, `GRIP_CLOSED` 135° (de verificat pe cub); deschiderea parțială nu există.
- Rămân deschise: timpi măsurați pe braț (9), starea după o eroare de comunicație (10; brain-ul presupune
  „necunoscută” și nu trimite `home`), transformarea cameră ↔ flanșă (11).

## Constatări pe braț: gheara între cuburi vecine (2026-10-29)

12. **Gheara coboară deschisă la `GRIP_OPEN` = 60, mult peste latura cubului (30 mm), și lovește cuburile vecine.**
    `robot.pick` deschide gheara la `GRIP_OPEN` înainte de apropiere, indiferent de comanda creierului, iar
    `open_gripper(full=False)` nu există. Pe foaia mică, cu cuburi alăturate, coliziunea e aproape sigură.
    Cerere: (a) `GRIP_OPEN` măsurat pentru o deschidere de ~33–34 mm (cubul + 1,5–2 mm pe fiecare parte);
    (b) ideal, un parametru opțional `grip_open` la `pick`/`place`, ca brațul să se apropie larg doar când are loc.
    Măsurare: brațul în `home`, `move_joints` cu 6 unghiuri (gheara în pași de 5) și rigla între degete.
13. **Axa degetelor nu poate fi aleasă.** `wristForYaw` aduce mereu încheietura la ±45° de centru, deci un cub cu
    vecini la stânga și la dreapta nu poate fi prins cu degetele pe direcția liberă (rotit cu 90°). Cerere: un
    parametru (de ex. `finger_axis: "cols" | "rows"`) sau acceptarea unui yaw pe 0..180° fără reducere modulo 90°.

## Contractul inițial (înainte de `CONTRACT_BRAT.md`)

Brain-ul (percepție, memorie, planificare, coordonare) comandă brațul **exclusiv** prin contractul `ArmInterface` de mai jos. Nu importă `Arm_Lib` și nu accesează I2C. Ca să vă putem integra modulul fără modificări în logica noastră, avem nevoie de răspunsurile de mai jos.

## Contractul pe care îl apelăm

```python
class ArmInterface(Protocol):
    def home(self) -> None: ...
    def go_pose(self, name: str, speed: Literal["slow", "medium", "fast"] = "fast") -> None: ...
    def move_joints(self, angles: List[float], time_ms: int) -> None: ...   # ID 1..6, grade [0, 180]
    def open_gripper(self, full: bool = True) -> None: ...
    def close_gripper(self) -> None: ...
    def pick(self, location: str, cell: Optional[Tuple[int, int]] = None, yaw_deg: float = 0.0) -> bool: ...
    def place(self, location: str, cell: Optional[Tuple[int, int]] = None, level: int = 0, yaw_deg: float = 0.0) -> bool: ...
    def read_joints(self) -> List[float]: ...
    def beep(self, count: int = 1, duration_ms: int = 100) -> None: ...
```

## Convențiile pe care le propunem (confirmați sau corectați)

| Element | Propunere |
|---|---|
| `location` | `source`, `dest`, `staging`, `sort:<nume>` (pentru sortare), `observe:<nume>` (poziții de observare). |
| `cell` | `(row, col)`, 0..2. `(0,0)` = colțul stânga-sus al caroiajului **văzut din poziția de observare**; `row` crește în jos, `col` spre dreapta. Corespondența fizică o fixăm împreună pe masă. |
| `level` | 0 = cub pus direct pe masă; 1, 2, 3 = peste 1, 2, 3 cuburi (turn de max. 4). |
| `yaw_deg` | Rotația cubului față de axele caroiajului, în `[-45, 45)`, pozitiv = sens antiorar văzut de sus. Implicit nu cerem control de yaw (`yaw_control: disabled`) până la calibrare. |
| Unități | Lungimi în mm, unghiuri servo în grade, timp în ms la interfața hardware. |
| Unghiuri articulare | În convenția `Arm_Lib` (biblioteca inversează intern servourile 2, 3, 4). |
| Cub | 30 × 30 × 30 mm nominal; celulă 40 × 40 mm nominal (de măsurat). |

## Întrebări

1. **Forma API-ului.** Implementați direct `ArmInterface`, sau ne dați alt API pe care îl împachetăm noi într-un adaptor (`TeamArm`)? Ce versiune de Python folosiți? Cum livrați codul (pachet, modul, repo)? E thread-safe?
2. **Semantica `pick` / `place`.**
   - Sunt blocante până la finalul mișcării?
   - Ce timeout au?
   - Ce înseamnă exact `True` / `False`? (Noi interpretăm `True` ca „secvența a rulat fără eroare detectată”, nu ca dovadă că cubul a fost prins.)
   - Ce excepții aruncă și în ce situații?
3. **Convenții.** Sunteți de acord cu tabelul de mai sus (`location`, `cell`, `level`, semnul yaw, unități)?
4. **Unghiuri și servo 5.** Lucrați cu unghiuri în convenția `Arm_Lib`? Cum tratați servo 5, care pare să aibă domeniul 0–270° (nu 0–180°)?
5. **Calibrare și poziții.** Cine deține calibrarea masă ↔ bază și fișierul `poses.yaml`? Noi propunem teach-in pe colțuri + centrul fiecărui caroiaj, cu interpolare în spațiul articular și validare per celulă (același mecanism pentru nivelurile turnului). Vă ajută sau aveți IK?
6. **Viteză.** Cum mapați `speed` (`slow` / `medium` / `fast`) în `time_ms`?
7. **Magistrala I2C.** Cine deschide `/dev/i2c-1`? Trebuie un singur proprietar al magistralei, cu serializarea comenzilor.
8. **Siguranță.**
   - E `home()` sigur din orice poziție?
   - Ce înălțimi de apropiere/retragere folosiți per nivel de turn?
   - Aveți vreun model de coliziune?
9. **Timpi.** Aveți durate măsurate pentru `pick`, `place` și `go_pose`? (Avem nevoie de ele pentru bugetul de 5 minute per probă.)
10. **Erori.** În ce stare rămâne brațul după o eroare de comunicație? E permis retry-ul, sau trebuie reobservare / intervenție manuală?
11. **Gripper.**
    - Ce unghi de închidere folosiți pentru cubul de 30 mm?
    - Ce înseamnă „deschis parțial” (`open_gripper(full=False)`)?
    - Aveți transformarea cameră ↔ flanșă (din CAD sau măsurată)?

## Ce vă dăm noi

- `tests/test_arm_contract.py` (Etapa 2): teste de contract pe care adaptorul vostru trebuie să le treacă înainte de integrare.
- `MockArm`: implementarea simulată a aceluiași contract, ca referință de comportament.
