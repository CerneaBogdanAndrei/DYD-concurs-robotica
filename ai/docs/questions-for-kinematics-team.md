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
