// Configurare generală, portată din referinta-python/config.py.
// Toate distanțele în mm, unghiurile în grade.
// Valorile marcate cu (!) TREBUIE verificate/măsurate în laborator.
// Parametrii de cameră și culori HSV țin de viziune (colegii) și nu sunt aici.

// ---------------------------------------------------------------- BRAȚ
// Geometria Dofbot din manualul Yahboom / URDF (dofbot.urdf), primită de la echipă 2026-10-29.
// Unghiuri de rotație (manual): servo 1..4 și 6: 0..180, servo 5: 0..270 (vezi JOINT_MIN / JOINT_MAX).
export const L1 = 104.5; // masă -> axa servo 2 (umăr): 61,0 (bază -> servo 1) + 43,5 (servo 1 -> servo 2)
export const L2 = 82.85; // axa servo 2 -> axa servo 3
export const L3 = 82.85; // axa servo 3 -> axa servo 4
export const L4 = 183.85; // axa servo 4 -> punctul de prindere: 73,85 (-> axa servo 5) + 110,0 (-> mijlocul degetelor)

// Convenție (verificată cu `calib celule`):
//   servo 2/3/4 = 90 -> segment drept în sus; valori mai mici = înclinare în față.
//   unghi absolut față de orizontală:  a2 = s2, a3 = s2+s3-90, a4 = s2+s3+s4-180
export const BASE_SIGN = 1; // sensul bazei; e auto-consistent (calibrarea folosește aceeași formulă)
export const WRIST_SIGN = 1; // (!) dacă gheara apare rotită față de cub, puneți -1
export const J5_CENTER = 90; // servo 5 la care gheara e „dreaptă” față de braț
export const J5_MIN = 0;
export const J5_MAX = 180;

export const GRIP_OPEN = 60; // (!) gheara deschisă (puțin mai largă decât cubul)
export const GRIP_CLOSED = 140; // (!) gheara strânsă pe cub (nu forțați servo-ul peste 150); 135 abia atingea cubul de 29 mm (citea 133, goală 134)
export const GRIP_RELEASE = 115; // eliberarea fără gheara calibrată în mm: puțin peste cub (~33 mm), nu deschisă de tot

// Gheara în mm (date de echipă 2026-10-29). Deschiderile sunt INTERIOARE (între degete); legătura unghi servo 6
// -> mm se măsoară cu `npm run calib -- gheara UNGHI` și se ține în calib.json. Fără ea, gheara se deschide
// la GRIP_OPEN, ca înainte.
export const GRIP_MAX_OPEN_MM = 60.0; // deschiderea interioară maximă
export const GRIP_FINGER_MM = 6.0; // grosimea unui deget (lățimea exterioară = deschiderea + 2 x 6 mm)
// Deschiderea implicită (când clientul nu o cere): cubul + 2 x marja, pe fiecare latură.
export const APPROACH_MARGIN_MM = 5.0; // (!) abordarea: gheara călătorește și coboară pre-închisă la 29 + 10 mm
export const RELEASE_MARGIN_MM = 3.0; // (!) eliberarea: doar cât să elibereze cubul (29 + 6 mm)

/**
 * Limite de siguranță per servo (1..6), verificate la fiecare comandă.
 * O comandă în afara lor e refuzată, nu tăiată. Gheara: max 150 (vezi GRIP_CLOSED).
 */
export const JOINT_MIN: readonly number[] = [0, 0, 0, 0, 0, 0];
export const JOINT_MAX: readonly number[] = [180, 180, 180, 180, 270, 150];

/** Brațul drept în sus (servo 1..5). */
export const HOME: readonly number[] = [90, 90, 90, 90, 90];

export const T_MOVE = 1200; // ms, mișcări mari (2026-10-29: 900 -> 1200, mai lin)
export const T_FINE = 900; // ms, coborâre / ridicare (împărțită în pași legați, fără opriri între ei)
export const T_GRIP = 400; // ms, închidere / deschidere gheară
export const T_RELEASE = 1000; // ms, deschiderea la punere: lentă, să nu smucească cubul (echipa, 2026-10-30)
export const LIFT_SLOW_DZ = 20.0; // după punere, primii atâția mm de ridicare se fac încet (să nu agațe ceva alături)
export const T_LIFT_SLOW = 1500; // ms, pentru LIFT_SLOW_DZ
export const FINE_STEPS = 3; // în câți pași se face coborârea „pe verticală”
// Lângă cuburi (coborârea la prindere, ridicarea după eliberare): mai mulți pași = mai aproape de verticală.
export const VERTICAL_STEPS = 6;
export const SETTLE_S = 0.6; // s, așteptare după ce brațul ajunge în poza de observare
// După o comandă, înainte de următoarea: brațul se liniștește. Pașii unei coborâri/ridicări merg legați (0 ms),
// iar la capătul lor brațul stă SETTLE_MS înainte ca gheara să strângă sau să elibereze (2026-10-29: tremura).
export const PAUSE_MS = 120;
export const SETTLE_MS = 350;

// Înclinarea ghearei încercată la IK (în ordine): -90 = vertical, 0 = orizontal.
// Se preferă verticala; pentru puncte înalte/depărtate brațul înclină gheara.
export const PITCH_CANDIDATES: readonly number[] = Array.from({ length: 19 }, (_, i) => -90 + 5 * i);

// ---------------------------------------------------------------- CUBURI
export const CUBE_MM = 29.0; // latura cubului, măsurată de echipă 2026-10-29 (regulamentul: 30 mm nominal)
export const HOVER_DZ = 45.0; // cât de sus deasupra punctului de prindere se face apropierea
export const PLACE_GAP = -2.0; // cubul e eliberat la atâția mm față de înălțimea de prindere; 2026-10-30: +3 îl lăsa
// să cadă (echipa: „îi dă drumul prea devreme”), acum coboară până atinge foaia și apasă ușor
export const GRASP_DZ = 3.0; // corecție fină a înălțimii de prindere (+ = mai sus); 3 mm: degetele departe de foaie
export const TRANSIT_MIN_Z = 70.0; // înălțimea minimă a ghearei la deplasări laterale
// Pick-ul (doar el) primește poziții uv până la atâtea celule în afara caroiajului: cuburile scoase lângă caroiaj, în
// stânga, în dreapta sau spre robot (sortarea, 2026-10-30). Place rămâne în caroiaj (uv 0..3).
export const PICK_OUTSIDE_CELLS = 3;

// ---------------------------------------------------------------- CULORI
export const CULORI = ["rosu", "galben", "verde", "albastru"] as const;
export type Culoare = (typeof CULORI)[number];

/** Nume acceptate în linia de comandă (și în răspunsul viziunii). */
export const ALIAS: Readonly<Record<string, Culoare>> = {
  rosu: "rosu", red: "rosu", r: "rosu",
  galben: "galben", yellow: "galben", y: "galben",
  verde: "verde", green: "verde", g: "verde",
  albastru: "albastru", blue: "albastru", b: "albastru",
};

export const CALIB_FILE = "calib.json";
