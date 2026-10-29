// Configurare generală, portată din referinta-python/config.py.
// Toate distanțele în mm, unghiurile în grade.
// Valorile marcate cu (!) TREBUIE verificate/măsurate în laborator.
// Parametrii de cameră și culori HSV țin de viziune (colegii) și nu sunt aici.

// ---------------------------------------------------------------- BRAȚ
// Geometria Dofbot (aprox. după URDF Yahboom). Măsurați cu rigla.
export const L1 = 105.0; // (!) masă -> axa servo 2 (umăr)
export const L2 = 83.0; // axa servo 2 -> axa servo 3
export const L3 = 83.0; // axa servo 3 -> axa servo 4
export const L4 = 175.0; // (!) axa servo 4 -> punctul de prindere (mijlocul degetelor)

// Convenție (verificată cu `calib celule`):
//   servo 2/3/4 = 90 -> segment drept în sus; valori mai mici = înclinare în față.
//   unghi absolut față de orizontală:  a2 = s2, a3 = s2+s3-90, a4 = s2+s3+s4-180
export const BASE_SIGN = 1; // sensul bazei; e auto-consistent (calibrarea folosește aceeași formulă)
export const WRIST_SIGN = 1; // (!) dacă gheara apare rotită față de cub, puneți -1
export const J5_CENTER = 90; // servo 5 la care gheara e „dreaptă” față de braț
export const J5_MIN = 0;
export const J5_MAX = 180;

export const GRIP_OPEN = 60; // (!) gheara deschisă (puțin mai largă decât cubul)
export const GRIP_CLOSED = 135; // (!) gheara strânsă pe cub (nu forțați servo-ul peste 150)

/**
 * Limite de siguranță per servo (1..6), verificate la fiecare comandă.
 * O comandă în afara lor e refuzată, nu tăiată. Gheara: max 150 (vezi GRIP_CLOSED).
 */
export const JOINT_MIN: readonly number[] = [0, 0, 0, 0, 0, 0];
export const JOINT_MAX: readonly number[] = [180, 180, 180, 180, 270, 150];

/** Brațul drept în sus (servo 1..5). */
export const HOME: readonly number[] = [90, 90, 90, 90, 90];

export const T_MOVE = 900; // ms, mișcări mari
export const T_FINE = 450; // ms, coborâre / ridicare (împărțită în pași)
export const T_GRIP = 400; // ms, închidere / deschidere gheară
export const FINE_STEPS = 3; // în câți pași se face coborârea „pe verticală”
export const SETTLE_S = 0.6; // s, așteptare după ce brațul ajunge în poza de observare

// Înclinarea ghearei încercată la IK (în ordine): -90 = vertical, 0 = orizontal.
// Se preferă verticala; pentru puncte înalte/depărtate brațul înclină gheara.
export const PITCH_CANDIDATES: readonly number[] = Array.from({ length: 19 }, (_, i) => -90 + 5 * i);

// ---------------------------------------------------------------- CUBURI
export const CUBE_MM = 30.0; // (!) latura cubului
export const HOVER_DZ = 45.0; // cât de sus deasupra punctului de prindere se face apropierea
export const PLACE_GAP = 3.0; // cubul e eliberat cu atâția mm mai sus decât poziția teoretică
export const GRASP_DZ = 0.0; // corecție fină a înălțimii de prindere (+ = mai sus)
export const TRANSIT_MIN_Z = 70.0; // înălțimea minimă a ghearei la deplasări laterale

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
