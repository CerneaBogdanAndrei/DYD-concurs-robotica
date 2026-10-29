"""
Configurare generala. Toate distantele in mm, unghiurile in grade.
Valorile marcate cu (!) TREBUIE verificate/masurate in laborator.
"""

# ---------------------------------------------------------------- BRAT
# Geometria Dofbot din manualul Yahboom / URDF (dofbot.urdf), 2026-10-29 (la fel ca src/config.ts).
L1 = 104.5   # masa -> axa servo 2 (umar): 61,0 + 43,5
L2 = 82.85   # axa servo 2 -> axa servo 3
L3 = 82.85   # axa servo 3 -> axa servo 4
L4 = 183.85  # axa servo 4 -> punctul de prindere: 73,85 (-> axa servo 5) + 110,0 (-> mijlocul degetelor)

# Conventie folosita (verificata cu `calibrate.py celule`):
#   servo 2/3/4 = 90 -> segment drept in sus; valori mai mici = inclinare in fata.
#   unghi absolut fata de orizontala:  a2 = s2, a3 = s2+s3-90, a4 = s2+s3+s4-180
BASE_SIGN = 1     # sensul bazei; e auto-consistent (calibrarea foloseste aceeasi formula)
WRIST_SIGN = 1    # (!) daca gheara apare rotita fata de cub, puneti -1
J5_CENTER = 90    # servo 5 la care gheara e "dreapta" fata de brat
J5_MIN, J5_MAX = 0, 180

GRIP_OPEN = 60    # (!) gheara deschisa (putin mai larga decat cubul)
GRIP_CLOSED = 135 # (!) gheara stransa pe cub (nu forta servo-ul peste 150)

HOME = [90, 90, 90, 90, 90]          # brat drept in sus

T_MOVE = 900      # ms, miscari mari
T_FINE = 450      # ms, coborare / ridicare (impartita in pasi)
T_GRIP = 400      # ms, inchidere / deschidere gheara
FINE_STEPS = 3    # in cati pasi se face coborarea "pe verticala"
SETTLE_S = 0.6    # s, asteptare dupa ce bratul ajunge in poza de observare

# inclinarea ghearei incercata la IK (in ordine): -90 = vertical, 0 = orizontal.
# Se prefera verticala; pentru puncte inalte/departate bratul inclina gheara.
PITCH_CANDIDATES = list(range(-90, 1, 5))

# ---------------------------------------------------------------- CUBURI
CUBE_MM = 30.0     # (!) latura cubului
HOVER_DZ = 45.0    # cat de sus deasupra punctului de prindere se face apropierea
PLACE_GAP = 3.0    # cubul e eliberat cu atatia mm mai sus decat pozitia teoretica
GRASP_DZ = 0.0     # corectie fina a inaltimii de prindere (+ = mai sus)
TRANSIT_MIN_Z = 70.0  # inaltimea minima a ghearei la deplasari laterale

# ---------------------------------------------------------------- CAMERA
CAM_INDEX = 0
CAM_W, CAM_H = 640, 480
FRAMES_AVG = 3        # cadre mediate (reduce zgomotul)
WARP = 360            # px latura caroiajului rectificat (120 px / celula)
CAM_HEIGHT_MM = 250.0 # (!) aprox. inaltimea camerei peste masa in poza de observare
REFINE_MAX_PX = 30    # cat se poate deplasa un colt detectat automat fata de calibrare

# ---------------------------------------------------------------- CULORI
# HSV OpenCV: H 0..180, S 0..255, V 0..255. Ajustati cu `calibrate.py culori A`.
CULORI = {
    "rosu":     [((0, 90, 60), (8, 255, 255)), ((168, 90, 60), (180, 255, 255))],
    "galben":   [((18, 90, 90), (35, 255, 255))],
    "verde":    [((40, 60, 35), (88, 255, 255))],
    "albastru": [((92, 90, 40), (130, 255, 255))],
}
CULOARE_BGR = {"rosu": (40, 40, 220), "galben": (0, 220, 240),
               "verde": (60, 170, 40), "albastru": (220, 120, 30)}

MIN_FRACTIE = 0.05     # fractia minima de pixeli colorati intr-o celula => exista cub
MIN_SCOR_CULOARE = 0.12
INNER_SCALE = 0.55     # interiorul fetei (unde e desenul) vs. rama (unde e culoarea)
FATA_PLINA_PRAG = 0.45 # >45% din interior colorat => fata plina (fara desen)

# nume acceptate in linia de comanda
ALIAS = {
    "rosu": "rosu", "red": "rosu", "r": "rosu",
    "galben": "galben", "yellow": "galben", "y": "galben",
    "verde": "verde", "green": "verde", "g": "verde",
    "albastru": "albastru", "blue": "albastru", "b": "albastru",
}

CALIB_FILE = "calib.json"
DEBUG_DIR = "debug"
