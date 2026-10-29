# Dofbot — Concurs Roboți Industriali

Soluție completă în Python (OpenCV + NumPy + `Arm_Lib` de la Yahboom) pentru cele două sarcini. Rulează integral pe Raspberry Pi 4, fără intervenție după start.

## Cum funcționează (pe scurt)

1. **Viziune.** Camera e pe încheietură, deci bratul merge într-o *poză de observare* fixă deasupra fiecărui caroiaj. Imaginea e rectificată (perspectivă → pătrat 360×360 px) pe baza celor 4 colțuri ale caroiajului, detectate automat la fiecare scanare (cu colțurile din calibrare ca rezervă). În fiecare celulă se caută fața de sus a cubului (`minAreaRect`).
2. **Culoarea feței de sus** se decide din **rama** feței (inelul exterior), nu din interior. Astfel desenele de pe cuburile Yahboom (doza portocalie, mărul roșu etc.) nu păcălesc clasificarea. Se raportează și tipul feței: `plina` (toată colorată) sau `imagine` (desen pe alb cu ramă colorată).
3. **Poziție și rotație.** Pentru fiecare cub se măsoară centrul exact (nu doar celula) și unghiul. Gheara coboară exact pe cub și se rotește (servo 5) ca să fie paralelă cu fețele lui.
4. **Cinematică inversă** analitică (servo 1–4), cu gheara cât mai verticală. Pentru puncte înalte sau îndepărtate se înclină automat. Coborârile se fac în pași, aproape pe verticală.
5. **Calibrarea brațului se face prin „învățare cu mâna”.** Se oprește cuplul, se pune gheara pe 4 celule, iar din unghiurile citite se calculează unde e caroiajul în spațiul robotului (potrivire prin cele mai mici pătrate). Erorile de lungime ale segmentelor se compensează astfel în mare parte.

### Sarcina 1
Se scanează caroiajul **A** (sursă) și se verifică dacă **B** e gol. Fiecare cub e mutat din A(r,c) în B(r,c). **Orientarea se păstrează automat**, fiindcă gheara nu răstoarnă cubul. Se rescanează A și se reîncearcă ce a rămas, apoi se scanează B și se afișează un raport OK/GREȘIT pe fiecare celulă.

### Sarcina 2
Se scanează caroiajul și se identifică culorile. Dacă pe poziția țintă e deja un cub de altă culoare, acesta e mutat în cea mai îndepărtată celulă liberă. Apoi se ia câte un cub din fiecare culoare, în ordinea dată de juriu, și se stivuiește pe țintă: nivelul *k* e plasat la `z + k·latura`. Cuburile se iau începând cu cele mai îndepărtate de turn.

## Instalare pe Raspberry Pi

Imaginea Yahboom pentru Dofbot are deja `Arm_Lib`, OpenCV și NumPy. Copiați folderul pe Pi (de ex. în `~/dofbot_concurs`). Dacă lipsește OpenCV:
```bash
pip3 install opencv-python numpy
```
**Opriți notebook-urile/serviciile Yahboom care folosesc camera sau brațul** (Jupyter, `dofbot` ROS), altfel camera sau portul serial sunt ocupate.

## Așezarea pe masă (important)

* Caroiajele trebuie să fie în zona accesibilă: **celulele la cel puțin ~14 cm de axa bazei și cel mult ~28 cm**. Mai aproape de 14 cm, Dofbot nu se poate apropia de sus (zonă moartă, din cauza ghearei lungi). Recomandat: A în stânga-față și B în dreapta-față, ca un „V”, cu centrul fiecăruia la ~20 cm de bază.
* Fixați caroiajele cu bandă adezivă. **Dacă se mută un caroiaj, refaceți calibrarea lui.**
* Lumină uniformă, fără umbre puternice. Masa albă e perfectă.

## Calibrare (o singură dată, ~10 minute)

Aveți nevoie de ecran pe Pi (monitor sau VNC) pentru pașii cu click.

```bash
cd ~/dofbot_concurs
python3 calibrate.py poza A      # mutați bratul cu mâna până camera vede tot A -> Enter -> click pe cele 4 colțuri
python3 calibrate.py poza B
python3 calibrate.py celule A    # gheara (deschisă, verticală) în jurul unui cub pe R1C1, R1C3, R3C1, R3C3
python3 calibrate.py celule B
python3 calibrate.py test A 2 2  # verificare: gheara trebuie să coboare centrată pe celulă
python3 calibrate.py culori A    # vizualizare live a recunoașterii + valori HSV sub mouse
```

**Ordinea click-urilor la colțuri:** colțul exterior al lui R1C1 → R1C3 → R3C3 → R3C1. Numerotarea R/C e cea folosită de juriu, deci stabiliți cu ei care colț e R1C1.

Interpretarea rezultatelor la `celule`:
* `z` ar trebui să fie ~15 mm (jumătate din cub). Dacă e mult diferit, ajustați `L1` / `L4` în `config.py`.
* Eroarea de potrivire ar trebui să fie sub ~3 mm. Altfel reînvățați celula respectivă. Se pot adăuga celule: `--celule r1c1 r1c3 r3c1 r3c3 r2c2`.
* Pasul coloanelor/liniilor trebuie să fie egal cu latura celulei măsurată cu rigla.

La `test`, dacă gheara e **rotită** față de liniile caroiajului, puneți `WRIST_SIGN = -1` în `config.py`.

## Rulare în concurs

```bash
python3 run.py s1                                              # Sarcina 1 (A -> B)
python3 run.py s2 --ordine rosu,galben,albastru,verde --pozitie 2,3
```
* `--ordine` e de **jos în sus**. Merge și cu diacritice sau săgeți: `"roșu → galben → albastru → verde"`.
* `--pozitie` e `linie,coloana` sau `R2C3`. Turnul se face pe caroiajul A; folosiți `--grila B` pentru celălalt.
* `--afisare` deschide ferestre cu ce vede camera, util în prezentare.
* Imaginile de diagnostic se salvează mereu în `debug/` (cadrul brut și grila rectificată cu cuburile marcate).
* `python3 run.py scan A` face doar recunoașterea, bun pentru verificări rapide înainte de start.

Timp estimat: ~9–10 s per cub (prindere + plasare). Sarcina 1 cu 9 cuburi durează sub 2 minute, mult sub limita de 5.

## Reglaje fine (`config.py`)

| Problemă | Ce modificați |
|---|---|
| Gheara nu strânge destul / cubul alunecă | `GRIP_CLOSED` mai mare (max ~150) |
| Gheara lovește cuburile vecine la coborâre | `GRIP_OPEN` mai mic (doar puțin peste lățimea cubului) |
| Prinde prea jos / prea sus | `GRASP_DZ` (± mm) |
| Turnul se dărâmă la eliberare | `PLACE_GAP` mai mic (1–2 mm) și `T_FINE` mai mare |
| O culoare nu e recunoscută | `calibrate.py culori A`, citiți H/S/V și ajustați intervalul din `CULORI` |
| Cub detectat unde nu e nimic | `MIN_FRACTIE` mai mare |
| Mișcări prea lente / smucite | `T_MOVE`, `T_FINE`, `FINE_STEPS` |

## Test fără robot (pe laptop)
```bash
python3 run.py s1 --sim --img-A pozaA.jpg --img-B pozaB.jpg
```
În acest mod comenzile pentru servo sunt doar afișate, iar „camera” citește imaginile date. Fișierul `calib.json` trebuie să existe (copiați-l de pe Pi). Puteți salva poze cu caroiajul din poza de observare și testa recunoașterea acasă.

## Idei pentru bonusul de creativitate (10 p)

* **Cuburile sunt de reciclare** (Yahboom „garbage sorting”): fiecare culoare corespunde unei categorii de deșeu (albastru – reciclabil, roșu – periculos, verde – organic, galben – altele). Robotul poate anunța ce deșeu a recunoscut, iar cu un model TFLite mic sau cu template matching pe desene poate face o a treia demonstrație de sortare a deșeurilor.
* **Afișare live pentru juriu** (`--afisare`): grila recunoscută, cu culori și unghiuri, în timp ce brațul lucrează.
* **Verificare în buclă închisă** (deja implementată la Sarcina 1): rescanare, reîncercare și raport final.
* Semnale sonore la start/final (deja incluse, prin buzzerul plăcii).

## Fișiere
| Fișier | Rol |
|---|---|
| `config.py` | toți parametrii (geometrie, gheară, culori HSV, timpi) |
| `dofbot.py` | cinematică directă/inversă, mișcări prinde/plasează |
| `grid.py` | modelul caroiajului în spațiul robotului, `calib.json` |
| `vision.py` | cameră, rectificare, detecția cuburilor și a culorii |
| `calibrate.py` | toate etapele de calibrare |
| `run.py` | Sarcina 1, Sarcina 2, scanare |
