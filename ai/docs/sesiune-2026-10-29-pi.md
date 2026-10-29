# Jurnalul sesiunii de pe Raspberry Pi — 2026-10-29

Tot ce s-a aflat și s-a făcut în sesiunea de lucru pe Pi (Claude Code, cu echipa la braț), în ordine.
Detaliile tehnice sunt în: [`stage2.md`](stage2.md) (Etapa 2), [`calibrare-2026-10-29.md`](calibrare-2026-10-29.md)
(calibrarea pe braț), [`questions-for-kinematics-team.md`](questions-for-kinematics-team.md) (constatări pentru serviciu).

## 1. Hardware și alimentare

- Pi: Debian 13 (trixie), aarch64, Python 3.13.5, Node 22.23.4, npm 10.9.9, IP în rețeaua locală 192.168.230.100.
- Placa brațului răspunde pe I2C la 0x15. Camera USB e `/dev/video0` (cele `/dev/video10..31` sunt ale Pi-ului).
- **Tensiune:** `vcgencmd get_throttled` = `0x50000` după pornire → Pi-ul a avut tensiune prea mică și și-a redus
  frecvența (nu și în momentul citirii). Brațul trebuie alimentat de la alimentatorul lui (12 V 5 A), nu prin Pi.
- **Cotul (servo 3), mai devreme în zi:** nu urca peste ~−30°, nici cu mâna, cu alimentarea brațului scoasă; după
  ce placa Pi a fost scoasă de pe braț s-a mișcat liber. Cauza probabilă: Pi-ul alimenta placa prin pini și servo-ul
  ținea poziția. Bănuiala principală pentru blocarea sub comandă: curent insuficient (cotul ridică cea mai mare
  greutate). Echipa a confirmat că placa Pi nu poate bloca fizic cotul. Brațul identic al altei echipe ajunge vertical.
- **Servo 4 (încheietura):** citirea dă `null` când încheietura e îndoită peste capătul cursei (impuls > 3100; s-a
  văzut 3155 și 3606). Cu încheietura dreaptă impulsul e 1955 ≈ mijlocul → servo-ul e montat corect.
  Servo 2 și 3 sunt și ele aproape de mijloc (2000, 1968) cu brațul drept.
- Specificațiile brațului (Yahboom DOFBOT, de la echipă): 5 grade de libertate + gheară; servo-uri seriale 15 kg×5
  + 6 kg×1; alimentare 12 V 5 A; anvergură 350 mm; zonă de lucru efectivă: rază ≤ 30 cm, semicerc în fața axului;
  repetabilitate ±0,5 mm; obiecte de 1–6 cm; cameră 0,3 MP cu unghi larg de **110°**, 640×480, 30 fps, focalizare
  manuală; dimensiuni 303×135×473 mm, 1256 g.

## 2. Instalarea pe Pi

- `ai/.venv`: Python 3.13 + numpy 2.5.4, opencv-contrib-python 5.0.0, PyYAML 6.0.3, pytest 9.2.0, Pillow 13
  (pentru GIF-uri), uv; pachetul `brain` instalat editabil.
- Serviciul colegilor: `npm run build`, testele lor 53/53; `i2c-bus` compilat (npm 10 nu blochează scripturile).
- Test de compatibilitate: venv temporar cu Python 3.8.20 + numpy 1.24.4 + OpenCV 4.6.0.66 (creat cu uv), 402 teste trecute.

## 3. Git și GitHub

- Repo: `CerneaBogdanAndrei/DYD-concurs-robotica` (public). Se lucrează pe `brain-etapa2`; `main` e adus la același
  commit prin fast-forward.
- Push de pe Pi: cheie SSH de deploy `~/.ssh/dyd_deploy` (adăugată în repo cu drept de scriere), alias `github-dyd`
  în `~/.ssh/config`, folosită doar ca URL de push (`git remote set-url --push origin git@github-dyd:...`).
  Amprenta GitHub verificată: `SHA256:+DiY3wvvV6TuJJhbpZisF/zLDA0zPMSvHdkr4UvCOqU`.
- Identitatea commit-urilor de pe Pi (doar în acest repo): `DYD Raspberry Pi <dyd@raspberrypi.local>`.
- Înainte de pull: copie `~/DYD-backups/ai-20261029-152458.zip`.
- Commit-uri: `cd992d5` (corecturi TeamArm), `4a69474` (Etapa 2: planificare, orchestrare, simulare, CLI),
  `d5afed6` (documentație, rezultate), `cf7b7ab` (calibrarea pe braț).

## 4. Foaia folosită la probă

- Echipa a primit trei foi A4, fiecare cu un singur caroiaj 3×3 (mic, mediu, mare), și a ales **foaia `medium`**.
- Măsurători (poza `examples/sablon_3x3_masuratori.jpeg`): celulă 4,3 cm; margini sus 1 cm, stânga 4,7 cm, dreapta
  2,8 cm, sub caroiaj 15,9 cm (pe lățime suma dă 20,4 cm, nu 21: ~6 mm de remăsurat); grosimea liniilor măsurată
  din poză ≈ 0,8 mm.
- Pe masă: caroiajul cu latura spre robot, centrat pe direcția brațului, apoi **mutat cu ~3 cm mai departe**
  (ca să încapă în cameră) și lipit cu bandă. Centrul lui e la ~25 cm de axul bazei în modelul serviciului.
  **Foaia nu se mai mută** (altfel calibrarea trebuie refăcută).

## 5. Etapa 2 (cod) — pe scurt

Planificare S1/S2/S3, orchestrare (FSM), simulare cap-coadă, CLI (`sim run`, `run`, `report timings`,
`calib corners`), 402 teste. Decizii noi: `planning.active_locations: [medium]`, puncte tampon opționale
(`planning.buffer_points`) pentru S1 cu 9 cuburi, stive până la 9 cuburi. Rezultatele: `stage2.md`.

## 6. Calibrarea pe braț (pași și ce s-a învățat)

1. **Poza de observare.** Prima poză ținută cu mâna avea încheietura cu ~40° peste capăt (servo 4 necitibil și
   necomandabil). Poza s-a căutat apoi din cod, cu camera urmărită de Claude:
   `[90,75,15,20]` prea aproape → `[90,75,35,10]` prea departe → `[90,75,28,5]`, `[90,85,30,3]`, `[90,80,29,5]`,
   `[90,83,29,4]`: caroiajul (~450 px) nu încăpea în 480 px → foaia mutată 3 cm → `[90,90,32,2]` „medium OK” →
   **`[90,90,29,2,90]`** centrat (margini ~50 px). Cu încheietura la capăt, camera nu poate urca mai sus.
2. **Orientarea:** sus în imagine = departe de robot; `r0c0` = stânga-departe (verificat cu un cub).
3. **Colțurile** (cu mâna, bip când toate motoarele sunt în interval): eroare de potrivire 2,4 mm; în modelul
   serviciului pasul celulelor iese 33–35 mm în loc de 43.
4. **Testul pe `r1c1`:** gheara centrată, la înălțime, paralelă cu liniile. `pick r1c1 → place r0c0`: reușit.
5. **Stivuirea:** pe nivelul 1 gheara ajunge ~1 cm mai departe de robot; cubul a căzut. Potrivirea lungimilor pe
   9 puncte: config (L1 105, L4 175) RMS 9,1 mm; cea mai bună (L1 116, L4 181) RMS 8,2 mm; rigla (L1 125, L4 160)
   RMS 16,7 mm. `config.ts` nu a fost modificat. Cauza dominantă: colțurile învățate la înălțimi diferite.
6. **Ultimul test:** `pick r0c0` (serviciu) → `place_corr.mjs 1 1 1 0.23` (pus pe `r1c1` nivel 1, cu 1 cm spre robot).
   Rezultatul fizic nu a fost încă confirmat de echipă.

## 7. Viziunea cu camera reală

- Din `observe_medium` camera privește oblic (~38° de verticală) cu un obiectiv de 110° (liniile apar ușor curbe).
- Fața de sus a unui cub apare deplasată ~0,8 celule spre departe (34 mm la 30 mm înălțime); se văd și fețele
  laterale, care au alte culori și pictograme decât fața de sus (cuburile au fețe diferite).
- Efect: un cub apare în 2–4 celule și cu culori amestecate → **`brain run` nu e încă utilizabil**.
- Ce merge: viziunea găsește caroiajul (gol sau cu un cub în colț), iar orientarea celulelor e corectă.
- De făcut: atribuirea cubului după bază (corecția de paralaxă cu punctul de sub cameră în afara caroiajului,
  aprox. 3 celule spre robot), culoarea doar de pe fața de sus, ignorarea fețelor laterale, eventual corecția
  distorsiunii obiectivului. `config/system.yaml` are încă `nadir_uv` în centrul caroiajului și
  `camera_height_mm` 250 (NECALIBRATE); validarea config cere acum ca `nadir_uv` să fie în interiorul caroiajului.

## 8. Procese și fișiere la final

- Serviciul brațului (`node dist/src/cli/server.js`, brațul real) rulează pornit de Claude în fundal (port 8765);
  previzualizarea camerei (`brain camera preview --location medium`, port 8080) e cea pornită de echipă.
  După o repornire a Pi-ului trebuie pornite din nou.
- `calib.json` (rădăcina repo-ului) e acum în git.
- Uneltele de calibrare: `ai/scripts/calib_live/`.

## 9. Pașii următori

1. Confirmarea fizică a ultimei stivuiri (cu corecție). Dacă e bună, corecția pe niveluri trebuie pusă în serviciu
   (`place` nu primește o poziție corectată) sau colțurile reînvățate la aceeași înălțime și refăcut testul.
2. Viziunea pentru vederea oblică (secțiunea 7), apoi `brain run` fără `--execute`, apoi cu `--execute`.
3. Zonele S3 (`npm run calib -- zona zone_red` …) și un punct tampon pentru S1 (`zona buffer_1`), trecut în
   `planning.buffer_points`.
4. Alimentarea: alimentatorul brațului pornit; urmărit `vcgencmd get_throttled`.
5. Etapa 3 din `next-session-prompt.md` (scripturi pentru Pi, plan de test live), după confirmarea echipei.
