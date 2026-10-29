# Calibrarea pe braț, 2026-10-29 (Raspberry Pi, brațul real)

> **Corectură (2026-10-29 seara):** foaia calibrată aici era foaia MICĂ, nu medium (configurația o numea
> atunci medium). În `calib.json` poza a fost redenumită `observe_small`, iar caroiajul `small`; punctele învățate
> sunt neschimbate. Măsurătorile corecte ale foii: celulă 45 x 43 mm (vezi `config/system.yaml`).

Foaia `medium` e mutată cu ~3 cm mai departe de robot decât poziția inițială și lipită cu bandă. Nu se mută.

## În `calib.json` (rădăcina repo-ului, nu e în git)
- `observe_medium` = [90, 90, 29, 2, 90]: camera vede tot caroiajul centrat (viziunea: „medium OK”).
  Servo 4 (încheietura) e la 2: aproape de capătul cursei; o poză mai verticală nu e comandabilă.
- `medium` (caroiaj), învățat cu mâna: r0c0 [95,28,54,55], r0c2 [80,33,42,66], r2c0 [96,49,30,39], r2c2 [79,50,33,36]
  → eroare de potrivire 2,4 mm; pasul celulelor în modelul serviciului 34,8 / 32,6 mm (fizic 43).
- Orientarea: r0 = rândul departe de robot, c0 = stânga (privind din spatele robotului). Confirmat cu un cub.

## Teste pe braț
- `calib test medium 1 1`: gheara centrată, la înălțimea corectă, paralelă cu liniile.
- `TeamArm`: pick r1c1 → place r0c0: reușit.
- Stivuire: place pe r2c1 nivel 1 → cubul a ajuns ~1 cm spre r1c1 și a căzut.
  Gheara dusă deschisă pe r2c1 nivel 1 (servo [87,54,48,18]): ~1 cm spre r1c1; lateral și înălțime bune.

## Puncte suplimentare pe nivelul 1 (cub peste cub, gheara pe cubul de sus)
- r0c1 [88,37,47,58], r1c1 [88,53,19,70], r2c1 [88,70,5,60]

## Potrivirea L1/L4 (9 puncte)
- config.ts (105 / 175): RMS 9,1 mm, max 21,6 mm
- cea mai bună: L1 116, L4 181: RMS 8,2 mm, max 22 mm → îmbunătățire mică; NU s-a modificat config.ts
- măsurat cu rigla: L1 125, L4 160 → RMS 16,7 mm (puncte de măsură probabil diferite de model)
- Eroarea dominantă: colțurile învățate la înălțimi diferite (r2c0 ~15 mm mai jos). De refăcut colțurile
  cu aceeași înălțime a ghearei, apoi reverificat nivelul 1.

## Viziune: camera privește oblic (camera la ~38° de verticală, 110° unghi larg)
- Fața de sus a unui cub apare deplasată ~0,8 celule spre departe; se văd și fețele laterale (alte culori/pictograme).
- Viziunea actuală atribuie greșit celulele (un cub apare în 2–4 celule) → `brain run` nu e încă utilizabil.
- De făcut: atribuirea după bază (corecția de paralaxă cu nadirul în afara caroiajului), culoarea doar de pe fața
  de sus, ignorarea fețelor laterale; eventual corecția distorsiunii lentilei de 110°.

## Stivuire cu corecție (ultimul test)
- pick r0c0 (serviciul) → place pe r1c1 nivel 1 cu `ai/scripts/calib_live/place_corr.mjs 1 1 1 0.23`
  (corecție de 0,23 celule ≈ 1 cm spre robot, pe direcția liniilor). Rezultatul fizic: de confirmat de echipă.
- Serviciul nu primește o poziție corectată la `place` (doar la `pick`, prin `uv`): o corecție permanentă pe niveluri
  trebuie făcută în serviciul colegilor (model cinematic) sau reînvățând colțurile la aceeași înălțime.

## Cotele din manualul Yahboom (2026-10-29, seara)
- `src/config.ts` și `referinta-python/config.py`: L1 104,5 (61,0 + 43,5), L2 = L3 82,85, L4 183,85 (73,85 până la axa
  servo 5 + 110,0 până la mijlocul degetelor). Unghiurile de rotație din manual (servo 1..4 și 6: 0..180, servo 5: 0..270)
  erau deja în `JOINT_MIN` / `JOINT_MAX`.
- `calib.json`: colțurile `small` recalculate cu FK-ul nou din aceleași unghiuri învățate (r0c0 [95,28,54,55],
  r0c2 [80,33,42,66], r2c0 [96,49,30,39], r2c2 [79,50,33,36]). La nivelul 0, gheara ajunge fizic în aceleași locuri.
  Modelul: pas coloane 35,5 mm, linii 33,8 mm, z prindere 1,2 mm, eroare de potrivire 2,5 mm.
- Comparare pe cele 7 puncte învățate (4 colțuri + 3 pe nivelul 1) cu o foaie rigidă 45 x 43 mm: RMS 16,6 mm (vechi) față de
  16,5 mm (carte). La nivelul 1, modelul se abate de punctele învățate pe direcția liniilor cu 4,9 / 18,8 / 17,5 mm
  (r0c1 / r1c1 / r2c1; vechi 4,5 / 17,2 / 15,4).
- Concluzie: lungimile segmentelor nu explică nici pasul caroiajului (35 față de 44 mm), nici eroarea de stivuire. Rămân de
  verificat: punctele învățate cu înclinări diferite ale ghearei (−39…−62°), zero-ul servo-urilor 2-4 și poziția reală
  a foii. Măsurătoarea cea mai utilă: distanța cu rigla de la axa bazei la centrul celulei r1c1.
- Postura `[90, 120, 0, 0, 90, 90]` -> `[-0,006; 0,116; 0,091] m` din document ține de DOFBOT-Pro (alt URDF): nu e folosită.

## Calibrarea automată din cameră (2026-10-29, seara)
- Foaia se mutase. Comparată cu imaginea din `observe_small` de la 18:14 (calibrarea cu mâna, foaia în poziția învățată),
  pe cele 11 intersecții de margine (cubul acoperea restul): **−23,7 mm pe coloane (spre c0 = stânga robotului), −8,1 mm pe
  linii (departe de robot), rotire −0,69 grd**. 6 reveniri home -> `observe_small`: unghiuri citite identice
  [90, 90, 29, 3, 89], mutarea ±0,2 mm. Presupunere: la 18:14 brațul citea aceleași unghiuri (nu s-au salvat).
- `calib.json`: caroiajul `small` mutat peste modelul învățat (prin celulele foii vechi): r0c0 (295,1; 41,0), r0c2
  (295,5; −30,1), r2c0 (227,5; 43,1), r2c2 (227,9; −28,0), z 1,2. Toate cele 9 celule sunt accesibile pe nivelurile 0..4.
- Proba fizică: gheara deschisă la 20 mm peste prindere pe r1c1, r0c0, r0c2, r2c2, r2c0 (cu o mutare intermediară,
  ~3 mm diferență): toate accesibile, unghiurile citite la ±2 grd de cele comandate. **Centrarea nu e confirmată cu ochiul**:
  camera privește înainte, peste gheară, și nu vede vârful ei.
- Repetabilitatea pozei: venind din altă parte decât home, cotul a citit 30 în loc de 29 -> 3-7 mm eroare la măsurare.
  De aceea `mutare_foaie.py` trece mereu prin home.
- Camera (din 9 vederi ale caroiajului): **nu se rotește cu servo 5**; f ≈ 740 px, deci ~47 grd pe orizontală (nu 110 grd);
  k1 ≈ −0,21. Calibrarea completă cameră-braț (hand-eye) a eșuat: rezidual 7 px, soluții nefizice (masa la −57 mm);
  cauza: unghiuri citite la grad întreg (1 grd = ~10 px) și vederi prea asemănătoare (în altele caroiajul ieșea din cadru).
- Poziția camerei în `observe_small`, măsurată din imagine (PnP pe cele 16 intersecții, f ≈ 740 px): H = 221 mm
  (210..233 pentru f 700..780), nadir uv = (1,86; 3,63) celule, axa la ~19 grd de verticală. `config/system.yaml` (small)
  le folosește în loc de estimările 220 / (1,5; 4,5).
- Cubul roșu pus pe r1c1 are ~40 mm (muchia de jos pe masă ~41 mm; viziunea, cu latura 40: 39,8 x 42,5 mm). Cu
  `cube.size_mm: 29`, pata e prea mare, e tăiată după celule și apare pe r0c1; cu 40: r1c1, încredere 1,0, fără motive.

## Calibrarea cu mâna, refăcută (2026-10-29, ~20:45)
- Proba pe cub a arătat că mutarea din cameră nu ajungea: cu ținta r1c1, gheara cobora deasupra lui r2c2 (văzut de echipă).
  Calibrarea de după-amiază nu mai corespundea brațului: ori foaia s-a mutat mai mult decât arată camera (camera se mișcă
  odată cu brațul), ori s-a schimbat zero-ul servo-urilor (brațul e folosit și de cealaltă echipă și a fost găsit căzut).
- Refăcută cu `npm run calib -- celule small`, cu un cub de 40 mm, la aceeași înălțime pe cele 4 colțuri:
  r0c0 [96,27,44,74], r0c2 [84,29,40,74], r2c0 [97,54,30,42], r2c2 [82,54,28,43] -> eroare de potrivire 0,7 mm,
  z 4..14 mm. În model: pas 31,3 / 31,2 mm, unghi între axe 83,6 grd (fizic 45 / 43 mm, 90 grd): modelul brațului
  „strânge” distanțele cu ~30%. La nivelul 0 nu contează (interpolare între colțuri); la stivuire poate conta.
- Referința camerei (`autocal/ref/`) refăcută după calibrare (`mutare_foaie.py --ref-nou`); baza citește 89 în observe_small.
