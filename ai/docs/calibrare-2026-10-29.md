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
