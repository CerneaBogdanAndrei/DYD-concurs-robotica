# Unelte folosite la calibrarea pe braț (2026-10-29)

- `start.sh <args calib>`: pornește `npm run calib` cu Enter trimis dintr-un FIFO (`echo > in`).
- `monitor.py [secunde] [--fara-viziune]`: citește impulsurile servo 1..5 (doar citire) și starea viziunii din
  previzualizare; bipăie când toate motoarele sunt în intervalul comandabil (și viziunea vede caroiajul).
- `hover.mjs r c nivel`: duce gheara deschisă unde ar fi pus un cub pe celula (r, c) la nivelul dat și așteaptă Enter.
- `place_corr.mjs r c nivel dv`: pune cubul ținut cu o corecție `dv` (celule, + = spre robot) pe direcția liniilor.

Toate folosesc modulele compilate ale serviciului (`dist/`) și `calib.json` din rădăcina repo-ului.
Rezultatele: `ai/docs/calibrare-2026-10-29.md`.

## `autocal/`: foaia s-a mutat -> recalibrare din cameră (2026-10-29 seara)
- `mutare_foaie.py [--aplica]`: duce brațul home -> `observe_small`, compară cele 16 intersecții ale caroiajului cu
  `ref/observe_small.jpg` și mută caroiajul `small` din `calib.json` cu cât s-a mutat foaia (fără model de cameră).
  Cere serviciul și previzualizarea pornite, foaia goală. Refuză mutări > 1 celulă sau > 8 grd (atunci: calibrare cu mâna).
- `tur_celule.mjs [DZ] [ms] [celule]`: gheara deschisă la DZ mm (implicit 20) peste prindere, pe celule, pentru verificare cu ochiul.
- `detect.py`: intersecțiile caroiajului cu precizie sub-pixel (linii ajustate cu parabole, ~0,3 px).
- `aplica_mutarea.mjs`: folosit de `mutare_foaie.py --aplica`.
