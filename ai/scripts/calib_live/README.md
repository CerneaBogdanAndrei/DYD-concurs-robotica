# Unelte folosite la calibrarea pe braț (2026-10-29)

- `start.sh <args calib>`: pornește `npm run calib` cu Enter trimis dintr-un FIFO (`echo > in`).
- `monitor.py [secunde] [--fara-viziune]`: citește impulsurile servo 1..5 (doar citire) și starea viziunii din
  previzualizare; bipăie când toate motoarele sunt în intervalul comandabil (și viziunea vede caroiajul).
- `hover.mjs r c nivel`: duce gheara deschisă unde ar fi pus un cub pe celula (r, c) la nivelul dat și așteaptă Enter.
- `place_corr.mjs r c nivel dv`: pune cubul ținut cu o corecție `dv` (celule, + = spre robot) pe direcția liniilor.

Toate folosesc modulele compilate ale serviciului (`dist/`) și `calib.json` din rădăcina repo-ului.
Rezultatele: `ai/docs/calibrare-2026-10-29.md`.
