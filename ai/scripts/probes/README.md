# Sonde de diagnostic (sesiunea 2026-10-29)

Scripturi ad-hoc folosite la audit. Nu sunt teste automate: se rulează manual din `ai/`, cu mediul activ.

| Script | Ce face | Cerințe |
|---|---|---|
| `probe_team_arm.py` | toate metodele `TeamArm` + accesibilitatea pe niveluri | `npm run sim` pornit |
| `probe_reach.py` | accesibilitate pe niveluri cu/fără `home()`; tranzitul la înălțimea curentă | `npm run sim` pornit |
| `probe_neighbors.py [N]` | randare cu șabloane vecine → percepție pe small/medium/large; eroarea uv după paralaxă | – |
| `diag_candidates.py SEED DIFICULTATE [LOCAȚIE]` | candidații de grilă pentru o scenă din selftest și eroarea fiecăruia | – |
| `diag_polish.py SEED:DIF:LOC ...` | efectul trecerilor repetate de rafinare | – |

Exemplu: `.venv\Scripts\python scripts\probes\diag_candidates.py 4200295 nominal small`
