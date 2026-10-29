# Prompt pentru sesiunea următoare: termină învățarea pe celule și traseul cubului (scris 2026-10-30, ~01:15)

Ești pe Raspberry Pi 4B, lângă brațul Yahboom DOFBOT, în `/home/admin/DYD-concurs-robotica`, pe ramura
`foaie-small-33mm`. Echipa stă lângă braț și îți scrie în română, iar tu le răspunzi în română, scurt.
**Nu mai adăugăm nimic nou.** Rulezi testul de mai jos până la capăt și raportezi. Nu schimbi parametri, praguri sau
logica de mișcare. Dacă testul se oprește din cauza unui defect care îl blochează, explici echipei ce s-a întâmplat și
ceri acordul înainte de orice schimbare de cod.

## Reguli (obligatorii)
1. Primul pas: `git fetch origin && git checkout foaie-small-33mm && git pull`. `brain-etapa2` = `main`. **Nu urca pe
   main cod cu teste picate.** Ramura asta are teste picate (lista e mai jos), deci nu se unește cu main.
2. Tot ce schimbi ajunge pe GitHub: commit-uri mici, cu mesaje în română. Fiecare mesaj se termină cu linia
   `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`. Urci cu `git push origin foaie-small-33mm`. Push-ul
   trece prin cheia de deploy `~/.ssh/dyd_deploy`, iar aliasul `github-dyd` e deja setat ca URL de push. Fișierele din
   `ai/out/preview/` nu se urcă.
3. Brațul e folosit și de altă echipă. Înainte de orice rulare cu mișcare verifici:
   - `i2cdetect -y 1`: trebuie să apară `15`;
   - `curl -s localhost:8765/read_joints`;
   - `vcgencmd get_throttled`.

   Dacă brațul e prăbușit, îl ridici câte un servo: 2 la 50, apoi la 90, apoi 4, 3, 1.
4. Spui echipei dinainte ce va face brațul și aștepți „gata”. Camera NU vede vârful ghearei, așa că întrebi echipa ce
   vede.
5. Nu folosi `pkill -f ...` pentru procesele brațului sau ale camerei: îți omoară și shell-ul. PID-ul îl găsești cu
   `ss -ltnp | grep -E ':8765|:8080'` sau `pgrep -af "brain camera preview"`, apoi dai `kill <pid>`.
6. Pentru `GRIP_OPEN` 60, `GRIP_CLOSED` 140 și `GRIP_RELEASE` 115 din `src/config.ts` întrebi înainte să le schimbi.
   Pentru punerea aprobată pe R1C1, echipa a spus „nu modifica nimic”.

## Ce rulează
- **Serviciul de mișcare** (TypeScript): `node dist/src/cli/server.js`, pe `127.0.0.1:8765`.
  - Metodele merg cu POST `/<metodă>` și un corp JSON, de exemplu `/beep {"n":2}`, `/go_pose {"name":"home"}`,
    `/place {...}`.
  - Starea se citește cu GET `/status` și `/read_joints`.
  - Dacă nu rulează, îl pornești cu
    `cd /home/admin/DYD-concurs-robotica && setsid nohup node dist/src/cli/server.js > <scratchpad>/server.log 2>&1 < /dev/null &`.
  - Testele TS: `npm test` (60/60 trec).
- **Camera și viziunea creierului**: `brain camera preview --index 0`, pe `:8080`.
  - Pagina live e la `http://192.168.230.100:8080/`.
  - Scrie `ai/out/preview/latest_observation.json` și imaginile de lângă el.
  - Pornirea, cu repornire automată:
    `cd /home/admin/DYD-concurs-robotica && setsid nohup bash ai/scripts/calib_live/camera_wait.sh > <scratchpad>/preview.log 2>&1 < /dev/null &`.
  - Previzualizarea are DOUĂ procese, părinte și copil, iar copilul ține portul. La repornire le oprești pe amândouă
    (`kill <pid1> <pid2>`); bucla o pornește din nou în ~2 s.
- Python: `ai/.venv/bin/python`. Testele creierului: `cd ai && .venv/bin/python -m pytest tests -q` (~3,5 min).

## Ce e pe masă
- Foaia `small`, lipită cu bandă: 3 × 3 celule de **33 × 33 mm** (pătrate perfecte, măsurate cu rigla). Foaia nu se
  mută.
- Un cub verde de **29 mm**.
- Celulele se notează `rXcY`. r0 e rândul cel mai departe de robot, r2 cel mai aproape. Coloana c0 e în stânga
  imaginii.
- Poza de start (observare) e `observe_small` = `[90, 90, 30, 2, 90]` (`calib.json`). Camera e lângă bază și vede
  foaia oblic.
- Pe rândul r0, fața de sus a cubului iese peste marginea de sus a imaginii. Viziunea o marchează
  `blob_clipped_by_image`, iar scriptul o completează. Așa trebuie, nu e o eroare.

## Starea la predare (2026-10-30, ~01:10)
- **Brațul a rămas sus, în poza de start, cu cubul în gheară**: gheara e comandată la 140 și citește 136. Dacă servo 6
  citește sub 138, cubul e încă în gheară. Cu acordul echipei, îl pui jos pe r0c0:
  ```
  curl -s -X POST -H 'Content-Type: application/json' -d '{"location":"small","uv":[0.499,0.499],"yaw_deg":0}' localhost:8765/place
  curl -s -X POST -H 'Content-Type: application/json' -d '{"name":"home"}' localhost:8765/go_pose
  curl -s -X POST -H 'Content-Type: application/json' -d '{"name":"observe_small","speed":"slow"}' localhost:8765/go_pose
  ```
  Dacă echipa a scos deja cubul cu mâna, întreabă-i unde l-au pus. Pe foaie trebuie să fie exact un cub.
- Învățarea s-a oprit la a 4-a punere pe r0c0, din cauza unui defect rezolvat în `28bd068`. După pick, cubul din gheară
  se vede jos în imagine, peste r2c1, ca o fâșie, iar scriptul a crezut că a rămas un cub pe foaie („NECLAR”).
- Ultimele commit-uri: `d712a5a` (viziunea: margine din paralaxă, cub tăiat de cadru), `00521ff` (completarea feței
  tăiate pe r0), `28bd068` (fâșia cubului din gheară), `29161d1` (datele învățate).

## Testul cerut de echipă
Se face exact ca pe R1C1, pe fiecare celulă pe rând:
1. Cubul vine de pe R1C1 și se pune pe celulă.
2. Se ridică și se pune din nou pe aceeași celulă de încă 3 ori, adică 4 puneri în total.
3. Cubul se duce înapoi pe R1C1.

Ordinea celulelor: R0C1, R0C0, R0C2, R1C0, R1C2, R2C0, R2C1, R2C2. La sfârșit, brațul face singur traseul
R0C0 → R0C1 → R0C2 → R1C0 → R1C1 → R1C2 → R2C0 → R2C1 → R2C2.

Făcut până acum:
- R1C1: proba inițială, 4 puneri;
- R0C1: 4 puneri, dintre care 2 bune;
- înapoi pe R1C1: 4 puneri, niciuna în toleranță;
- R0C0: 3 puneri din 4.

**Rămas:**
- R0C0 de la capăt (4 puneri);
- R0C2, R1C0, R1C2, R2C0, R2C1, R2C2, fiecare cu întoarcere pe R1C1;
- apoi traseul complet.

Rulezi în fundal, după verificările de la regula 3 și după „gata” de la echipă:
```
cd /home/admin/DYD-concurs-robotica
ai/.venv/bin/python -u ai/scripts/calib_live/autocal/traseu_cub.py --repetari 3 --intoarcere 11 00 02 10 12 20 21 22 > <scratchpad>/invatare3.log 2>&1 && ai/.venv/bin/python -u ai/scripts/calib_live/autocal/traseu_cub.py 00 01 02 10 11 12 20 21 22 > <scratchpad>/traseu_final.log 2>&1
```
`--intoarcere 11` primește o valoare, iar celulele se scriu după ea. Scriptul găsește singur cubul cu camera, deci
poate porni de oriunde, cu condiția să fie exact un cub pe foaie.

Semnale: 2 bipuri după fiecare pick, 1 bip după fiecare punere măsurată, 4 bipuri când pick-ul a ratat (atunci se
oprește). O punere durează ~70 s. Învățarea rămasă are ~45 de puneri (~50 min), iar traseul 10-30 de puneri
(~15-35 min).

Urmărești jurnalul, de exemplu cu Monitor pe
`tail -F <log> | grep --line-buffered -E "^r[0-2]c[0-2] -> |  pus|gol|neclar|opresc|Traceback|Error|Gata|refuzat|Camera|nu vede"`.
După fiecare celulă îi spui echipei cum a ieșit: câți mm de centru, cât de rotit, BINE sau DE CORECTAT.

Ce face scriptul la o mutare (nu schimba nimic din asta):
1. Din poza de start face două analize la rând, care trebuie să vadă cubul în același loc (±1,5 mm) și să-i dea
   rotirea.
2. Pick pe cub, cu corecția laterală învățată pe celula aceea.
3. Verifică prinderea: gheara citește cel puțin 3 sub 140 și nu se vede niciun cub rămas pe foaie.
4. Place pe centrul celulei, luat din liniile salvate din `ref/observe_small`. Poziția e corectată cu abaterile
   învățate (medie ponderată cu 1/d², maxim 8 mm), iar rotirea încheieturii vine din modelul rotirii.
5. Revine la poza de start și măsoară. „BINE” înseamnă: centrul cubului la cel mult 2 mm de centrul celulei, cel mult
   0,4 mm peste linii și rotit cel mult 1,5°. Altfel ridică și pune din nou, de cel mult 3 ori. La învățare face 4
   puneri oricum.
6. După fiecare punere scrie totul în `ai/scripts/calib_live/autocal/invatat.json`. Jurnalul merge în
   `traseu_invatare.json` la învățare (se suprascrie la fiecare rulare) sau în `traseu_cub.json` la traseu.

## Dacă se oprește
| Mesajul din jurnal | Ce faci |
|---|---|
| „Camera nu vede un singur cub ...” | Te uiți pe pagina live: o mână sau alt obiect pe foaie, sau cubul răsturnat. Întrebi echipa. |
| „Pick ratat ... (gheara deschisă)” | Cubul e pe foaie. Reiei cu celulele rămase. |
| „NECLAR ... gheara închisă” | Citești servo 6. Dacă e sub 138, cubul e în gheară: îl pui jos (comenzile de mai sus) și reiei. |
| „După place camera nu vede bine cubul” | Te uiți pe pagina live și întrebi echipa cum stă cubul. |
| „place refuzat” sau „pick refuzat” | Punct inaccesibil. Notezi și întrebi echipa. |

La reluare: întâi dai commit la `traseu_invatare.json`, pentru că se suprascrie. Apoi scoți din comandă celulele
terminate (le vezi în jurnal) și pornești din nou. Ce s-a învățat rămâne în `invatat.json`.

## La final
- Raport pentru echipă, pe fiecare celulă: câte puneri, la câți mm de centru și cât de rotită a ieșit ultima, BINE
  sau nu; plus durata totală.
- Commit și push pentru `invatat.json`, `traseu_invatare.json` și `traseu_cub.json`.
- Actualizezi memoria proiectului (`brain-etapa2-pi.md`).

## Ce s-a observat (NU implementezi fără acordul echipei)
1. Pe direcția brațului, abaterea punerii depinde de celula de unde a fost luat cubul, pentru că degetele centrează
   cubul doar lateral.

   | Mutare | Abatere (ajuns − trimis) |
   |---|---|
   | pe aceeași celulă | ~−1 mm |
   | r1c1 → r0c1 | −5,7 mm |
   | r0c1 → r1c1 | +2,6 mm |

   Modelul de acum le amestecă. De aceea, după o mutare între rânduri, prima corecție sare ~4 mm în partea cealaltă,
   iar a doua o readuce.
2. Rotirea cubului la punere e întâmplătoare, cam ±2-3°; pe r0c0 a ajuns până la ±4,5°. Toleranța de 1,5° iese rar.
   k (cât din rotirea încheieturii ajunge pe cub) e măsurat din perechi de puneri și e zgomotos:
   [0,54; −0,09; 0,37; 0,12; 0,06]. Mediana e 0,12, limitată la 0,15, așa că încheietura ajunge la limita de ±25°.
3. Lateral, abaterea ghearei e stabilă pe r0c1 (−4,7 ± 0,2 mm). Pe r1c1 s-a mutat de la −2,6 mm (proba de la ~00:20) la
   −1..+1,7 mm, odată cu comanda încheieturii.
4. Colțurile foii sunt la ~6,2 px față de imaginea de referință, din cauza jocului bazei. Scriptul folosește atunci
   liniile din imaginea curentă (apare avertismentul „ATENȚIE: colțurile ...”). E normal.

## Teste picate (de aceea ramura nu se unește cu main)
- Trei teste sintetice de viziune pe foaia small, picate de dinainte:
  - `test_pipeline::test_best_fit_wins_among_candidates_of_same_grid[4200228-nominal-small]`;
  - `test_render_perception::test_three_sizes_single_sheet[small]`;
  - `test_render_perception::test_neighbor_templates_do_not_confuse_the_grid[small-2]`.
- `test_team_service::test_tasks_through_real_service[sort]`. Folosește un serviciu simulat, nu brațul. De la
  `PLACE_GAP` −2, punerea pe `zone_yellow`, nivelul 1, cere x=138, z=90 în fixture-ul foii medium
  (`ai/tests/fixtures/calib_brain.json`), iar IK-ul nu ajunge acolo. Nu afectează foaia small.
- Restul trece: creierul 431 de teste, TS 60/60.

## Fișiere
- `ai/scripts/calib_live/autocal/traseu_cub.py`: scriptul de învățare și traseu.
- `invatat.json`: modelul învățat. Conține abaterile laterale, abaterile de punere (x, y), rotirile și perechile k.
- `ref/observe_small.jpg` / `.json`: foaia goală din poza de start; liniile celulelor se iau de aici.
- `calib.json` (rădăcina repo-ului): poza `observe_small` și colțurile caroiajului `small` în sistemul robotului.
- `ai/config/system.yaml`: foaia small, cu celula de 33 mm, camera la 163,7 mm și nadirul la (1,60; 3,67).
- `src/config.ts`:

  | Parametru | Valoare |
  |---|---|
  | `GRIP_OPEN` | 60 |
  | `GRIP_CLOSED` | 140 |
  | `GRIP_RELEASE` | 115 |
  | `T_RELEASE` | 1000 |
  | `PLACE_GAP` | −2 |
  | `LIFT_SLOW_DZ` | 20 |
  | `T_LIFT_SLOW` | 1500 |
- Istoricul zilei: `ai/docs/next-session-calibrare-testare.md`, `ai/docs/calibrare-2026-10-29.md`.
