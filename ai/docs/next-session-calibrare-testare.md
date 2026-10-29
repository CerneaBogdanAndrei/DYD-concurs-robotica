# Prompt pentru sesiunea următoare: calibrare și teste pe brațul real (scris 2026-10-29, ~20:55)

Lipește tot ce e sub linie într-o sesiune nouă de Claude Code pornită pe Raspberry Pi, în `/home/admin`.

---

Lucrezi pe Raspberry Pi 4B, conectat la brațul Yahboom DOFBOT (I2C bus 1, adresa 0x15) și la camera USB de pe încheietură.
Repo: `/home/admin/DYD-concurs-robotica` (GitHub `CerneaBogdanAndrei/DYD-concurs-robotica`, public). Răspunde în română.

## Reguli
1. **Fă `git pull` ÎNAINTE de orice** (`git fetch origin` și ce ramuri sunt în lucru: `brain-etapa2` = `main`, plus `cub-40mm`).
2. **Orice modificare se urcă pe GitHub.** Commit-uri mici, mesaje în română. Push prin cheia de deploy: remote-ul de push
   e deja `git@github-dyd:...` (cheia `~/.ssh/dyd_deploy`). Identitatea commit-urilor e setată local în repo.
   `main` se aduce la `brain-etapa2` prin fast-forward (`git push origin brain-etapa2:main`), cum s-a făcut până acum.
   Nu urca pe `main` cod cu teste picate: lucrul neterminat stă pe o ramură separată.
3. Brațul e **comun cu altă echipă**: poate fi găsit căzut, fără curent sau mutat. Înainte de orice mișcare:
   `i2cdetect -y 1` (trebuie `15`), `curl -s localhost:8765/read_joints`, `vcgencmd get_throttled`. Dacă e căzut
   (ex. umărul la ~8°), ridică-l câte un servo (servo 2 la 50 apoi 90, apoi 4, 3, 1), cu verificare după fiecare.
4. Echipa e lângă braț și testează cu tine: spune-le înainte ce face brațul, folosește bipuri la opriri, întreabă-i ce văd.
   Camera NU vede vârful ghearei: unde coboară gheara află doar de la ei.
5. Citește `CLAUDE.md` (rădăcina repo-ului, ignorat de git), `docs/CONTRACT_BRAT.md`, `ai/docs/calibrare-2026-10-29.md`
   și `ai/scripts/calib_live/README.md` înainte să schimbi ceva.

## Ce rulează și cum
- Serviciul de mișcare (TypeScript): `npm run build && node dist/src/cli/server.js` → `http://127.0.0.1:8765` (doar local).
  Recitește `calib.json` singur când se schimbă fișierul; `config.ts` cere rebuild + repornire.
- Pagina de lucru / camera: `cd ai && .venv/bin/python -m brain camera preview` → `http://192.168.230.100:8080/`
  (din rețea). Ultima analiză: `ai/out/preview/latest_observation.json`, `latest_raw.jpg`, `latest_annotated.jpg`
  (fișierele din `ai/out/preview/` sunt urmărite de git, dar NU se urcă: se schimbă la fiecare analiză).
- La oprirea proceselor nu folosi `pkill -f "brain camera preview"` (omoară și shell-ul care rulează comanda): ia PID-urile
  cu `pgrep -af` și `kill <pid>`.
- Calibrarea cu mâna, cu Enter trimis din altă consolă: `bash ai/scripts/calib_live/start.sh celule small`, apoi
  `echo > ai/scripts/calib_live/in` după fiecare „gata” al echipei; ieșirea în `ai/scripts/calib_live/log.txt`.
  ATENȚIE: la fiecare punct cuplul se oprește (brațul cade dacă nu e ținut), după Enter brațul se întărește și urcă singur.

## Starea la predare
- Ramura locală pe Pi: `cub-40mm` (urcată). `brain-etapa2` = `main` = ultimul commit stabil.
- Cotele brațului din manualul Yahboom sunt în `src/config.ts`: L1 104,5, L2 = L3 82,85, L4 183,85 (73,85 + 110).
  Față de valorile vechi (105/83/83/175) schimbă poziția cu sub 2 mm.
- `calib.json`: poza `observe_small` = [90, 90, 29, 2, 90]; caroiajul `small` **refăcut cu mâna la ~20:45** cu un cub de
  40 mm: r0c0 [96,27,44,74], r0c2 [84,29,40,74], r2c0 [97,54,30,42], r2c2 [82,54,28,43], eroare de potrivire 0,7 mm.
- Foaia: caroiaj 3x3, celule 45 mm (coloane) x 43 mm (linii); r0 = rândul departe de robot, c0 = stânga privind din
  spatele robotului. Se mișcă ușor când lucrează cineva lângă ea: ideal, lipiți-o cu bandă.
- Cuburile: echipa lucrează acum cu **cuburi de 40 mm** (viziunea: 39,8 x 42,5 mm). Pe `cub-40mm`: `CUBE_MM = 40`
  (`src/config.ts`) și `cube.size_mm: 40` (`ai/config/system.yaml`); testele TS trec (58), **21 de teste ale brain-ului
  pică** (scene sintetice gândite pentru 29 mm).
- Gheara (servo 6, mai mare = mai închis): deschis la 0 ≈ 60 mm interior (nemăsurat cu șublerul); **contact pe cubul de
  40 mm la 95** (măsurat: servo-ul rămâne la 95 când i se cere 100, 105); la 30 e prea îngustă (a împins cubul);
  `GRIP_OPEN = 60` / `GRIP_CLOSED = 135` din `config.ts` sunt pentru cuburile de 29 mm și NU sunt schimbate
  (utilizatorul a refuzat schimbarea lor; întreabă înainte). `proba_pick_place.mjs` folosește 0 / 103 doar în script.
- Camera în `observe_small` (măsurată din imagine): ~221 mm deasupra mesei, nadir uv (1,86; 3,63), axa la ~19° de verticală,
  f ≈ 740 px (~47° pe orizontală, NU 110°), k1 ≈ −0,21; nu se rotește cu servo 5. Valorile sunt în `system.yaml` (small).
  Cu latura 40 mm viziunea vede corect cubul pe r1c1 (încredere 1,0); cu 29 mm îl punea pe r0c1 (pata tăiată după celule).
- Repetabilitate: brațul revine identic în `observe_small` doar venind din `home` (`go_pose home`, apoi `observe_small`).
- Ultima probă (întreruptă de utilizator): pick de pe r1c1 (uv 1,53; 1,43) cu gheara la 0 → **a ratat din nou** și a
  împins cubul ~1 cm spre robot; brațul a revenit în `observe_small`, cubul e încă pe r1c1 (ușor deplasat).

## Unelte (în `ai/scripts/calib_live/autocal/`)
- `mutare_foaie.py [--aplica | --ref-nou]`: foaia mutată → corecția caroiajului din cameră (home → observe, 16 intersecții
  comparate cu `ref/`). `--ref-nou` după fiecare calibrare cu mâna. Precizie ~2 mm, dar NU detectează schimbări ale brațului.
- `tur_celule.mjs`: gheara deschisă deasupra celulelor (verificare cu ochiul).
- `coborare_lenta.mjs U,V`: coborâre cu opriri și bipuri deasupra unui punct uv; echipa spune unde e gheara.
- `proba_contact_gheara.mjs`: gheara la 0 coboară pe r1c1 și strânge în pași; raportează unghiul de contact.
- `proba_pick_place.mjs uv:U,V RC`: pick/place cu gheara 0 / 103; se oprește dacă gheara e goală (servo 6 la comandă).
- `detect.py`, `tur_poze_camera.py`, `hand_eye_incercare.py` (calibrarea cameră-braț, nereușită: rezidual 7 px).

## Ce e de făcut (în ordine)
1. **Starea brațului și a cubului** (fără mișcare): citire, apoi `go_pose home` + `observe_small`, apoi ce vede camera.
2. **De ce ratează pick-ul pe r1c1 deși colțurile sunt învățate.** Indicii:
   - modelul cinematic „strânge” distanțele: între colțurile învățate pasul e 31 mm în model față de 44-45 mm pe foaie, iar
     unghiul dintre axe e 83,6° (fizic 90°). Grila e potrivită afin pe colțuri, deci centrul r1c1 e o interpolare, iar
     IK-ul alege altă înclinare a ghearei (`pitchFor`: prima posibilă de la −90°) decât cea de la învățare (−35…−55°);
     cu un model greșit, altă înclinare = alt loc fizic;
   - de verificat cu echipa, cu `coborare_lenta.mjs 1.5,1.5`, unde ajunge gheara (direcția și cm);
   - remedii posibile, de discutat cu utilizatorul: învățarea tuturor celor 9 celule
     (`npm run calib -- celule small --celule r0c0 r0c1 r0c2 r1c0 r1c1 r1c2 r2c0 r2c1 r2c2`) și folosirea punctului
     învățat (unghiurile lui) pentru celula respectivă; IK cu înclinarea de la învățare; sau calibrarea modelului
     (offset-urile de zero ale servo 2-4, eventual o scară a unghiurilor) din punctele învățate. Cauza compresiei nu e
     lămurită: cotele din carte nu o explică.
3. **Gheara pentru 40 mm**: măsurați cu șublerul deschiderea interioară la câteva unghiuri (`npm run calib -- gheara 0`,
   `... gheara 30`, `60`, `95`) → tabelul din `calib.json` (atunci brain-ul poate cere `grip_mm` / `release_mm`). Stabiliți
   cu utilizatorul `GRIP_OPEN` / `GRIP_CLOSED` pentru 40 mm (propunere: deschis 0–10, strâns ~103).
4. **Pick/place la nivelul 0** pe mai multe celule, cu verificare din cameră după fiecare mutare
   (`proba_pick_place.mjs`, apoi serviciul HTTP `pick` / `place`).
5. **Stivuirea** (nivelul 1+, +40 mm pe nivel): data trecută cubul a ajuns ~1 cm alături. Testează după punctul 2.
6. **Testele brain-ului pentru 40 mm** (ramura `cub-40mm`): 21 picate. Cu cuburi de 40 mm pe celule de 43-45 mm rămân doar
   3-5 mm între cuburi vecine, iar degetele au 6 mm: între doi vecini pe direcția degetelor gheara nu încape. Regulile
   din `brain/domain/grasp.py` și scenele sintetice trebuie revăzute (sau aflat de la echipă pe ce foaie se folosesc cuburile
   de 40 mm). Abia apoi `cub-40mm` → `brain-etapa2` → `main`.
7. **Cap-coadă**: `brain run` (S1/S2/S3) pe foaia `small`, cu previzualizarea pornită.
