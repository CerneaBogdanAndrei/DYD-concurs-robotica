#!/bin/bash
# Pornește `node dist/src/cli/calib.js <args>` cu intrarea dintr-un FIFO, ca Enter-ul să poată fi trimis din altă
# consolă (`echo > ai/scripts/calib_live/in`), iar ieșirea în log.txt. Folosit la calibrarea din 2026-10-29.
D="$(cd "$(dirname "$0")" && pwd)"
rm -f "$D/in"; mkfifo "$D/in"
sleep 100000 > "$D/in" &            # ține FIFO-ul deschis între Enter-uri
echo $! > "$D/holder.pid"
cd "$D/../../.."
node dist/src/cli/calib.js "$@" < "$D/in" > "$D/log.txt" 2>&1 &
echo $! > "$D/calib.pid"
