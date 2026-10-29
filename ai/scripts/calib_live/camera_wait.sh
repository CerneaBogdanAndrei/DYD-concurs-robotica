#!/bin/bash
# Previzualizarea camerei, repornită singură: așteaptă camera USB (UVC), pornește pe indexul ei și o oprește
# când dispare dispozitivul (cablul scos), ca la reconectare să pornească pe noul index.
cd /home/admin/DYD-concurs-robotica/ai
while true; do
  n=""
  for d in /sys/class/video4linux/video*; do
    [ -e "$d/name" ] && grep -qi "usb" "$d/name" && [ "$(cat $d/index 2>/dev/null)" = "0" ] && { n=${d##*video}; break; }
  done
  if [ -z "$n" ]; then sleep 1; continue; fi
  echo "$(date +%T) camera pe /dev/video$n"
  .venv/bin/python -m brain camera preview --index "$n" &
  pid=$!
  while kill -0 $pid 2>/dev/null && [ -e /dev/video$n ]; do sleep 2; done
  echo "$(date +%T) camera /dev/video$n a dispărut sau previzualizarea s-a oprit"
  pkill -P $pid 2>/dev/null; kill $pid 2>/dev/null; wait $pid 2>/dev/null
  sleep 1
done
