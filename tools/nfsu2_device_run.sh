#!/bin/bash
# NFSU2 device run launcher.
#
# Runs the OpenUG2 aarch64 GLES binary against the data copy on the SD card and
# writes logs next to itself. It performs a library and load check, the
# deterministic GL-free instance audit twice, then three render captures while
# sampling process RSS and system memory:
#   still3    a still scene, captured after 3 frames (does the engine render?)
#   still120  a longer still scene for frame timings
#   drive40   an auto-driven scene for a moving sample
# A capture writes its own PNG, perf CSV, console log, and memory log. The
# 640x480 size is required: the KMSDRM drawable is the panel mode.
#
# On the device: PORTS menu -> NFSU2. Eject the card and read
# <SD>/roms/ports/nfsu2/nfsu2-run.log and the per-capture logs.

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PORT_DIR="$SCRIPT_DIR/nfsu2"
BIN="$PORT_DIR/nfsu2-gles-aarch64"
LOG="$PORT_DIR/nfsu2-run.log"
mkdir -p "$PORT_DIR"
: > "$LOG"
exec > "$LOG" 2>&1

printf '=== NFSU2 run launcher: %s ===\n' "$(date -Is 2>/dev/null || date)"
echo "port dir: $PORT_DIR"

DATA=""
for d in /roms/nfsu2/data /roms2/nfsu2/data "$SCRIPT_DIR/../nfsu2/data"; do
  [ -d "$d/TRACKS" ] && { DATA="$d"; break; }
done
echo "data dir: ${DATA:-NOT FOUND}"

for cf in /opt/system/Tools/PortMaster /opt/tools/PortMaster "$HOME/.local/share/PortMaster" /roms/ports/PortMaster; do
  if [ -f "$cf/control.txt" ]; then
    echo "sourcing $cf/control.txt"
    # shellcheck disable=SC1090
    . "$cf/control.txt"
    break
  fi
done
[ -n "${sdl_controllerconfig:-}" ] && export SDL_GAMECONTROLLERCONFIG="$sdl_controllerconfig"

MALI_BLOB=""
for c in /usr/lib/aarch64-linux-gnu/libmali-bifrost-g31-rxp0-gbm.so \
         /lib/aarch64-linux-gnu/libmali-bifrost-g31-rxp0-gbm.so \
         /usr/lib/aarch64-linux-gnu/libmali.so.1 /lib/aarch64-linux-gnu/libmali.so.1; do
  [ -e "$c" ] && { MALI_BLOB="$c"; break; }
done
if [ -n "$MALI_BLOB" ]; then
  GL_SHIM=/tmp/openug2-gl
  rm -rf "$GL_SHIM"
  if mkdir -p "$GL_SHIM" && ln -sf "$MALI_BLOB" "$GL_SHIM/libEGL.so.1" \
                          && ln -sf "$MALI_BLOB" "$GL_SHIM/libGLESv2.so.2"; then
    export LD_LIBRARY_PATH="$GL_SHIM:$LD_LIBRARY_PATH"
    echo "GL: forcing Mali blob from $MALI_BLOB"
  fi
fi

if [ ! -x "$BIN" ]; then
  echo "ERROR: $BIN missing"
  exit 1
fi

mem_avail() { awk '/MemAvailable/{print $2}' /proc/meminfo 2>/dev/null; }

echo
echo "=== dynamic dependencies ==="
ldd "$BIN" 2>&1 | grep -i 'not found' || echo "all libraries resolved"

echo
echo "=== load test: no arguments ==="
timeout 20 "$BIN"
echo "load test exit: $?"

if [ -z "$DATA" ]; then
  echo "ERROR: game data not found; skipping engine runs"
  exit 0
fi

echo
echo "=== instance audit 1 ==="
timeout 300 "$BIN" "$DATA" --track STREAML4RA --instance-audit > "$PORT_DIR/audit1.log" 2>&1
echo "audit1 exit: $?"
echo "=== instance audit 2 ==="
timeout 300 "$BIN" "$DATA" --track STREAML4RA --instance-audit > "$PORT_DIR/audit2.log" 2>&1
echo "audit2 exit: $?"
if cmp -s "$PORT_DIR/audit1.log" "$PORT_DIR/audit2.log"; then
  echo "audit determinism: IDENTICAL"
else
  echo "audit determinism: DIFFER"
  diff "$PORT_DIR/audit1.log" "$PORT_DIR/audit2.log" | head -n 40
fi
tail -n 6 "$PORT_DIR/audit1.log"

if command -v pm_platform_helper >/dev/null 2>&1; then
  pm_platform_helper "$BIN"
fi

mem_run() {
  label="$1"; shift
  echo
  echo "=== capture: $label ($*) ==="
  echo "MemAvailable before: $(mem_avail) kB"
  rm -f "$PORT_DIR/$label.png" "$PORT_DIR/$label-perf.csv"
  "$BIN" "$DATA" --car MIATA --track STREAML4RA --resolution 640x480 \
    --shot "$PORT_DIR/$label.png" --perf-csv "$PORT_DIR/$label-perf.csv" \
    "$@" > "$PORT_DIR/$label-console.log" 2>&1 &
  pid=$!
  peak=0
  : > "$PORT_DIR/$label-mem.log"
  while kill -0 "$pid" 2>/dev/null; do
    rss=$(awk '/VmRSS/{print $2}' "/proc/$pid/status" 2>/dev/null)
    if [ -n "$rss" ] && [ "$rss" -gt "$peak" ] 2>/dev/null; then peak=$rss; fi
    printf '%s rss_kB=%s avail_kB=%s\n' "$(date +%T)" "${rss:-?}" "$(mem_avail)" >> "$PORT_DIR/$label-mem.log"
    sleep 0.5
  done
  wait "$pid"; rc=$?
  echo "exit: $rc (137 = killed, 124 = timeout)"
  echo "peak process RSS: ${peak} kB"
  if [ -s "$PORT_DIR/$label.png" ]; then
    echo "shot written: $label.png $(wc -c < "$PORT_DIR/$label.png") bytes"
  else
    echo "no shot"
  fi
  if [ -s "$PORT_DIR/$label-perf.csv" ]; then
    echo "perf rows: $(wc -l < "$PORT_DIR/$label-perf.csv")"
    awk -F, 'NR==1{for(i=1;i<=NF;i++)if($i=="cpu_total")c=i;next}
             c && $c+0>0{s+=$c;n++;if($c>mx)mx=$c}
             END{if(n)printf "cpu_total mean %.1f ms (%.1f fps), max %.1f ms over %d frames\n",s/n,1000/(s/n),mx,n; else print "no cpu_total samples"}' \
      "$PORT_DIR/$label-perf.csv"
  else
    echo "no perf csv"
  fi
}

echo
echo "system MemAvailable before captures: $(mem_avail) kB"
mem_run still3 --perf-still --frames 3
mem_run still120 --perf-still --frames 120
mem_run drive40 --frames 40

if command -v pm_finish >/dev/null 2>&1; then
  pm_finish
fi
sync
echo "=== run complete ==="
exit 0
