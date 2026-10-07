#!/bin/bash
# NFSU2 Device Probe launcher.
#
# Runs the OpenUG2 Foundation probe on the R36S and writes probe.log into the
# nfsu2-probe folder next to this script. The probe is read-only and needs no
# GL context, so it is safe to run without the PortMaster runtime.
#
# On the device: PORTS menu -> NFSU2 Device Probe. When it returns to the menu,
# eject the card and read <SD>/roms/ports/nfsu2-probe/probe.log.

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROBE_DIR="$SCRIPT_DIR/nfsu2-probe"
mkdir -p "$PROBE_DIR"
LOG="$PROBE_DIR/probe.log"
: > "$LOG"
exec > "$LOG" 2>&1

echo "=== NFSU2 Device Probe launcher: $(date -Is 2>/dev/null || date) ==="
echo "script dir: $SCRIPT_DIR"
echo "probe dir: $PROBE_DIR"

# Best effort: the PortMaster runtime sets LD_LIBRARY_PATH and SDL variables
# the optional GL capability binary needs. Missing runtime is not fatal.
for cf in /opt/system/Tools/PortMaster /opt/tools/PortMaster "$HOME/.local/share/PortMaster" /roms/ports/PortMaster; do
  if [ -f "$cf/control.txt" ]; then
    echo "sourcing $cf/control.txt"
    # shellcheck disable=SC1090
    . "$cf/control.txt"
    break
  fi
done

if command -v timeout >/dev/null 2>&1; then
  timeout 180 bash "$PROBE_DIR/r36s_probe.sh"
  RC=$?
else
  bash "$PROBE_DIR/r36s_probe.sh"
  RC=$?
fi
echo "probe exit: $RC (124 = timeout)"
sync
exit 0
