#!/bin/bash
# OpenUG2 R36S Foundation probe. Collects the device evidence the Foundation
# milestone needs: firmware, CPU, memory, storage, graphics, controller, and
# audio. Writes one plain-text log. Runs on the handheld; reads only.
#
#   ./r36s_probe.sh [LOGFILE]
#
# Frame timing is not collected here. It needs the OpenUG2 engine installed,
# which is a later step.

set -u

LOG="${1:-}"
if [ -n "$LOG" ]; then exec > "$LOG" 2>&1; fi

have() { command -v "$1" >/dev/null 2>&1; }
section() { printf '\n===== %s =====\n' "$1"; }

printf '=== OpenUG2 R36S Foundation probe ===\n'
printf 'run at: %s\n' "$(date -Is 2>/dev/null || date)"

section "identity"
have uname && uname -a
printf 'machine: %s\n' "$(uname -m)"
printf 'word size: %s\n' "$(getconf LONG_BIT 2>/dev/null || echo unknown)"
printf 'kernel: %s\n' "$(uname -r)"
printf 'hostname: %s\n' "$(hostname 2>/dev/null || echo unknown)"
[ -r /proc/device-tree/model ] && printf 'device-tree model: %s\n' "$(tr -d '\0' < /proc/device-tree/model)"
[ -r /proc/cmdline ] && printf 'kernel cmdline: %s\n' "$(cat /proc/cmdline)"

section "distribution and libc architecture"
printf 'dpkg architecture: %s\n' "$(dpkg --print-architecture 2>/dev/null || echo 'dpkg absent')"
printf 'libc triplet dirs:\n'; ls -d /usr/lib/*-linux-gnu* /lib/*-linux-gnu* 2>/dev/null || echo '  none'
printf 'ldd: %s\n' "$(ldd --version 2>/dev/null | head -n1 || echo unknown)"
have arch && printf 'arch: %s\n' "$(arch)"

section "firmware / CFW"
for f in /etc/os-release /etc/arkos* /etc/dArkOS* /etc/r36_config.ini /etc/debian_version /etc/version; do
  [ -r "$f" ] && { printf -- '--- %s ---\n' "$f"; cat "$f" 2>/dev/null; }
done

section "cpu"
grep -m1 -E 'model name|Hardware|Processor|CPU part' /proc/cpuinfo 2>/dev/null || cat /proc/cpuinfo 2>/dev/null | head -n 30
printf 'cores: %s\n' "$(nproc 2>/dev/null || echo unknown)"
for g in /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor /sys/devices/system/cpu/cpu0/cpufreq/cpuinfo_max_freq; do
  [ -r "$g" ] && printf '%s: %s\n' "$g" "$(cat "$g")"
done

section "memory"
grep -E 'MemTotal|MemFree|MemAvailable|SwapTotal|SwapFree|Cached|Buffers' /proc/meminfo 2>/dev/null || echo 'meminfo unreadable'
have free && free -m
[ -r /proc/swaps ] && { printf -- '--- /proc/swaps ---\n'; cat /proc/swaps; }
have zramctl && zramctl 2>/dev/null

section "storage and mounts"
have df && df -h
[ -r /proc/mounts ] && grep -E '/roms|/boot|root' /proc/mounts

section "graphics"
printf -- '--- /dev/dri ---\n'; ls -l /dev/dri 2>/dev/null || echo none
printf -- '--- drm uevent ---\n'; cat /sys/class/drm/card*/device/uevent 2>/dev/null || echo none
printf -- '--- gpu devfreq ---\n'; grep -r . /sys/class/devfreq/*/cur_freq 2>/dev/null || echo none
printf -- '--- dmesg gpu ---\n'; dmesg 2>/dev/null | grep -iE 'mali|panfrost|drm|rk3|gpu|bifrost' | tail -n 40 || echo 'dmesg unreadable (needs root)'

section "display"
printf -- '--- fb modes ---\n'; cat /sys/class/graphics/fb0/modes 2>/dev/null || echo none
printf -- '--- drm modes ---\n'; for m in /sys/class/drm/card*/card*/modes; do [ -r "$m" ] && { printf '%s: ' "$m"; head -n1 "$m"; }; done 2>/dev/null || echo none

section "input / controller"
printf -- '--- /proc/bus/input/devices ---\n'; cat /proc/bus/input/devices 2>/dev/null || echo unreadable
printf -- '--- evdev names ---\n'; for f in /sys/class/input/event*/device/name; do printf '%s: ' "$f"; cat "$f" 2>/dev/null; done
printf -- '--- js devices ---\n'; ls -l /dev/input/js* /dev/input/by-id 2>/dev/null || echo none
printf 'SDL_GAMECONTROLLERCONFIG: %s\n' "${SDL_GAMECONTROLLERCONFIG:-unset}"

section "audio"
cat /proc/asound/cards 2>/dev/null || echo unreadable

section "libraries"
printf 'sdl2-config: %s\n' "$(sdl2-config --version 2>/dev/null || echo absent)"
ldconfig -p 2>/dev/null | grep -Ei 'libSDL2|libEGL|libGLESv2|libmali|libMali|libgbm' || echo 'none listed'

section "gl capability binary"
CAP=""
for c in "$(dirname "$0")/device_probe.aarch64" /roms/ports/device_probe.aarch64 /roms2/ports/device_probe.aarch64; do
  [ -x "$c" ] && { CAP="$c"; break; }
done
if [ -n "$CAP" ]; then
  printf 'running %s (timeout 45s)\n' "$CAP"
  if have timeout; then timeout 45 "$CAP"; else "$CAP"; fi
  printf 'capability binary exit: %s\n' "$?"
else
  echo 'no capability binary found; skipping GL/EGL/GLES string capture'
fi

printf '\n=== probe complete ===\n'
