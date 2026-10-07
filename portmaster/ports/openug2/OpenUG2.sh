#!/bin/bash
XDG_DATA_HOME=${XDG_DATA_HOME:-$HOME/.local/share}

if [ -d "/opt/system/Tools/PortMaster/" ]; then
  controlfolder="/opt/system/Tools/PortMaster"
elif [ -d "/opt/tools/PortMaster/" ]; then
  controlfolder="/opt/tools/PortMaster"
elif [ -d "$XDG_DATA_HOME/PortMaster/" ]; then
  controlfolder="$XDG_DATA_HOME/PortMaster"
else
  controlfolder="/roms/ports/PortMaster"
fi

source "$controlfolder/control.txt"
[ -f "${controlfolder}/mod_${CFW_NAME}.txt" ] && source "${controlfolder}/mod_${CFW_NAME}.txt"

get_controls

GAMEDIR="/$directory/ports/openug2"
CONFDIR="$GAMEDIR/conf"
mkdir -p "$CONFDIR"
cd "$GAMEDIR" || exit 1

> "$GAMEDIR/log.txt" && exec > >(tee "$GAMEDIR/log.txt") 2>&1

export XDG_DATA_HOME="$CONFDIR"
export LD_LIBRARY_PATH="$GAMEDIR/libs.${DEVICE_ARCH}:$LD_LIBRARY_PATH"
export SDL_GAMECONTROLLERCONFIG="$sdl_controllerconfig"

DATA="$GAMEDIR/data"
if [ ! -d "$DATA/TRACKS" ]; then
  pm_message "OpenUG2 data missing. See the README to build openug2/data with tools/import_nfsu2_data.py."
  sleep 5
  exit 1
fi

BIN="nfsu2.${DEVICE_ARCH}"
if [ ! -x "$GAMEDIR/$BIN" ]; then
  pm_message "OpenUG2 binary $BIN missing."
  sleep 5
  exit 1
fi

$GPTOKEYB "$BIN" &
pm_platform_helper "$GAMEDIR/$BIN"
"$GAMEDIR/$BIN" "$DATA"
pm_finish
