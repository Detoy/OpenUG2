# Install OpenUG2 on the R36S

This guide builds the engine and puts it on the handheld. The engine ships no
game data, so each user supplies their own copy of Need for Speed Underground 2.

## What you need

- A working desktop build to confirm your data copy loads (optional but fast).
- An aarch64 cross toolchain and a target prefix that holds the target's SDL2,
  OpenGL ES 2.0, and zlib headers and libraries. A bare `aarch64-linux-gnu-gcc`
  is not enough: it has glibc but no target SDL2, no GLES2 headers, and no
  target libraries.
- An owned NFSU2 installation to read.

## Build the engine

Build the desktop binary first and run the automated tests.

```sh
make
make import-test
make world-instance-test car-material-test world-render-test world-cli-test
```

Cross-build for the handheld with `aarch64-linux-gnu-gcc` and a target prefix
that holds the target's SDL2, GLES2, and zlib headers and libraries. The
compiler's own sysroot supplies libc. On Debian, take those dev and runtime
packages for the target architecture (`libsdl2-dev`, `libsdl2-2.0-0`,
`libgles-dev`, `libgles2`, `libegl-dev`, `libegl1`, `libglvnd-dev`,
`libglvnd-core-dev`, `libglvnd0`, `libgl-dev`, `zlib1g-dev`, `zlib1g`) and
unpack them into one prefix.

```sh
make gles-aarch64 TARGET_PREFIX=/path/to/aarch64-prefix
file nfsu2-gles-aarch64
```

The build fails on purpose when `TARGET_PREFIX` is unset or has no SDL2 headers.
The host `sdl2-config` cannot cross-compile, so swapping the compiler alone does
not work. `file` must report `ELF 64-bit LSB executable, ARM aarch64`.

## Prepare the data directory

Run the importer. It reads the installation and writes a separate working copy.
It never writes to the source, and it refuses any output that resolves inside
the source.

```sh
python3 tools/import_nfsu2_data.py \
  --source "/path/to/Need for Speed Underground 2" \
  --output /path/to/prepared-data
```

To preview the copy without writing, add `--dry-run`. To overwrite files in an
existing output, add `--force`.

## Install and run on the SD card

PortMaster launches a port from a shell script in `/roms/ports/`, with the
port's own files in a subdirectory. The card mounts at `/roms` in the single-SD
layout and at `/roms2` in the two-SD layout.

1. Copy the binary to the port directory.

   ```sh
   mkdir -p /roms/ports/nfsu2
   cp nfsu2-gles-aarch64 /roms/ports/nfsu2/
   ```

2. Copy the prepared data.

   ```sh
   cp -r /path/to/prepared-data /roms/nfsu2/data
   ```

3. Copy the run launcher into `/roms/ports/`.

   ```sh
   cp tools/nfsu2_device_run.sh /roms/ports/NFSU2.sh
   ```

4. On the handheld, open **PORTS** and run **NFSU2**. The launcher runs a
   no-argument load test, the deterministic instance audit twice, and a
   one-frame render capture. It writes `/roms/ports/nfsu2/nfsu2-run.log`.

5. Shut the device down, move the card back to the computer, and read the log.

   ```sh
   cat /roms/ports/nfsu2/nfsu2-run.log
   ```

## Package for PortMaster

`portmaster/ports/openug2/` holds a PortMaster package skeleton that ships the
engine only. It has `port.json`, `gameinfo.xml`, the `OpenUG2.sh` launcher, a
README, and the MIT license. Copy the aarch64 binary in as `openug2/nfsu2.aarch64`
and have each user prepare `openug2/data` with the importer. A gameplay
screenshot and testing across CFWs are still pending.

## Collect device diagnostics

Copy the probe collector and its launcher to the card.

```sh
mkdir -p /roms/ports/nfsu2-probe
cp tools/r36s_probe.sh /roms/ports/nfsu2-probe/
cp "tools/r36s_probe_launcher.sh" "/roms/ports/NFSU2 Device Probe.sh"
```

On the handheld, open **PORTS** and run **NFSU2 Device Probe**. It collects the
firmware, CPU, memory, storage, graphics, controller, and audio data into
`/roms/ports/nfsu2-probe/probe.log`, then returns to the menu. If the
openrockport capability binary is present, the probe also runs it to capture
the GL and EGL strings.

Shut the device down, move the card back to the computer, and read the log.

```sh
cat /roms/ports/nfsu2-probe/probe.log
```

Frame timing needs the installed engine. Add the engine's FPS and frame-time
output from `nfsu2-run.log` after the first run.

## Known gaps

The aarch64 binary builds and links, but no R36S has run the engine yet. The
desktop build does not substitute for a device measurement. The load, render,
and frame-timing gates stay open until the run above lands and its log is
recorded.
