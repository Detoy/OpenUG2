## Notes

OpenUG2 is an open-source, from-scratch reimplementation of the Need for Speed:
Underground 2 engine. It reads your own copy of the game's data files directly
and runs a native racing scene. This port ships the engine only. It contains no
game assets and no Electronic Arts code.

Thanks to the OpenUG2 project and its maintainer at
[github.com/whoismept/OpenUG2](https://github.com/whoismept/OpenUG2) for the
engine this port is built from.

## Status

Early prototype. The aarch64 build loads and renders the L4RA world on the
R36S. Controller gameplay, the menu, and career progression are not implemented
yet, so this port is experimental.

## What you supply

A legally acquired copy of Need for Speed Underground 2. The port reads it from
`openug2/data`. Prepare that directory with the importer, which never writes to
your installation:

```sh
python3 tools/import_nfsu2_data.py --source "/path/to/Need for Speed Underground 2" --output /roms/ports/openug2/openug2/data
```

## Controls

Keyboard only at this stage. Gamepad control arrives with the first handheld
race milestone.

## Compile

```sh
make
make gles-aarch64 TARGET_PREFIX=/path/to/aarch64-prefix
cp nfsu2-gles-aarch64 /roms/ports/openug2/openug2/nfsu2.aarch64
```

`TARGET_PREFIX` holds the target's SDL2, GLES2, and zlib dev files. See
`docs/INSTALL_R36S.md` for how to assemble it.
