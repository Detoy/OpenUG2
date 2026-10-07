#!/usr/bin/env python3
"""Validate a retail NFSU2 installation and prepare a portable data directory.

Reads the source installation without modifying it and copies the engine's data
directories into --output. Refuses any output that resolves into the source, so
an owned installation can never be written to by accident.
"""

import argparse
import os
import shutil
import sys

EXIT_OK = 0
EXIT_IO = 1
EXIT_USAGE = 2
EXIT_SOURCE = 3
EXIT_OUTPUT = 4

# (directory, required) copied in this order. Required directories must exist
# in a valid installation; the rest are copied when present.
DATA_DIRS = (
    ("TRACKS", True),
    ("CARS", True),
    ("GLOBAL", True),
    ("FRONTEND", False),
    ("SOUND", False),
    ("MOVIES", False),
    ("NIS", False),
    ("SUBTITLES", False),
    ("LANGUAGES", False),
    ("CREDITS", False),
    ("SCRIPTS", False),
)


def resolve(path):
    return os.path.realpath(os.path.abspath(path))


def contains(parent, child):
    parent = parent.rstrip(os.sep)
    return child == parent or child.startswith(parent + os.sep)


def validate_source(source):
    errors = []
    missing = []
    for name, required in DATA_DIRS:
        path = os.path.join(source, name)
        if os.path.isdir(path):
            continue
        if required:
            errors.append("required directory missing: %s/" % name)
        else:
            missing.append(name)
    if not os.path.isdir(source):
        errors.append("source is not a directory")
    return errors, missing


def check_output(source_real, output_real):
    if contains(source_real, output_real):
        return "--output resolves inside --source; refusing to write to the installation"
    if contains(output_real, source_real):
        return "--output is a parent of --source; refusing"
    return None


def copy_dirs(source, output, force, dry_run):
    copied = 0
    bytes_total = 0
    for name, _ in DATA_DIRS:
        src = os.path.join(source, name)
        if not os.path.isdir(src):
            continue
        dst = os.path.join(output, name)
        if dry_run:
            print("would copy %s/ -> %s/" % (name, dst))
            continue
        if os.path.exists(dst):
            if not force:
                raise FileExistsError(dst)
        else:
            os.makedirs(dst, exist_ok=True)
        for root, _dirs, files in os.walk(src):
            rel = os.path.relpath(root, src)
            target_root = dst if rel == "." else os.path.join(dst, rel)
            os.makedirs(target_root, exist_ok=True)
            for f in files:
                s = os.path.join(root, f)
                if os.path.islink(s):
                    raise ValueError("refusing to copy symlink: %s" % s)
                t = os.path.join(target_root, f)
                shutil.copy2(s, t)
                copied += 1
                bytes_total += os.path.getsize(s)
        print("copied %s/" % name)
    return copied, bytes_total


def main(argv):
    ap = argparse.ArgumentParser(
        description="Prepare a portable OpenUG2 data directory from an owned NFSU2 installation."
    )
    ap.add_argument("--source", required=True, help="retail installation to read (never modified)")
    ap.add_argument("--output", required=True, help="portable data directory to create")
    ap.add_argument("--force", action="store_true", help="overwrite files in an existing output")
    ap.add_argument("--dry-run", action="store_true", help="report the plan without writing")
    args = ap.parse_args(argv)

    source_real = resolve(args.source)
    output_real = resolve(args.output)

    if not os.path.isdir(source_real):
        print("error: --source is not a directory: %s" % args.source, file=sys.stderr)
        return EXIT_SOURCE
    if output_real == source_real:
        print("error: --output must differ from --source", file=sys.stderr)
        return EXIT_OUTPUT
    unsafe = check_output(source_real, output_real)
    if unsafe:
        print("error: %s" % unsafe, file=sys.stderr)
        return EXIT_OUTPUT
    if os.path.exists(output_real) and os.path.islink(args.output):
        print("error: --output is a symlink; refusing", file=sys.stderr)
        return EXIT_OUTPUT

    errors, optional_missing = validate_source(source_real)
    if errors:
        for e in errors:
            print("error: %s" % e, file=sys.stderr)
        print("not a usable NFSU2 data root: %s" % source_real, file=sys.stderr)
        return EXIT_SOURCE
    for name in optional_missing:
        print("warning: optional directory absent: %s/" % name, file=sys.stderr)

    if not args.dry_run:
        try:
            os.makedirs(output_real, exist_ok=True)
        except OSError as e:
            print("error: cannot create --output: %s" % e, file=sys.stderr)
            return EXIT_IO

    try:
        copied, bytes_total = copy_dirs(source_real, output_real, args.force, args.dry_run)
    except FileExistsError as e:
        print("error: output already contains %s; pass --force to overwrite" % e, file=sys.stderr)
        return EXIT_OUTPUT
    except (OSError, ValueError) as e:
        print("error: copy failed: %s" % e, file=sys.stderr)
        return EXIT_IO

    if args.dry_run:
        print("dry run: no files written")
        return EXIT_OK

    for name, required in DATA_DIRS:
        if required and not os.path.isdir(os.path.join(output_real, name)):
            print("error: post-copy check failed: %s/ missing" % name, file=sys.stderr)
            return EXIT_IO
    print("ready: %d files, %.2f MiB in %s" % (copied, bytes_total / 1048576.0, output_real))
    return EXIT_OK


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
