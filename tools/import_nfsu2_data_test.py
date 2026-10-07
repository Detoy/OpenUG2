#!/usr/bin/env python3
"""Synthetic-fixture tests for tools/import_nfsu2_data.py. No retail data needed."""

import hashlib
import os
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
TOOL = os.path.join(HERE, "import_nfsu2_data.py")

FAILURES = []


def check(name, ok, detail=""):
    print("  %-58s %s" % (name, "PASS" if ok else "FAIL"))
    if not ok:
        FAILURES.append("%s %s" % (name, detail))


def run(*args):
    p = subprocess.run([sys.executable, TOOL] + list(args),
                       capture_output=True, text=True)
    return p.returncode, p.stdout + p.stderr


def make_install(root, symlink_file=False):
    os.makedirs(os.path.join(root, "TRACKS/ROUTESL4RA"))
    os.makedirs(os.path.join(root, "CARS/MIATA"))
    os.makedirs(os.path.join(root, "GLOBAL"))
    os.makedirs(os.path.join(root, "FRONTEND"))
    with open(os.path.join(root, "TRACKS/STREAML4RA.BUN"), "wb") as f:
        f.write(b"stream-bytes")
    with open(os.path.join(root, "TRACKS/ROUTESL4RA/Paths4175.bin"), "wb") as f:
        f.write(b"routes")
    with open(os.path.join(root, "CARS/MIATA/GEOMETRY.BIN"), "wb") as f:
        f.write(b"geometry")
    with open(os.path.join(root, "GLOBAL/InGameCommon.bun"), "wb") as f:
        f.write(b"common")
    with open(os.path.join(root, "FRONTEND/menu.bin"), "wb") as f:
        f.write(b"frontend")
    with open(os.path.join(root, "SPEED2.EXE"), "wb") as f:
        f.write(b"exe-not-copied")
    if symlink_file:
        os.symlink("/etc/hosts", os.path.join(root, "TRACKS/evil.bin"))


def tree_state(root):
    state = {}
    for dirpath, _dirs, files in os.walk(root):
        for f in files:
            p = os.path.join(dirpath, f)
            if os.path.islink(p):
                state[p] = "symlink"
                continue
            with open(p, "rb") as fh:
                state[p] = hashlib.sha256(fh.read()).hexdigest()
    return state


def main():
    tmp = tempfile.mkdtemp(prefix="openug2-import-test.")
    try:
        src = os.path.join(tmp, "install")
        make_install(src)
        before = tree_state(src)

        out = os.path.join(tmp, "prepared-data")
        code, log = run("--source", src, "--output", out)
        check("copies a valid installation", code == 0, log)
        check("TRACKS copied", os.path.isfile(os.path.join(out, "TRACKS/STREAML4RA.BUN")))
        check("nested route copied", os.path.isfile(os.path.join(out, "TRACKS/ROUTESL4RA/Paths4175.bin")))
        check("CARS copied", os.path.isfile(os.path.join(out, "CARS/MIATA/GEOMETRY.BIN")))
        check("GLOBAL copied", os.path.isfile(os.path.join(out, "GLOBAL/InGameCommon.bun")))
        check("optional FRONTEND copied", os.path.isfile(os.path.join(out, "FRONTEND/menu.bin")))
        check("SPEED2.EXE not copied", not os.path.exists(os.path.join(out, "SPEED2.EXE")))
        check("source unchanged", tree_state(src) == before)

        code, log = run("--source", src, "--output", out)
        check("re-run without --force refuses", code == 4, log)
        code, log = run("--source", src, "--output", out, "--force")
        check("re-run with --force succeeds", code == 0, log)

        code, log = run("--source", src, "--output", os.path.join(src, "TRACKS/out"))
        check("output inside source refused", code == 4, log)
        check("no output written into source", not os.path.exists(os.path.join(src, "TRACKS/out")))

        code, log = run("--source", src, "--output", src)
        check("output equal to source refused", code == 4, log)

        link_out = os.path.join(tmp, "link-out")
        os.symlink(src, link_out)
        code, log = run("--source", src, "--output", link_out)
        check("symlinked output into source refused", code == 4, log)

        bad = os.path.join(tmp, "bad")
        os.makedirs(os.path.join(bad, "TRACKS"))
        code, log = run("--source", bad, "--output", os.path.join(tmp, "out2"))
        check("missing required dirs rejected", code == 3, log)

        dry = os.path.join(tmp, "dry")
        code, log = run("--source", src, "--output", dry, "--dry-run")
        check("dry run exits 0", code == 0, log)
        check("dry run writes nothing", not os.path.exists(dry))

        sym = os.path.join(tmp, "syminstall")
        make_install(sym, symlink_file=True)
        code, log = run("--source", sym, "--output", os.path.join(tmp, "out3"))
        check("symlinked source file refused", code == 1, log)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    if FAILURES:
        print("import_test: FAIL (%d)" % len(FAILURES))
        for f in FAILURES:
            print("  " + f)
        return 1
    print("import_test: PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
