#!/usr/bin/env python3
"""Install the pinned, patched OMPL dependency without importing its Git history."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import tarfile
import tempfile
import urllib.request

ROOT = Path(__file__).resolve().parents[1]
THIRD_PARTY = ROOT / "third_party"
DEST = THIRD_PARTY / "ompl"
PATCH = THIRD_PARTY / "ompl-local.patch"
LOCK_FILE = THIRD_PARTY / "ompl.lock.json"
MARKER = ".bacon-source.json"


def digest(path):
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def check(lock):
    marker = DEST / MARKER
    if not marker.is_file():
        raise RuntimeError("OMPL is not prepared; run python3 scripts/setup_ompl.py")
    saved = json.loads(marker.read_text())
    if saved.get("lock") != lock:
        raise RuntimeError("OMPL version or patch changed; move third_party/ompl aside and run setup again")
    for name, sha in saved["patched_files"].items():
        path = DEST / name
        if not path.is_file() or digest(path) != sha:
            raise RuntimeError("Prepared OMPL file changed: " + name)
    print("OMPL ready: " + lock["revision"])


def extract(archive, output):
    # The GitHub archive has one top-level directory. Reject paths and links
    # that could escape the private extraction directory (also on Python 3.8).
    with tarfile.open(archive, "r:gz") as tar:
        members = tar.getmembers()
        roots = set()
        for member in members:
            path = Path(member.name)
            if path.is_absolute() or ".." in path.parts or not path.parts:
                raise RuntimeError("Unsafe archive path: " + member.name)
            if not (member.isfile() or member.isdir()):
                raise RuntimeError("Unsupported archive entry: " + member.name)
            roots.add(path.parts[0])
        if len(roots) != 1:
            raise RuntimeError("Expected one top-level archive directory")
        tar.extractall(output, members=members)
        return output / roots.pop()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="Check version and patched files; never download")
    parser.add_argument("--archive", type=Path, help="Use an already downloaded archive (same SHA-256 required)")
    args = parser.parse_args()
    lock = json.loads(LOCK_FILE.read_text())
    if digest(PATCH) != lock["patch_sha256"]:
        raise RuntimeError("OMPL patch does not match the lock file")
    if args.check or DEST.exists():
        check(lock)
        return
    with tempfile.TemporaryDirectory(prefix="ompl-setup-", dir=THIRD_PARTY) as tmp:
        tmp = Path(tmp)
        archive = args.archive
        if archive is None:
            archive = tmp / "ompl.tar.gz"
            print("Downloading " + lock["archive_url"], flush=True)
            with urllib.request.urlopen(lock["archive_url"], timeout=60) as response, archive.open("wb") as out:
                shutil.copyfileobj(response, out)
        if digest(archive) != lock["archive_sha256"]:
            raise RuntimeError("OMPL archive SHA-256 mismatch; no dependency was installed")
        tree = extract(archive, tmp)
        # Disable inherited Git context: this applies to the downloaded tree.
        import os
        env = {k: v for k, v in os.environ.items() if not k.startswith("GIT_")}
        env["GIT_CEILING_DIRECTORIES"] = str(tree.parent.resolve())
        for extra in (["--check"], []):
            subprocess.run(["git", "apply", *extra, str(PATCH)], cwd=tree, env=env, check=True)
        patched = [line[6:] for line in PATCH.read_text().splitlines() if line.startswith("+++ b/")]
        saved = {"lock": lock, "patched_files": {name: digest(tree / name) for name in patched}}
        (tree / MARKER).write_text(json.dumps(saved, indent=2) + "\n")
        tree.rename(DEST)
    check(lock)


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError, RuntimeError, subprocess.CalledProcessError, tarfile.TarError) as error:
        raise SystemExit(str(error))
