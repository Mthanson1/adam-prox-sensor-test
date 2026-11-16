#!/usr/bin/env python3
"""Check -- or fix -- C/C++ formatting against .clang-format.

CI and developers both run this script, so the file list cannot drift between
them.

    python scripts/check_format.py          # report violations, exit 1 if any
    python scripts/check_format.py --fix    # rewrite the offending files
"""

import argparse
import re
import shutil
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent

SOURCE_DIRS = ("src", "include")
EXTENSIONS = ("c", "cc", "cpp", "h", "hpp")


def pinned_version():
    """The clang-format version requirements-dev.txt asks for, if pinned."""
    req = REPO_ROOT / "requirements-dev.txt"
    if not req.exists():
        return None
    match = re.search(r"^clang-format==(\S+)", req.read_text(), re.MULTILINE)
    return match.group(1) if match else None


def installed_version(clang_format):
    out = subprocess.run(
        [clang_format, "--version"], capture_output=True, text=True, check=True
    ).stdout
    match = re.search(r"version (\S+)", out)
    return match.group(1) if match else out.strip()


def source_files():
    """Tracked sources under SOURCE_DIRS, via git so ignored files stay out."""
    pathspecs = [f"{d}/*.{ext}" for d in SOURCE_DIRS for ext in EXTENSIONS]
    out = subprocess.run(
        ["git", "ls-files", "-z", "--", *pathspecs],
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
        check=True,
    ).stdout
    return sorted(path for path in out.split("\0") if path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--fix", action="store_true", help="rewrite files in place instead of reporting"
    )
    parser.add_argument(
        "--clang-format",
        default=shutil.which("clang-format"),
        help="path to the clang-format binary (default: first on PATH)",
    )
    args = parser.parse_args()

    if not args.clang_format:
        print(
            "clang-format not found on PATH.\n"
            "Install the pinned version with: pip install -r requirements-dev.txt",
            file=sys.stderr,
        )
        return 2

    found, wanted = installed_version(args.clang_format), pinned_version()
    print(f"clang-format {found} ({args.clang_format})")
    if wanted and found != wanted:
        # Not fatal -- a mismatch usually still agrees, but when it does not the
        # symptom is a CI failure that reproduces nowhere, so say so up front.
        print(
            f"warning: requirements-dev.txt pins {wanted}; results may differ from CI",
            file=sys.stderr,
        )

    files = source_files()
    if not files:
        print(f"No sources found under {'/, '.join(SOURCE_DIRS)}/ -- nothing to check.")
        return 0

    mode = ["-i"] if args.fix else ["--dry-run", "--Werror"]
    result = subprocess.run(
        [args.clang_format, "--style=file", *mode, *files], cwd=REPO_ROOT
    )

    if args.fix:
        print(f"Formatted {len(files)} file(s) in place.")
        return result.returncode
    if result.returncode != 0:
        print(
            "\nFormatting violations above. Fix them with:\n"
            "    python scripts/check_format.py --fix",
            file=sys.stderr,
        )
        return result.returncode

    print(f"All {len(files)} file(s) match .clang-format.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
