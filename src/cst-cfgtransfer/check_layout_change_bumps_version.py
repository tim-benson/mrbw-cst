#!/usr/bin/env python3
# Copyright (C) 2026 Tim Benson <blw@east-slope.com>
# License: GNU General Public License v3 (see LICENSE)

"""Guards against a commit that changes src/cst-eeprom.h's actual layout (new field, moved offset,
repurposed byte) but never bumps EEPROM_LAYOUT_VERSION. Without the bump the CNF format version stays
stale and offline tooling can't tell a re-laid-out chip apart from an in-sync one - this script closes
that gap by diffing cst-eeprom.h's own #define set against its previous committed state.

Scope: this only guarantees the version number moves whenever cst-eeprom.h's layout does. It cannot verify
that cst_eeprom_layout.py/slot_codec.py's *content* was actually updated to match (see CLAUDE.md's
maintenance checklist for that), and it cannot see a semantic reinterpretation of a byte whose offset/name
didn't change - that logic lives in mrbw-cst.c's readConfig(), not in the header.

Run standalone:
    python3 src/cst-cfgtransfer/check_layout_change_bumps_version.py

Wired into .githooks/pre-commit via `make setup`'s `git config core.hooksPath .githooks` - see that file.
This is a local safety net, not enforcement: `git commit --no-verify` bypasses it, same as any git hook.
"""

import re
import subprocess
import sys

EEPROM_H_PATH = "src/cst-eeprom.h"
VERSION_MACRO = "EEPROM_LAYOUT_VERSION"

# Matches "#define NAME value" and "#define NAME(args) value" (the one function-like macro in this file,
# CONFIG_OFFSET) - captures the value up to end of line. The bare include-guard "#define _CST_EEPROM_H_"
# has no trailing value and simply won't match. Comment lines never start with #define, so a pure
# comment/formatting edit can't be mistaken for a layout change.
_DEFINE_RE = re.compile(r"^#define\s+(\w+)(?:\([^)]*\))?\s+(.+?)\s*$", re.MULTILINE)


def _extract_defines(content):
    return dict(_DEFINE_RE.findall(content))


def check_layout_change_requires_bump(old_content, new_content):
    """Pure diff logic, no git/filesystem involved. Returns None if OK, or an error message string
    naming exactly which #define(s) changed."""
    old_defines = _extract_defines(old_content)
    new_defines = _extract_defines(new_content)
    old_version = old_defines.get(VERSION_MACRO)
    new_version = new_defines.get(VERSION_MACRO)

    old_rest = {k: v for k, v in old_defines.items() if k != VERSION_MACRO}
    new_rest = {k: v for k, v in new_defines.items() if k != VERSION_MACRO}

    if old_rest == new_rest:
        return None  # no layout change either way
    if old_version != new_version:
        return None  # a layout change with its required bump - the correct, expected case

    added = sorted(set(new_rest) - set(old_rest))
    removed = sorted(set(old_rest) - set(new_rest))
    changed = sorted(k for k in (set(new_rest) & set(old_rest)) if old_rest[k] != new_rest[k])

    lines = [
        "check_layout_change_bumps_version: MISMATCH",
        "  %s changed layout-defining #define(s) but EEPROM_LAYOUT_VERSION is still %s:" %
        (EEPROM_H_PATH, new_version),
    ]
    for name in added:
        lines.append("    added:   %s = %s" % (name, new_rest[name]))
    for name in removed:
        lines.append("    removed: %s (was %s)" % (name, old_rest[name]))
    for name in changed:
        lines.append("    changed: %s: %s -> %s" % (name, old_rest[name], new_rest[name]))
    lines.append(
        "Bump EEPROM_LAYOUT_VERSION in cst-eeprom.h alongside this change, and update the field tables "
        "in cst_eeprom_layout.py/slot_codec.py to match (SUPPORTED_LAYOUT_VERSION follows the header "
        "automatically) - see CLAUDE.md's \"PC tooling\" maintenance checklist."
    )
    return "\n".join(lines) + "\n"


def _git_show(repo_root, rev, path):
    # rev="HEAD" -> "HEAD:path" (a specific commit); rev="" -> ":path" (the index/staged content) - git's
    # own syntax for the latter is a single leading colon, not a "<rev>:" prefix with rev=":".
    result = subprocess.run(["git", "show", "%s:%s" % (rev, path)], cwd=repo_root,
                             capture_output=True, text=True)
    if result.returncode != 0:
        return None  # file did not exist at that revision (or there is no such revision yet)
    return result.stdout


def main():
    repo_root = subprocess.run(["git", "rev-parse", "--show-toplevel"], capture_output=True,
                                text=True, check=True).stdout.strip()

    old_content = _git_show(repo_root, "HEAD", EEPROM_H_PATH)
    if old_content is None:
        print("check_layout_change_bumps_version: OK (no previous commit to compare against)")
        return 0

    new_content = _git_show(repo_root, "", EEPROM_H_PATH)
    if new_content is None:
        print("check_layout_change_bumps_version: OK (%s not staged)" % EEPROM_H_PATH)
        return 0

    message = check_layout_change_requires_bump(old_content, new_content)
    if message:
        sys.stderr.write(message)
        return 1
    print("check_layout_change_bumps_version: OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
