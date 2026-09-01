"""Catch the one installer bug that breaks a FRESH install and nothing else.

Both installers copy an explicit list of files. Twice now a module was added
to the payload, imported by code that was already being copied, and left out
of that list - so every existing box kept working (the file was already
there) while a brand new install died at startup with ModuleNotFoundError.
That is the worst possible failure shape for a project whose stated goal is
that a non-technical person can install it.

This walks each installer's copy list, reads every Python file it copies,
resolves the local modules those files import, and fails if any of them is
not itself copied. No hardware, no network, no root.

    py -3.12 deploy/test_installer_payload.py
"""

import ast
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))

TARGETS = (
    ("server", os.path.join(HERE, "server", "install_server.sh"),
     os.path.join(HERE, "server", "payload")),
    ("amu", os.path.join(HERE, "amu", "install_amu.sh"),
     os.path.join(HERE, "amu", "payload")),
)

# Files that exist to be run by hand or by a test, never imported by the
# service at startup. Missing one degrades a tool, it does not stop the unit.
OPTIONAL = {"test_buffer_triggers.py", "test_fixture.ini", "clear_database.py"}


def copied_files(installer_path):
    """Every payload filename the installer copies, however it spells it."""
    with open(installer_path, "r", encoding="utf-8") as handle:
        script = handle.read()
    names = set(re.findall(r'\$PAYLOAD/([A-Za-z0-9_./-]+)', script))
    # The `for f in a.py b.py \ <newline> c.py; do` form names files without
    # the $PAYLOAD prefix, so pick those up from the loop headers too.
    for block in re.findall(r'for f in (.*?);\s*do', script, re.S):
        for token in block.replace("\\\n", " ").split():
            if token.endswith(".py") or token.endswith(".txt"):
                names.add(token)
    # Relative paths, not basenames: the AMU installer copies
    # config/global.ini.example, which lives in a subdirectory of the payload.
    # Flattening it to a basename made the existence check look for it in the
    # payload root and report a file that is present as missing.
    return names


def local_imports(py_path, payload_dir):
    """Modules this file imports that live in the payload beside it."""
    with open(py_path, "r", encoding="utf-8") as handle:
        try:
            tree = ast.parse(handle.read())
        except SyntaxError as error:
            print("  SYNTAX ERROR in %s: %s" % (os.path.basename(py_path), error))
            return set()
    found = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            candidates = [alias.name for alias in node.names]
        elif isinstance(node, ast.ImportFrom):
            candidates = [node.module] if node.module and node.level == 0 else []
        else:
            continue
        for name in candidates:
            root = name.split(".")[0]
            if os.path.exists(os.path.join(payload_dir, root + ".py")):
                found.add(root + ".py")
    return found


def check(label, installer_path, payload_dir):
    print("== %s (%s)" % (label, os.path.basename(installer_path)))
    if not os.path.exists(installer_path):
        print("  MISSING installer"); return 1
    copied = copied_files(installer_path)
    # Imports resolve by module name, so the import walk below compares
    # against basenames while the existence check above uses the real
    # relative path.
    copied_basenames = {os.path.basename(n) for n in copied}
    print("  installer copies %d payload file(s)" % len(copied))

    problems = 0
    for name in sorted(copied):
        # Existence is checked for EVERY copied file, whatever its extension.
        # This loop used to skip anything that was not a .py before it got as
        # far as the existence check, and so walked straight past a missing
        # device_config.example.json - the one file whose absence stops the
        # listener from starting at all. The import walk below is still
        # Python-only, because only Python files have imports.
        source = os.path.join(payload_dir, name)
        if not os.path.exists(source):
            print("  FAIL %s is copied but is NOT in the payload" % name)
            problems += 1
            continue
        if not name.endswith(".py"):
            continue
        for needed in sorted(local_imports(source, payload_dir)):
            if needed not in copied_basenames:
                print("  FAIL %s imports %s, which the installer does NOT copy "
                      "- a fresh install dies at startup" % (name, needed))
                problems += 1

    orphans = sorted(
        f for f in os.listdir(payload_dir)
        if f.endswith(".py") and f not in copied_basenames and f not in OPTIONAL)
    if orphans:
        print("  NOTE payload files never copied (intentional?): %s"
              % ", ".join(orphans))

    if problems == 0:
        print("  PASS every imported local module is copied")
    return problems


NGINX_RUNTIME_VARS = ("host", "remote_addr", "request_uri", "scheme", "uri",
                      "ssl_client_s_dn", "ssl_client_verify", "proxy_add_x_forwarded_for")


def check_nginx_vars(installer_path):
    """An nginx variable written bare inside an UNQUOTED heredoc is expanded by
    the shell at install time, not by nginx at request time. It lands as an
    empty string, nginx -t rejects the directive, and the mTLS gateway silently
    never comes up - leaving the dashboard unauthenticated on :8081. Every one
    of them must be written \\$name."""
    print("== nginx variables survive the installer heredoc (%s)"
          % os.path.basename(installer_path))
    if not os.path.exists(installer_path):
        print("  MISSING installer"); return 1
    with open(installer_path, "r", encoding="utf-8") as handle:
        text = handle.read()
    problems = 0
    for name in NGINX_RUNTIME_VARS:
        for match in re.finditer(r"(.?)\$" + name + r"\b", text):
            if match.group(1) != "\\":
                line = text[:match.start()].count("\n") + 1
                print("  FAIL line %d: $%s is not escaped - the shell will eat "
                      "it and nginx -t will reject the config" % (line, name))
                problems += 1
    if problems == 0:
        print("  PASS every nginx runtime variable is escaped")
    return problems


def main():
    total = 0
    total += check_nginx_vars(os.path.join(HERE, "server", "install_server.sh"))
    print()
    for label, installer, payload in TARGETS:
        total += check(label, installer, payload)
        print()
    if total:
        print("RESULT: FAIL - %d installer/payload mismatch(es)" % total)
        return 1
    print("RESULT: PASS - installers copy everything their own code imports")
    return 0


if __name__ == "__main__":
    sys.exit(main())
