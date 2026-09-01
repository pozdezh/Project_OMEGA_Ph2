"""Build and run the NMU record-dating proof (C++), from the gate.

The NMU is C++ that normally only runs on hardware, so its logic sits outside
everything the gate can check. That is exactly how the timestamp bug survived:
`late backlog fills the gap it left` passes, because it exercises the case
where the clock was already synced - the only case that was ever correct.

nmu/omega_time.h was split out to be free of Arduino types so the one rule
that got it wrong can be compiled and proved on this PC. This wrapper is what
lets the gate do that.

Skips rather than fails when no C++ compiler is present: the gate must stay
runnable on a machine without a toolchain, and a skip that says so is honest
where a silent pass is not.

    py -3.12 simlab/test_nmu_time_wrapper.py
"""

import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
SOURCE = os.path.join(HERE, "test_nmu_time.cpp")
INCLUDE = os.path.join(ROOT, "nmu")


def _to_wsl(path):
    drive, rest = os.path.splitdrive(os.path.abspath(path))
    return "/mnt/" + drive[0].lower() + rest.replace("\\", "/")


def _run(command):
    return subprocess.run(command, capture_output=True, text=True, timeout=300)


def main():
    if os.name == "nt":
        binary = "/tmp/omega_test_nmu_time"
        script = "g++ -std=c++11 -Wall -Wextra -I %s -o %s %s && %s" % (
            _to_wsl(INCLUDE), binary, _to_wsl(SOURCE), binary)
        result = _run(["wsl", "-e", "bash", "-c", script])
    else:
        binary = os.path.join(tempfile.gettempdir(), "omega_test_nmu_time")
        build = _run(["g++", "-std=c++11", "-Wall", "-Wextra",
                      "-I", INCLUDE, "-o", binary, SOURCE])
        if build.returncode != 0:
            print(build.stderr)
            return 1
        result = _run([binary])

    print(result.stdout.strip())
    if result.returncode != 0:
        stderr = (result.stderr or "").strip()
        if "command not found" in stderr or "not recognized" in stderr:
            print("SKIPPED: no C++ compiler available to build the NMU proof")
            return 0
        if stderr:
            print(stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
