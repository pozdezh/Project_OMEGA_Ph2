"""The learned heartbeat must survive a process restart.

Found on AMU_15 during the 2026-08-26 field test: a process restart (a
crash, or the recovery ladder's OWN restart rung) wiped the server-learned
heartbeat back to global.ini's static default. That default was a stale
30-minute value left over from before the fleet moved to a 15-minute server
heartbeat, so the unit's recovery ladder ran at HALF the intended speed for
the one outage it exists to survive - a real unit sat cut off for over an
hour before its first escalation rung even fired.

    py -3.12 amu/test_heartbeat_persistence.py
"""

import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
FIXTURE_INI = os.path.join(HERE, "test_fixture.ini")
FIXTURE_DEFAULT_S = 300  # alarm_heartbeat_interval in test_fixture.ini


def _fresh_state_path():
    return os.path.join(tempfile.mkdtemp(prefix="omega_hb_"), "heartbeat_state.json")


def _fresh_env(state_path):
    env = dict(os.environ, OMEGA_AMU_CONFIG=FIXTURE_INI,
               OMEGA_HEARTBEAT_STATE=state_path)
    env.pop("PYTHONDONTWRITEBYTECODE", None)
    return env


def _read_heartbeat_in_fresh_process(state_path):
    """A real restart re-imports amu_config from scratch - the only way to
    prove persistence actually helps is a genuinely new process, not a
    second call inside the same one."""
    result = subprocess.run(
        [sys.executable, "-c",
         "import sys; sys.path.insert(0, %r); import amu_config; "
         "print(amu_config.heartbeat_interval)" % HERE],
        env=_fresh_env(state_path), capture_output=True, text=True, timeout=15)
    assert result.returncode == 0, "fresh import failed: " + result.stderr
    return int(result.stdout.strip())


def test_no_persisted_value_falls_back_to_the_ini_default():
    state_path = _fresh_state_path()
    got = _read_heartbeat_in_fresh_process(state_path)
    assert got == FIXTURE_DEFAULT_S, \
        "first-ever boot must read global.ini, got " + str(got)
    print("PASS first boot (no persisted value) uses the ini default")


def test_a_learned_heartbeat_survives_a_fresh_process():
    state_path = _fresh_state_path()
    sys.path.insert(0, HERE)
    os.environ["OMEGA_AMU_CONFIG"] = FIXTURE_INI
    os.environ["OMEGA_HEARTBEAT_STATE"] = state_path
    for name in ("amu_config",):
        sys.modules.pop(name, None)
    import amu_config

    changed = amu_config.apply_learned_heartbeat_s(900)
    assert changed is True, "a genuinely new value must report a change"
    assert amu_config.heartbeat_interval == 900

    got = _read_heartbeat_in_fresh_process(state_path)
    assert got == 900, \
        "a restarted process must start from the LEARNED value, not the " \
        "ini default - got " + str(got)
    print("PASS a learned heartbeat survives a fresh process (the AMU_15 bug)")


def test_an_unchanged_heartbeat_does_not_rewrite_the_file():
    """The fleet re-handshakes every few minutes. Rewriting on every ACK
    would be needless SD-card wear, same reasoning as device_addresses.py
    on the server."""
    state_path = _fresh_state_path()
    sys.path.insert(0, HERE)
    os.environ["OMEGA_AMU_CONFIG"] = FIXTURE_INI
    os.environ["OMEGA_HEARTBEAT_STATE"] = state_path
    for name in ("amu_config",):
        sys.modules.pop(name, None)
    import amu_config

    assert amu_config.apply_learned_heartbeat_s(900) is True
    mtime_after_first_write = os.path.getmtime(state_path)

    again = amu_config.apply_learned_heartbeat_s(900)
    assert again is False, "an unchanged heartbeat must not report a change"
    assert os.path.getmtime(state_path) == mtime_after_first_write, \
        "an unchanged heartbeat must not touch the file on disk"
    print("PASS a routine unchanged heartbeat does not rewrite the file")


def main():
    test_no_persisted_value_falls_back_to_the_ini_default()
    test_a_learned_heartbeat_survives_a_fresh_process()
    test_an_unchanged_heartbeat_does_not_rewrite_the_file()
    print("RESULT: PASS - learned heartbeat survives a restart, AMU_15's bug fixed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
