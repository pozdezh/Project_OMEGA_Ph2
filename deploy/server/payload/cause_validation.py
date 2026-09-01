"""Closed vocabulary for the AMU's "cause" field.

It is the one free-text-shaped value a device controls that later reaches an
operator's AI context, via latest_reading and the dashboard. A compromised or
spoofed device could try to smuggle instructions into that text. Device-side
validation is not a boundary - a compromised AMU can simply skip it - so this
module is the only authority, and nothing downstream may see an unvalidated
value.

amu/triggers.py composes the real value from a small, fixed grammar: a
handful of literal phrases, or one of a fixed set of metric labels, optionally
followed by " (+N)" when several conditions fired together (observed live:
"CO2 Spike/Drop (+1)", "New Alarm: High CO2 (+2)"). A flat list of exact
strings cannot represent that - every multi-condition reading produces a
different literal sentence - so this validates the GRAMMAR, matched exactly
to triggers.py's ALARM_RULES / DELTA_RULES / STEP_RULES, not a guessed list.
"""

import re

LITERAL_CAUSES = frozenset({
    "Operator Query",
    "Initial Server Sync",
    "All Alarms Cleared",
    "Routine Heartbeat",
    "Unknown",
})

# Must match triggers.ALARM_RULES's first column exactly.
ALARM_LABELS = frozenset({
    "High CO2", "High PM2.5", "High Temp", "Low Temp",
    "High Humidity", "Low Humidity", "High Lux", "Low Lux",
})

# Must match "%s Spike/Drop" / "%s Step/Drift" for every label in
# triggers.DELTA_RULES / triggers.STEP_RULES.
CHANGE_LABELS = frozenset({
    "CO2 Spike/Drop", "PM1.0 Spike/Drop", "PM2.5 Spike/Drop", "PM10.0 Spike/Drop",
    "Temp Step/Drift", "Humidity Step/Drift", "Lux Step/Drift",
    "CO2 Step/Drift", "PM1.0 Step/Drift", "PM2.5 Step/Drift", "PM10.0 Step/Drift",
})

MAX_CAUSE_LEN = 64
# Real ceiling is len(ALARM_RULES) - 1 = 7 simultaneous extra conditions;
# generous headroom costs nothing since N can never carry text.
MAX_EXTRA_COUNT = 20
FALLBACK_CAUSE = "Unknown"

_SUFFIX_RE = re.compile(r"^(.+) \(\+(\d+)\)$")


def _strip_suffix(value):
    """("<base> (+N)", True) becomes (base, True) if N is in range.
    A value with no suffix passes through unchanged as (value, True)."""
    match = _SUFFIX_RE.match(value)
    if not match:
        return value, True
    base, count = match.group(1), int(match.group(2))
    if count < 1 or count > MAX_EXTRA_COUNT:
        return value, False
    return base, True


def sanitize_cause(raw):
    """Return raw unchanged if it matches the AMU's known cause grammar,
    otherwise FALLBACK_CAUSE. Never raises regardless of input type."""
    if not isinstance(raw, str) or not raw or len(raw) > MAX_CAUSE_LEN:
        return FALLBACK_CAUSE

    if raw in LITERAL_CAUSES:
        return raw

    if raw.startswith("Sustained Alarm: "):
        body, suffix_ok = _strip_suffix(raw[len("Sustained Alarm: "):])
        return raw if suffix_ok and body in ALARM_LABELS else FALLBACK_CAUSE

    if raw.startswith("New Alarm: "):
        body, suffix_ok = _strip_suffix(raw[len("New Alarm: "):])
        return raw if suffix_ok and body in ALARM_LABELS else FALLBACK_CAUSE

    body, suffix_ok = _strip_suffix(raw)
    if suffix_ok and body in CHANGE_LABELS:
        return raw

    return FALLBACK_CAUSE
