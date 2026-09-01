"""When to next try emptying the offline buffer.

The rule this encodes: a record that failed to send must be retried on its
OWN clock, seconds later, not whenever the next reading happens to arrive.
On an event-driven unit the next reading can be an hour away, and an alarm
sitting on disk for an hour is not telemetry - it is a lost alarm.

Retries back off while the link stays down so a dead server is not hammered
by the whole fleet, and snap back to the floor the moment anything succeeds.
"""


class RetrySchedule:
    def __init__(self, floor_s, ceiling_s):
        self._floor = float(floor_s)
        self._ceiling = float(ceiling_s)
        self._interval = self._floor
        self._next_at = 0.0

    def due(self, now):
        return now >= self._next_at

    def succeeded(self, now):
        """The link works: be ready to retry promptly again."""
        self._interval = self._floor
        self._next_at = now + self._interval

    def failed(self, now):
        """Still down: wait longer next time, up to the ceiling."""
        self._next_at = now + self._interval
        self._interval = min(self._interval * 2.0, self._ceiling)

    @property
    def interval_s(self):
        return self._interval
