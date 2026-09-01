"""Brick 4 maintenance - keep the sqlite database bounded (Phase 2 goal 3).

The fleet appends rows forever; an unbounded sqlite file eventually degrades
queries and can fill the disk. This job checks the DATABASE FILE'S SIZE ON
DISK against a budget; once crossed, every managed table has its OLDEST
~10-20% of rows deleted (oldest by timestamp), then VACUUM reclaims the freed
pages. Safe to run from cron alongside the live listener - WAL mode, short
transactions, never touches the current write path's schema.

Run daily (or hourly on a busy fleet) via cron:
    python3 db_retention.py /path/to/sensor_data.db
Options via env:
    OMEGA_DB_MAX_SIZE_MB  database file size budget in MB (default 500)
    OMEGA_DB_PRUNE_FRAC   fraction removed per table when over budget (default 0.15)
"""

import os
import sqlite3
import sys

MANAGED_TABLES = ("noise_data", "air_data")
DEFAULT_MAX_SIZE_MB = 500
DEFAULT_PRUNE_FRACTION = 0.15
MIN_PRUNE_FRACTION = 0.10
MAX_PRUNE_FRACTION = 0.20
BYTES_PER_MB = 1024 * 1024


def _on_disk_bytes(db_path):
    """Total bytes this database occupies, not just the main file. In WAL mode
    recent writes live in the sibling -wal file until a checkpoint folds them
    back in; measuring only the .db under-reports real disk use (observed
    2026-08-21: 2.8 MB db alongside a 4.1 MB wal)."""
    total = 0
    for suffix in ("", "-wal", "-shm"):
        try:
            total += os.path.getsize(db_path + suffix)
        except OSError:
            pass
    return total


def _row_count(cursor, table):
    cursor.execute("SELECT COUNT(*) FROM " + table)
    return cursor.fetchone()[0]


def prune_table(conn, table, fraction):
    """Unconditionally delete the oldest `fraction` of this table's rows.
    The caller (run()) already decided pruning is needed - this function does
    not re-check a budget of its own. Returns the number of rows deleted."""
    fraction = max(MIN_PRUNE_FRACTION, min(MAX_PRUNE_FRACTION, fraction))
    cursor = conn.cursor()
    count = _row_count(cursor, table)
    to_delete = int(count * fraction)
    if to_delete <= 0:
        return 0
    cursor.execute(
        "DELETE FROM " + table + " WHERE rowid IN ("
        "SELECT rowid FROM " + table + " ORDER BY timestamp ASC, rowid ASC LIMIT ?)",
        (to_delete,))
    conn.commit()
    return to_delete


def run(db_path, max_size_mb=DEFAULT_MAX_SIZE_MB, fraction=DEFAULT_PRUNE_FRACTION):
    if not os.path.exists(db_path):
        print("retention: db not found: " + db_path)
        return 1

    size_mb = _on_disk_bytes(db_path) / BYTES_PER_MB
    print("retention: db size %.1f MB (budget %d MB)" % (size_mb, max_size_mb))
    if size_mb <= max_size_mb:
        print("retention: under budget, nothing to do")
        return 0

    conn = sqlite3.connect(db_path, timeout=30)
    conn.execute("PRAGMA journal_mode=WAL;")
    total_deleted = 0
    for table in MANAGED_TABLES:
        try:
            deleted = prune_table(conn, table, fraction)
        except sqlite3.OperationalError as error:
            print("retention: skipped " + table + " (" + str(error) + ")")
            continue
        total_deleted += deleted
        print("retention: %s pruned %d rows" % (table, deleted))
    if total_deleted > 0:
        conn.execute("VACUUM;")
        conn.commit()
        print("retention: vacuumed, reclaimed space for %d rows" % total_deleted)
    conn.close()
    return 0


def main(argv):
    if len(argv) < 2:
        print("usage: db_retention.py <db_path>")
        return 2
    max_size_mb = int(os.environ.get("OMEGA_DB_MAX_SIZE_MB", DEFAULT_MAX_SIZE_MB))
    fraction = float(os.environ.get("OMEGA_DB_PRUNE_FRAC", DEFAULT_PRUNE_FRACTION))
    return run(argv[1], max_size_mb, fraction)


if __name__ == "__main__":
    sys.exit(main(sys.argv))
