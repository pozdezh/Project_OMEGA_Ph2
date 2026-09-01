"""Brick 4 - wipe stored sensor readings without touching the live listener.

Opens its own short-lived connection to the SAME database file the listener
is already using (WAL mode allows this safely) - the listener's own
connection is never closed, so nothing needs restarting and no data in
flight is lost. This is the same mechanism db_retention.py already uses
every night; this script just empties the tables completely instead of
trimming the oldest 10-20%.

Run:
    python3 clear_database.py /path/to/sensor_data.db --confirm
Omit --confirm to see row counts without deleting anything (dry run).
"""

import os
import sqlite3
import sys

MANAGED_TABLES = ("noise_data", "air_data")


def _row_count(cursor, table):
    cursor.execute("SELECT COUNT(*) FROM " + table)
    return cursor.fetchone()[0]


def run(db_path, confirmed):
    if not os.path.exists(db_path):
        print("clear_database: db not found: " + db_path)
        return 1

    conn = sqlite3.connect(db_path, timeout=30)
    conn.execute("PRAGMA journal_mode=WAL;")
    cursor = conn.cursor()

    counts = {table: _row_count(cursor, table) for table in MANAGED_TABLES}
    for table, count in counts.items():
        print("%s: %d rows" % (table, count))

    if not confirmed:
        print("Dry run - nothing deleted. Re-run with --confirm to actually erase.")
        conn.close()
        return 0

    for table in MANAGED_TABLES:
        cursor.execute("DELETE FROM " + table)
    conn.commit()
    conn.execute("VACUUM")
    conn.commit()

    for table in MANAGED_TABLES:
        print("%s: %d -> %d" % (table, counts[table], _row_count(cursor, table)))
    conn.close()
    print("done - listener untouched, no restart needed")
    return 0


def main(argv):
    if len(argv) < 2:
        print("usage: clear_database.py <db_path> [--confirm]")
        return 2
    return run(argv[1], "--confirm" in argv[2:])


if __name__ == "__main__":
    sys.exit(main(sys.argv))
