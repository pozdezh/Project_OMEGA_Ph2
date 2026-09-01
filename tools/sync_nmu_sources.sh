#!/bin/sh
# Copies the canonical NMU sources into every shipped copy of them.
#
# There are three: the flashing sketch, the operator template, and the
# pendrive provisioning image. Editing them by hand is three chances to ship
# a unit running last week's firmware, and the gate's byte-sync check exists
# because that already happened.
set -e
ROOT=$(cd "$(dirname "$0")/.." && pwd)
SRC="$ROOT/nmu"
for dest in \
  "$ROOT/deploy/esp32/stage3_beta_2_dtls" \
  "$ROOT/deploy/esp32/NMU_TEMPLATE/stage3_beta_2_dtls" \
  "$ROOT/provisioning/pendrive/firmware"; do
  [ -d "$dest" ] || continue
  for f in "$SRC"/*.cpp "$SRC"/*.h "$SRC"/*.ino "$SRC"/partitions.csv; do
    name=$(basename "$f")
    [ -f "$dest/$name" ] || continue
    cp -p "$f" "$dest/$name"
  done
  echo "synced ${dest#$ROOT/}"
done
