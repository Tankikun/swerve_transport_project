#!/usr/bin/env bash
# regenerate_map.sh — turn ~/maps/tb3_1_room.db into interface/map.json.
# Run from the interface/ directory.
#
# Usage:
#     ./regenerate_map.sh                              # uses ~/maps/tb3_1_room.db -> map.json
#     ./regenerate_map.sh path/to/other.db             # custom .db -> map.json
#     ./regenerate_map.sh path/to/other.db custom.json # custom .db -> custom.json
#
# Flags below mirror the recommended values in interface/README.md. Tweak
# them there (or pass extra args after the second positional) if your scene
# is bigger than 8 m x 8 m or your camera reaches further than 3 m.

set -euo pipefail
DB="${1:-$HOME/maps/tb3_1_room.db}"
OUT="${2:-map.json}"
echo "[regenerate_map] db=$DB  ->  $OUT"
test -f "$DB" || { echo "ERROR: db not found at $DB"; exit 1; }
/usr/bin/env python3 db_to_map_json.py \
    --db "$DB" \
    --output "$OUT" \
    --ceiling-above-floor 1.5 \
    --bbox -4 4 -4 4 \
    --rtabmap-max-range 3.0 \
    --rtabmap-noise-radius 0.05 \
    --rtabmap-noise-k 10 \
    --rtabmap-prop-radius 0.01
echo "[regenerate_map] done. $(du -h "$OUT" | cut -f1) at $OUT"
