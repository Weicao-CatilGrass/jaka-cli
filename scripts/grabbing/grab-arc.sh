#!/bin/sh
# Continuous arc grab: carry a block from (fromx, fromy) to (tox, toy).
#
# Coordinates are mm relative to the base point, the one stored by
# restore + set-base. The whole pick is ONE continuous servo trajectory in
# jaka-cli (the `grab` primitive): it rises from the current pose to hover
# over the source, probes down and grips the block, carries it in an arc to
# hover over the target, probes down to seat it, releases the suction and
# lifts away, with no stop between the motions, only the two suction dwells.
#
# Set the base point once first:
#   jaka-cli restore
#   jaka-cli set-base
#
# Usage: grab-arc.sh fromx fromy tox toy
#   All four are mm offsets from the base point.

set -eu

[ $# -eq 4 ] || {
    echo "usage: grab-arc.sh fromx fromy tox toy   (mm, relative to the base point)" >&2
    exit 1
}
fromx=$1
fromy=$2
tox=$3
toy=$4

SPEED=${SPEED:-2000}   # linear speed in mm/s, the servo clamps it
APEX=${APEX:-80}       # rise of the carry arc above the straight line in mm

# Controller and binary, override via the environment
IP=${JAKA_IP:-10.5.5.100}

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
if [ -n "${JAKA_CLI:-}" ]; then
    CLI=$JAKA_CLI
elif [ -x "$SCRIPT_DIR/../../build/jaka-cli" ]; then
    CLI=$SCRIPT_DIR/../../build/jaka-cli
else
    CLI=$(command -v jaka-cli || true)
fi
if [ -z "${CLI:-}" ]; then
    echo "jaka-cli not found. Build it first (make build in jaka-test) or set JAKA_CLI" >&2
    exit 1
fi

echo "grab from rel ($fromx, $fromy) to rel ($tox, $toy)"
exec "$CLI" --ip "$IP" grab "$fromx" "$fromy" "$tox" "$toy" --rel --apex "$APEX" --speed "$SPEED"
