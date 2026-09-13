#!/bin/sh
# Linear grab: carry a block from (fromx, fromy) to (tox, toy).
#
# Coordinates are mm relative to the base point (restore + set-base). Every
# move is a straight move-to in one --sequence on a single connection: hover
# over the source at MID_Z, drop onto the block, grip, lift back to MID_Z,
# travel horizontally at MID_Z, drop to seat it, release, lift away. No servo
# arc, so the motion is deterministic point-to-point straight lines; the
# block clears the table while traveling at MID_Z.
#
# Usage: grab-linear.sh fromx fromy tox toy

set -eu

[ $# -eq 4 ] || {
    echo "usage: grab-linear.sh fromx fromy tox toy   (mm, relative to the base point)" >&2
    exit 1
}
fromx=$1
fromy=$2
tox=$3
toy=$4

SPEED=${SPEED:-2000}          # speed of every move in mm/s
MID_Z=${MID_Z:--40}           # cruise height, above the block while traveling
GRAB_Z=${GRAB_Z:--125}        # head height where the cup touches the block top

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

echo "linear grab from rel ($fromx, $fromy) to rel ($tox, $toy)"
exec "$CLI" --ip "$IP" --sequence \
    "do tool 0 off" "do tool 1 off" \
    "move-to $fromx $fromy $MID_Z --rel --speed $SPEED" \
    "move-to $fromx $fromy $GRAB_Z --rel --speed $SPEED" \
    "do tool 0 on" "do tool 1 on" \
    "move-to $fromx $fromy $MID_Z --rel --speed $SPEED" \
    "move-to $tox $toy $MID_Z --rel --speed $SPEED" \
    "move-to $tox $toy $GRAB_Z --rel --speed $SPEED" \
    "do tool 0 off" "do tool 1 off" \
    "move-to $tox $toy $MID_Z --rel --speed $SPEED"
