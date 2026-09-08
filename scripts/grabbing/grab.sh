#!/bin/sh
# Blind-grab primitive: carry a block from (fromx, fromy) to (tox, toy).
#
# Coordinates are mm relative to the base point, the one stored by
# restore + set-base, with +x forward and +y to the robot's left. The table
# is a single flat plane at rel -136 and the blocks share a top at rel -125,
# so the same GRAB_Z height both lands the cup on a block and seats one on
# the table. To pick, the arm approaches from above so it never knocks the
# block; to place it lowers onto the table, waits for the block to rest, and
# only then releases the suction, so a block is set down, never dropped.
#
# Set the base point once first:
#   jaka-cli restore
#   jaka-cli set-base
#
# Usage: grab.sh fromx fromy tox toy
#   All four are mm offsets from the base point. Speed and the geometry are
#   tunable through the constants below or the env vars SPEED and GRAB_Z.

set -eu

[ $# -eq 4 ] || {
    echo "usage: grab.sh fromx fromy tox toy   (mm, relative to the base point)" >&2
    exit 1
}
fromx=$1
fromy=$2
tox=$3
toy=$4

SPEED=${SPEED:-2000}   # speed of every move in mm/s
MID_Z=-40              # cruise height between the pickup and the pad
GRAB_Z=-125            # head height where the cup touches the block top / seats a block
SETTLE=0.6             # seconds to wait for the suction before lifting
SETTLE_AFTER=0.5       # seconds to wait after release before lifting away

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

# Move the TCP relative to the base point
mv() {
    echo "move rel $*"
    "$CLI" --ip "$IP" move-to --rel "$@" --speed "$SPEED"
}

# Drive both tool outputs: on grabs, off releases
suck() {
    echo "suction $1"
    "$CLI" --ip "$IP" do tool 0 "$1"
    "$CLI" --ip "$IP" do tool 1 "$1"
}

echo "grab from rel ($fromx, $fromy) to rel ($tox, $toy)"

# Pick: hover over the block, settle on its top, grab, lift to cruise height
suck off
mv "$fromx" "$fromy" "$MID_Z"
mv "$fromx" "$fromy" "$GRAB_Z"
suck on
sleep "$SETTLE"
mv "$fromx" "$fromy" "$MID_Z"

# Place: cruise over the target, lower the block onto the table, release
# only once it rests, then lift away
mv "$tox" "$toy" "$MID_Z"
mv "$tox" "$toy" "$GRAB_Z"
sleep "$SETTLE_AFTER"
suck off
sleep "$SETTLE_AFTER"
mv "$tox" "$toy" "$MID_Z"

echo "done, block moved from rel ($fromx, $fromy) to rel ($tox, $toy)"
