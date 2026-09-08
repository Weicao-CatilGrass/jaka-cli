#!/bin/sh
# Blind-grab primitive: carry a block from (fromx, fromy) to (tox, toy).
#
# Coordinates are mm relative to the base point, the one stored by
# restore + set-base. The table is one flat plane at rel -136 and blocks
# share a top at rel -125. The whole run stays in a low band: the arm
# hovers at APPROACH_Z (-110), descends 15 mm to grab the block, rises 15 mm
# with it, and carries it in one straight move to above the target, then
# probes down 15 mm to seat it. Vertical travel is tiny, so the motion is
# short and fast.
#
# Set the base point once first:
#   jaka-cli restore
#   jaka-cli set-base
#
# Usage: grab.sh fromx fromy tox toy
#   All four are mm offsets from the base point. Speed and the geometry are
#   tunable through the constants below or the env vars SPEED, APPROACH_Z
#   and GRAB_Z.

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
APPROACH_Z=-100        # hover height, 15 mm above the block top
GRAB_Z=-125            # head height where the cup touches the block top
SETTLE=0.6             # seconds to wait for the suction after grabbing
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

# Pick: hover 15 mm over the block, settle on its top, grab
suck off
mv "$fromx" "$fromy" "$APPROACH_Z"
mv "$fromx" "$fromy" "$GRAB_Z"
suck on
sleep "$SETTLE"

# Carry: one straight move up and over to above the target
mv "$tox" "$toy" "$APPROACH_Z"

# Place: probe down until the block rests on the table, release, lift away
mv "$tox" "$toy" "$GRAB_Z"
sleep "$SETTLE_AFTER"
suck off
sleep "$SETTLE_AFTER"
mv "$tox" "$toy" "$APPROACH_Z"

echo "done, block moved from rel ($fromx, $fromy) to rel ($tox, $toy)"
