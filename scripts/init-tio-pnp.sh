#!/usr/bin/env bash
# Configure the Mini TIO DO pins for PNP output, for tools that expect a
# high level to switch (e.g. a PNP suction cup valve).
#
# The controller silently ignores pin mode changes while the servos are
# enabled, so the robot is disabled first and enabled afterwards.
#
# Env:  DO_MODE=pnp    npn, pnp, push-pull, rs485 or a hex byte like 11
#       DI_MODE=        set to pnp to also configure the DI pins as PNP
#                       inputs, empty leaves them as they are
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
CLI="${JAKA_CLI:-$HERE/build/jaka-cli}"
IP="${JAKA_IP:-10.5.5.100}"
DO_MODE="${DO_MODE:-pnp}"
DI_MODE="${DI_MODE:-}"

echo "Disabling the servos to unlock the pin configuration..."
"$CLI" --ip "$IP" disable || exit 1
sleep 1

echo "Setting the tool DO pins to $DO_MODE..."
"$CLI" --ip "$IP" tio-pin do "$DO_MODE" || exit 1

if [ -n "$DI_MODE" ]; then
    echo "Setting the tool DI pins to $DI_MODE..."
    "$CLI" --ip "$IP" tio-pin di "$DI_MODE" || exit 1
fi

echo "Re-enabling the servos..."
sleep 1
"$CLI" --ip "$IP" enable || exit 1

echo "Verifying:"
"$CLI" --ip "$IP" tio-pin do
echo "Done. do tool 0 on now drives the PNP tool."
