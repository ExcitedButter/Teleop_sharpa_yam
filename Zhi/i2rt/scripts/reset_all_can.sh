#!/bin/bash

if [ "$(id -u)" != "0" ]; then
    SUDO="sudo"
else
    SUDO=""
fi

# Function to reset a CAN interface
#
# restart-ms 100 asks the kernel to re-initialise the controller ~100 ms after a
# BUS-OFF instead of leaving the link dead until a human resets it. The default
# (restart-ms 0) means one bad burst of bus errors wedges the interface for the
# rest of the session, which looks exactly like "teleop just stopped".
#
# NOT EVERY ADAPTER SUPPORTS IT. The yambox's CAN hardware rejects it with
# "Device doesn't support restart from Bus Off", and because that failure comes
# from the `up` command, adding it unconditionally left the interface DOWN --
# turning a routine reset into a dead follower arm. So: try it, and if the
# interface is not up afterwards, bring it up plainly. A missing restart-ms is a
# lost robustness feature; a down interface is a broken robot.
reset_can_interface() {
    local iface=$1
    echo "Resetting CAN interface: $iface"
    $SUDO ip link set "$iface" down
    if ! $SUDO ip link set "$iface" up type can bitrate 1000000 restart-ms 100 2>/dev/null; then
        $SUDO ip link set "$iface" up type can bitrate 1000000
        echo "  ($iface: adapter does not support restart-ms; brought up without it)"
    fi
    if ! ip link show "$iface" 2>/dev/null | grep -q "UP"; then
        echo "  WARNING: $iface did NOT come up." >&2
    fi
}

# Get all CAN interfaces
can_interfaces=$(ip link show | grep -oP '(?<=: )(can\w+)')

# Check if any CAN interfaces were found
if [[ -z "$can_interfaces" ]]; then
    echo "No CAN interfaces found."
    exit 1
fi

# Reset each CAN interface
echo "Detected CAN interfaces: $can_interfaces"
for iface in $can_interfaces; do
    reset_can_interface "$iface"
done

echo "All CAN interfaces have been reset with bitrate 1000000."
