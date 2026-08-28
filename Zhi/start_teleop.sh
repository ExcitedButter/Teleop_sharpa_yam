#!/usr/bin/env bash
# ============================================================
# start_teleop.sh  -  interactive teleop session (no recording)
#
#   ~/Desktop/Zhi/start_teleop.sh            # left arm  (default)
#   ~/Desktop/Zhi/start_teleop.sh right      # right arm
#   ~/Desktop/Zhi/start_teleop.sh both       # both arms
#
# Brings the arms up, then stays in a session showing live health.
# Teleop runs for as long as the session is open:
#     l (or q) / Ctrl-C / logout  ->  clean shutdown of leaders,
#                     followers and the sharpa hand driver
# There is no separate stop script - logging out IS the stop.
#
# Wiring:
#   left   leader hermes can_leader_l  <->  follower yambox can1 : 11334
#   right  leader hermes can_leader_r  <->  follower yambox can0 : 11333
#
# Grippers (override with LEFT_GRIPPER= / RIGHT_GRIPPER=):
#   sharpa       - the arm's original gripper was replaced by a Sharpa Wave
#                  hand (Ethernet, not CAN). The follower runs no_gripper
#                  (6 DOF), the leader streams the teaching-handle value over
#                  UDP 127.0.0.1:59200, and this script auto-starts
#                  ~/sharpa-venv/sharpa_hand_driver.py to drive the hand.
#                  This is the RIGHT arm's default since the 2026-08 swap.
#                  If the hand isn't broadcasting yet, arm teleop still works;
#                  the driver waits and grabs the hand when it appears.
#   linear_4310  - the original i2rt gripper (still the left arm's default).
#   no_gripper   - arm only. The leader now trims its command to the
#                  follower's DOF count, so this pairs fine.
#
# The left follower's gripper (motor 7, linear_4310 on can1) has been dead;
# if it is still dead, bring-up fails on that motor unless
# LEFT_GRIPPER=no_gripper. Repairing motor 7 is the real fix.
# ============================================================
set -uo pipefail

ARMS="${1:-left}"
case "$ARMS" in
    left|right|both) ;;
    *) echo "usage: $(basename "$0") [left|right|both]   (default: left)"; exit 2 ;;
esac

YAMBOX="yambox@192.168.1.9"
YAMBOX_IP="192.168.1.9"
YAMBOX_PW="root"
HERMES_PW="yam"
I2RT="/home/yam/gck/i2rt"
PY="$I2RT/.venv/bin/python"
GELLO="$I2RT/examples/minimum_gello/minimum_gello.py"
GELLO_REMOTE="examples/minimum_gello/minimum_gello.py"
PING="$I2RT/i2rt/motor_config_tool/ping_motors.py"
PING_REMOTE="i2rt/motor_config_tool/ping_motors.py"
LOGDIR="$HOME/teleop_logs"
mkdir -p "$LOGDIR"

LEFT_GRIPPER="${LEFT_GRIPPER:-linear_4310}"
RIGHT_GRIPPER="${RIGHT_GRIPPER:-sharpa}"   # original gripper swapped for a Sharpa hand
SHARPA_UDP="127.0.0.1:59200"
SHARPA_DRIVER="/home/yam/sharpa-venv/sharpa_hand_driver.py"
SHARPA_PY="/home/yam/sharpa-venv/bin/python"

# ---------------- looks ----------------
if [ -t 1 ]; then
    RED=$'\033[31m'; GRN=$'\033[32m'; YEL=$'\033[33m'; CYN=$'\033[36m'
    BLD=$'\033[1m';  DIM=$'\033[2m';  RST=$'\033[0m'
else
    RED=; GRN=; YEL=; CYN=; BLD=; DIM=; RST=
fi
HR="${DIM}────────────────────────────────────────────────────────────${RST}"
fmt_clock() { printf '%02d:%02d:%02d' $(($1/3600)) $(($1%3600/60)) $(($1%60)); }

# "sharpa" is not a CAN gripper - the follower arm runs without one.
follower_gripper() { [ "$1" = "sharpa" ] && echo no_gripper || echo "$1"; }

# per-arm wiring:  leader-can  follower-can  port  gripper
cfg() {
    case "$1" in
        left)  echo "can_leader_l can1 11334 $LEFT_GRIPPER" ;;
        right) echo "can_leader_r can0 11333 $RIGHT_GRIPPER" ;;
    esac
}
[ "$ARMS" = "both" ] && SIDES="left right" || SIDES="$ARMS"

log() { echo "[$(date +%H:%M:%S)] $*"; }

# Kill only real python processes - a bare "pkill -f minimum_gello"
# also matches this script and would kill it mid-run.
kill_leader() {
    for P in $(ps -eo pid,args | awk -v c="$1" '$2 ~ /python/ && $0 ~ c {print $1}'); do
        kill "$P" 2>/dev/null
    done
}

# NOTE: every ssh here is wrapped in `timeout ... ssh -n`.
# An ssh session that launches a background process never closes its
# channel, so a bare `ssh host "cmd &"` hangs forever. The remote job is
# nohup'd, so killing the ssh client does not kill it.
SSH="timeout 30 ssh -n"

# ---------------- shutdown (was stop_teleop.sh) ----------------
shutdown_teleop() {
    echo
    log "shutting teleop down ..."
    for P in $(ps -eo pid,args | awk '$2 ~ /python/ && /minimum_gello/ {print $1}'); do
        log "stopping leader $P"; kill "$P" 2>/dev/null
    done
    # The sharpa hand driver homes the hand and disconnects on SIGINT (its
    # KeyboardInterrupt/finally path); plain SIGTERM would leave the hand
    # wherever it was.
    for P in $(ps -eo pid,args | awk '$2 ~ /python/ && /sharpa_hand_driver/ {print $1}'); do
        log "stopping sharpa hand driver $P"; kill -INT "$P" 2>/dev/null
    done
    timeout 30 ssh -n "$YAMBOX" 'for P in $(ps -eo pid,args | awk "\$2 ~ /python/ && /minimum_gello/ {print \$1}"); do echo "stopping follower $P"; kill $P 2>/dev/null; done' 2>/dev/null
    sleep 5
    log "hermes leaders left : $(ps -eo pid,args | awk '$2 ~ /python/ && /minimum_gello/' | wc -l)"
    timeout 25 ssh -n "$YAMBOX" 'echo "yambox followers left: $(ps -eo pid,args | awk "\$2 ~ /python/ && /minimum_gello/" | wc -l)"' 2>/dev/null
}
on_interrupt() { shutdown_teleop; echo; exit 130; }
trap on_interrupt INT TERM HUP

# ================= bring-up =================
log "starting teleop for: $SIDES"

log "stopping anything already running..."
kill_leader can_leader_r
kill_leader can_leader_l
$SSH "$YAMBOX" 'for P in $(ps -eo pid,args | awk "\$2 ~ /python/ && /minimum_gello/ {print \$1}"); do kill $P 2>/dev/null; done' 2>/dev/null
sleep 8

# A bus left ERROR-PASSIVE makes every arm look dead. Always reset.
log "resetting yambox CAN buses..."
timeout 60 ssh -n "$YAMBOX" "echo $YAMBOX_PW | sudo -S -p '' sh ~/i2rt/scripts/reset_all_can.sh >/dev/null 2>&1" 2>/dev/null
sleep 4

for s in $SIDES; do
    read -r LCAN _ _ _ <<< "$(cfg "$s")"
    if ! ip link show "$LCAN" 2>/dev/null | grep -q "UP"; then
        log "$LCAN was down - bringing it up"
        echo "$HERMES_PW" | sudo -S -p '' ip link set "$LCAN" up type can bitrate 1000000 2>/dev/null
        sleep 3
    fi
done

# ---- preflight. Without this a dead chain costs ~3.5 min of retries before
#      it admits defeat. Note motor 7 is EXPECTED to fail on a leader: the
#      teaching handle is an encoder, not a motor. Set SKIP_PRECHECK=1 to skip.
# A cold bus answers with nothing, or a partial set, for the first two or three
# sweeps and only then reports all six - the same wake-up the leader start loop
# absorbs with its warm-up ping. Sweep up to 4 times and keep the best answer;
# a single cold sweep reports a perfectly healthy arm as dead.
has_arm() { case "$1" in *1*2*3*4*5*6*) return 0 ;; *) return 1 ;; esac; }

online_local() {
    local r="" i
    for i in 1 2 3 4; do
        r=$(timeout 40 "$PY" "$PING" --channel "$1" 2>/dev/null | sed -n 's/^online motors: //p' | tail -1)
        has_arm "$r" && break
    done
    echo "$r"
}
online_remote() {
    local r="" i
    for i in 1 2 3 4; do
        r=$(timeout 60 ssh -n "$YAMBOX" "cd ~/i2rt && timeout 40 .venv/bin/python $PING_REMOTE --channel $1 2>/dev/null" 2>/dev/null | sed -n 's/^online motors: //p' | tail -1)
        has_arm "$r" && break
    done
    echo "$r"
}

if [ "${SKIP_PRECHECK:-0}" != "1" ]; then
    FAIL=0
    for s in $SIDES; do
        read -r LCAN FCAN _ _ <<< "$(cfg "$s")"
        L=$(online_local "$LCAN");  L=${L:-[]}
        F=$(online_remote "$FCAN"); F=${F:-[]}
        log "$s leader   $LCAN online: $L"
        log "$s follower $FCAN online: $F"
        # leader needs arm motors 1-6; the handle is not a motor.
        if ! has_arm "$L"; then
            echo "ERROR: $s leader ($LCAN) arm motors are not responding."
            echo "       The teacher arm has a loose power connector - check that it"
            echo "       is powered and its CAN cable is seated, then re-run."
            FAIL=1
        fi
        if ! has_arm "$F"; then
            echo "ERROR: $s follower ($FCAN) arm motors are not responding."
            FAIL=1
        fi
        read -r _ _ _ GRIP <<< "$(cfg "$s")"
        if [ "$GRIP" != "no_gripper" ] && [ "$GRIP" != "sharpa" ]; then
            case "$F" in *7*) ;; *)
                echo "WARNING: $s follower gripper (motor 7) is NOT responding."
                echo "         Bring-up with --gripper $GRIP will fail on that motor."
                echo "         Repair it, or retry with ${s^^}_GRIPPER=no_gripper." ;;
            esac
        fi
    done
    [ "$FAIL" = "0" ] || { echo; echo "aborting - fix the above and re-run (SKIP_PRECHECK=1 to override)."; exit 1; }
fi

# ---- followers (yambox arms are yam_ultra + linear_4310) ----
start_follower() {  # $1=can channel  $2=port  $3=gripper
    timeout 25 ssh -n "$YAMBOX" "cd ~/i2rt && setsid nohup .venv/bin/python $GELLO_REMOTE \
        --mode follower --can-channel $1 --arm yam_ultra --gripper $3 \
        --bilateral-kp 0.2 --server-port $2 > /tmp/follower_$1.log 2>&1 < /dev/null &" 2>/dev/null
    return 0   # timeout killing the ssh client is expected, not an error
}

for s in $SIDES; do
    read -r _ FCAN PORT GRIP <<< "$(cfg "$s")"
    FGRIP=$(follower_gripper "$GRIP")
    log "starting follower $FCAN -> $PORT ($s, gripper $FGRIP)"
    start_follower "$FCAN" "$PORT" "$FGRIP"
done

# ---- sharpa hand driver (host side). Waits for the hand's UDP heartbeat, so
#      it is safe to start even while the hand is dark - arm teleop is
#      unaffected and the hand springs to life once it broadcasts.
NEED_SHARPA=0
for s in $SIDES; do
    read -r _ _ _ GRIP <<< "$(cfg "$s")"
    [ "$GRIP" = "sharpa" ] && NEED_SHARPA=1
done
if [ "$NEED_SHARPA" = "1" ]; then
    if pgrep -f sharpa_hand_driver >/dev/null 2>&1; then
        log "sharpa hand driver already running"
    else
        log "starting sharpa hand driver (udp $SHARPA_UDP, log $LOGDIR/sharpa_driver.log)"
        (cd /opt/sharpa-wave-sdk && setsid nohup "$SHARPA_PY" "$SHARPA_DRIVER" \
            --listen "$SHARPA_UDP" > "$LOGDIR/sharpa_driver.log" 2>&1 < /dev/null &)
    fi
fi
sleep 30

for s in $SIDES; do
    read -r _ _ PORT _ <<< "$(cfg "$s")"
    P=$(timeout 25 ssh -n "$YAMBOX" "ss -tln | grep -c $PORT" 2>/dev/null)
    [ "${P:-0}" = "1" ] || { echo "ERROR: $s follower port $PORT not up (see yambox /tmp/follower_*.log)"; exit 1; }
    log "$s follower listening on $PORT"
done

# ---- leader. The warm-up ping absorbs the cold-bus failure - without it,
#      bring-up (which needs all 6 motors in one pass) fails repeatedly.
start_leader() {  # $1=chan $2=port $3=label $4=extra args (e.g. --gripper-udp)
    local chan="$1" port="$2" label="$3" extra="${4:-}" a
    for a in 1 2 3 4 5; do
        kill_leader "$chan"; sleep 5
        timeout 25 "$PY" "$PING" --channel "$chan" >/dev/null 2>&1
        # shellcheck disable=SC2086  # $extra is deliberately word-split
        nohup "$PY" -u "$GELLO" --mode leader --can-channel "$chan" --arm yam \
            --gripper yam_teaching_handle --bilateral-kp 0.2 \
            --server-host "$YAMBOX_IP" --server-port "$port" $extra \
            > "$LOGDIR/leader_$label.log" 2>&1 &
        disown
        sleep 36
        if [ "$(grep -c 'Current follower joint pos' "$LOGDIR/leader_$label.log")" -ge 1 ]; then
            log "$label leader UP (attempt $a)"; return 0
        fi
        log "$label attempt $a failed: $(grep -oE 'fail to communicate with the motor [0-9]+|No encoders found' "$LOGDIR/leader_$label.log" | tail -1)"
    done
    return 1
}

for s in $SIDES; do
    read -r LCAN _ PORT GRIP <<< "$(cfg "$s")"
    EXTRA=""
    [ "$GRIP" = "sharpa" ] && EXTRA="--gripper-udp $SHARPA_UDP"
    start_leader "$LCAN" "$PORT" "$s" "$EXTRA" || { echo "ERROR: $s leader would not start"; exit 1; }
done

# ================= interactive session =================
# Health = the web-port io worker advancing in the leader log. "Process
# alive" is NOT health: the control loop keeps spinning at full rate
# against a dead chain. We compare io-line counts over a 6 s window.
declare -A IO_PREV IO_STATE
for s in $SIDES; do
    IO_PREV[$s]=$(grep -c 'web-port io' "$LOGDIR/leader_$s.log" 2>/dev/null || echo 0)
    IO_STATE[$s]="${DIM}checking${RST}"
done

printf '\n'
printf '   %s%sTELEOP SESSION%s  %s(%s)%s\n' "$BLD" "$CYN" "$RST" "$DIM" "$SIDES" "$RST"
printf '%s\n' "$HR"
for s in $SIDES; do
    read -r LCAN FCAN PORT GRIP <<< "$(cfg "$s")"
    printf '   %-6s %s%s ↔ yambox %s :%s%s   gripper %s\n' "$s" "$DIM" "$LCAN" "$FCAN" "$PORT" "$RST" "$GRIP"
done
printf '%s\n' "$HR"
printf '   Press the enable button (the [1,0] one, NOT the green-light\n'
printf '   one) on the handle to start driving.\n'
printf '   logs: %s/leader_*.log\n' "$LOGDIR"
printf '   %sl%s (or q) / Ctrl-C / logout = stop teleop and exit\n' "$BLD" "$RST"
printf '\n'

T0=$(date +%s)
TICK=0
STATUS_LINE=""
while :; do
    if IFS= read -rsn1 -t 2 key; then
        case "$key" in l|L|q|Q) break ;; esac
    else
        rc=$?
        [ $rc -le 128 ] && break   # stdin closed (logout / pipe gone)
    fi
    TICK=$((TICK + 1))
    if (( TICK % 3 == 0 )); then   # every ~6 s: did the io worker advance?
        for s in $SIDES; do
            c=$(grep -c 'web-port io' "$LOGDIR/leader_$s.log" 2>/dev/null || echo 0)
            if [ "$c" -gt "${IO_PREV[$s]}" ]; then
                IO_STATE[$s]="${GRN}LIVE${RST}"
            else
                IO_STATE[$s]="${RED}DEAD${RST}"
            fi
            IO_PREV[$s]=$c
        done
    fi
    if (( TICK % 2 )); then dot="${GRN}●${RST}"; else dot="${DIM}●${RST}"; fi
    STATUS_LINE=""
    for s in $SIDES; do STATUS_LINE+="   $s ${IO_STATE[$s]}"; done
    printf '\r\033[K   %s %s%sTELEOP%s  %s%s%s%s   %sl = stop & quit%s' \
        "$dot" "$BLD" "$GRN" "$RST" "$BLD" "$(fmt_clock $(( $(date +%s) - T0 )))" "$RST" \
        "$STATUS_LINE" "${DIM}" "$RST"
done
printf '\r\033[K'

shutdown_teleop

printf '\n%s\n' "$HR"
printf '   %s✔ teleop session ended%s - ran %s\n' "$GRN" "$RST" "$(fmt_clock $(( $(date +%s) - T0 )))"
printf '%s\n' "$HR"
