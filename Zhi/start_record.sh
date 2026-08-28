#!/usr/bin/env bash
# ============================================================
# start_record.sh  -  interactive recorder: HEAD + THIRD VIEW
#
#   ~/Desktop/Zhi/start_record.sh
#
# Opens a recording mode that stays up until you log out:
#     ENTER       start a take
#     ENTER or q  stop & save the take (Ctrl-C mid-take works too)
#     l           log out of recording mode, back to the terminal
#
# Cameras:
#     head : yambox usb port 0:3 (the head/ego cam) -> recorded on
#            the yambox, copied back automatically after each take
#     side : RealSense D435 on hermes usb 71:00.4 port 2 (third view)
#
# Each take -> next numbered folder in ~/Desktop/Zhi (head.mkv, side.mkv).
# Teleop is not needed and not touched. There is no separate stop
# script - everything is handled inside this mode.
# ============================================================
set -uo pipefail

ZHI="$HOME/Desktop/Zhi"
YAMBOX="yambox@192.168.1.9"
# one shared master connection so per-take ssh calls and status polls are cheap
SSH="ssh -n -o ConnectTimeout=4 -o ControlMaster=auto -o ControlPath=/tmp/zhi_ssh_ctl -o ControlPersist=120"

# ---------------- looks ----------------
if [ -t 1 ]; then
    RED=$'\033[31m'; GRN=$'\033[32m'; YEL=$'\033[33m'; CYN=$'\033[36m'
    BLD=$'\033[1m';  DIM=$'\033[2m';  RST=$'\033[0m'
else
    RED=; GRN=; YEL=; CYN=; BLD=; DIM=; RST=
fi
HR="${DIM}────────────────────────────────────────────────────────────${RST}"

fmt_clock() { printf '%02d:%02d:%02d' $(($1/3600)) $(($1%3600/60)) $(($1%60)); }
fsize()     { [ -s "$1" ] && numfmt --to=iec "$(stat -c %s "$1" 2>/dev/null)" 2>/dev/null || echo "0"; }
# discard keystrokes typed while nothing was listening, so a double-tapped
# ENTER can never silently start or stop the next take out of phase
drain_input() { while IFS= read -rsn1 -t 0.01; do :; done; }

# ---------------- cameras ----------------
resolve_head_cam() {  # head cam on the yambox, usb port 0:3
    timeout 10 $SSH "$YAMBOX" '
        base=/dev/v4l/by-path/pci-0000:c6:00.3-usb-0:3
        for iface in 1.0 1.3; do n=${base}:${iface}-video-index0; [ -e "$n" ] && { echo "$n"; exit 0; }; done
        exit 1' 2>/dev/null
}
resolve_side_cam() {  # local D435 colour node = the by-path entry advertising YUYV
    local n
    for n in /dev/v4l/by-path/pci-0000:71:00.4-usb-0:2:*-video-index*; do
        [ -e "$n" ] || continue
        v4l2-ctl -d "$n" --list-formats 2>/dev/null | grep -q "'YUYV'" && { echo "$n"; return 0; }
    done
    return 1
}

next_take() {
    local n=0 d b
    for d in "$ZHI"/[0-9]*; do
        [ -d "$d" ] || continue
        b=$(basename "$d")
        case "$b" in (*[!0-9]*) continue ;; esac
        [ "$b" -gt "$n" ] && n=$b
    done
    echo $((n + 1))
}

# ---------------- cleanup on Ctrl-C / quit ----------------
CUR_SPID=""; CUR_REMOTE=""; CUR_DEST=""
stop_current() {
    # SIGINT, never SIGKILL - ffmpeg must write the container header
    [ -n "$CUR_REMOTE" ] && timeout 10 $SSH "$YAMBOX" "pkill -INT -f '[f]fmpeg.*$CUR_REMOTE'" 2>/dev/null
    if [ -n "$CUR_SPID" ]; then
        kill -INT "$CUR_SPID" 2>/dev/null
        wait "$CUR_SPID" 2>/dev/null
        CUR_SPID=""
    fi
}
fetch_head() {  # copy the remote head recording into the take folder
    [ -n "$CUR_REMOTE" ] || return 0
    sleep 4   # let the remote ffmpeg finish writing after SIGINT
    scp -q -o ControlPath=/tmp/zhi_ssh_ctl "$YAMBOX:$CUR_REMOTE/head.mkv" "$CUR_DEST/" 2>/dev/null
    scp -q -o ControlPath=/tmp/zhi_ssh_ctl "$YAMBOX:$CUR_REMOTE/head.log" "$CUR_DEST/" 2>/dev/null
    timeout 10 $SSH "$YAMBOX" "rm -rf $CUR_REMOTE" 2>/dev/null
    CUR_REMOTE=""
}
on_interrupt() {
    if [ -n "$CUR_SPID$CUR_REMOTE" ]; then
        stop_current
        printf '\r\033[K   %sinterrupted - closing the take cleanly ...%s\n' "$YEL" "$RST"
        fetch_head
        printf '   take saved in %s\n' "$CUR_DEST"
    fi
    echo
    exit 130
}
trap on_interrupt INT TERM HUP

# ---------------- one take ----------------
STOP_REASON=""
run_take() {
    local n=$1 dest="$ZHI/$1" remote="/tmp/zhi_rec_$1"
    local sp t0 tick=0 key rc elapsed dot f dur sz hsize="-"
    mkdir -p "$dest"
    CUR_DEST="$dest"
    timeout 10 $SSH "$YAMBOX" "rm -rf $remote && mkdir -p $remote" 2>/dev/null

    # head on the yambox (setsid+nohup: survives the ssh client going away)
    timeout 10 $SSH "$YAMBOX" "setsid nohup ffmpeg -y -f v4l2 -input_format yuyv422 \
        -video_size 640x480 -framerate 30 -use_wallclock_as_timestamps 1 \
        -i $HEAD_CAM -c:v libx264 -preset ultrafast -crf 23 -pix_fmt yuv420p \
        $remote/head.mkv > $remote/head.log 2>&1 < /dev/null &" 2>/dev/null
    CUR_REMOTE=$remote

    # side (third view) locally, straight into the take folder
    ffmpeg -y -f v4l2 -input_format yuyv422 -video_size 640x480 -framerate 30 \
        -use_wallclock_as_timestamps 1 -i "$SIDE_CAM" \
        -c:v libx264 -preset ultrafast -crf 23 -pix_fmt yuv420p \
        "$dest/side.mkv" > "$dest/side.log" 2>&1 < /dev/null &
    sp=$!
    CUR_SPID=$sp

    sleep 2
    local head_ok side_ok
    head_ok=$(timeout 10 $SSH "$YAMBOX" "pgrep -f '[f]fmpeg.*$remote' >/dev/null && echo 1 || echo 0" 2>/dev/null)
    side_ok=$(kill -0 $sp 2>/dev/null && echo 1 || echo 0)
    if [ "${head_ok:-0}" != 1 ] || [ "$side_ok" != 1 ]; then
        printf '\r\033[K   %s✘ camera failed to start%s  (head=%s side=%s)\n' \
            "$RED" "$RST" "${head_ok:-0}" "$side_ok"
        [ "${head_ok:-0}" != 1 ] && { timeout 10 $SSH "$YAMBOX" "tail -c 2000 $remote/head.log 2>/dev/null" 2>/dev/null | tr '\r' '\n' | tail -3 | sed 's/^/     head: /'; }
        [ "$side_ok" != 1 ] && tr '\r' '\n' < "$dest/side.log" 2>/dev/null | tail -3 | sed 's/^/     side: /'
        stop_current; fetch_head
        STOP_REASON="error"
        return 1
    fi

    t0=$(date +%s)
    STOP_REASON=""
    # while a take is rolling, Ctrl-C stops THE TAKE, not the session
    CTRLC=0
    trap 'CTRLC=1' INT
    drain_input
    while :; do
        if IFS= read -rsn1 -t 1 key; then
            # ENTER or q: stop this take, stay in the session. l: log out.
            if [ -z "$key" ] || [ "$key" = q ] || [ "$key" = Q ]; then STOP_REASON="enter"; break
            elif [ "$key" = l ] || [ "$key" = L ]; then STOP_REASON="quit"; break
            fi
        else
            rc=$?
            [ "$CTRLC" = 1 ] && { STOP_REASON="ctrlc"; break; }
            [ $rc -le 128 ] && { STOP_REASON="eof"; break; }   # stdin closed
        fi
        tick=$((tick + 1))
        elapsed=$(( $(date +%s) - t0 ))
        if (( tick % 5 == 0 )); then   # remote head size every 5 s, via the shared master
            hsize=$(timeout 4 $SSH "$YAMBOX" "stat -c %s $remote/head.mkv 2>/dev/null" 2>/dev/null)
            hsize=$(numfmt --to=iec "${hsize:-0}" 2>/dev/null || echo '?')
        fi
        if (( tick % 2 )); then dot="${RED}●${RST}"; else dot="${DIM}●${RST}"; fi
        printf '\r\033[K   %s %s%sREC%s  %s%s%s   head %s   side %s   %sENTER/q = stop%s' \
            "$dot" "$BLD" "$RED" "$RST" "$BLD" "$(fmt_clock $elapsed)" "$RST" \
            "$hsize" "$(fsize "$dest/side.mkv")" "$DIM" "$RST"
    done

    trap on_interrupt INT   # at the prompt, Ctrl-C quits the session again
    stop_current
    printf '\r\033[K   %sfinishing - copying head footage from the yambox ...%s' "$DIM" "$RST"
    fetch_head
    printf '\r\033[K'
    [ "$STOP_REASON" = "ctrlc" ] && printf '   %snote:%s Ctrl-C stops the take - you are back at the prompt (q quits)\n' "$YEL" "$RST"

    printf '   %s✔ take %s saved%s  →  %s\n' "$GRN" "$n" "$RST" "$dest"
    for f in "$dest"/head.mkv "$dest"/side.mkv; do
        if [ -s "$f" ]; then
            dur=$(ffprobe -v error -show_entries format=duration -of csv=p=0 "$f" 2>/dev/null | cut -d. -f1)
            sz=$(du -h "$f" | cut -f1)
            printf '       %-10s %6s   %s\n' "$(basename "$f")" "$sz" "$(fmt_clock "${dur:-0}")"
        else
            printf '       %-10s %s%s%s\n' "$(basename "$f")" "$RED" "MISSING" "$RST"
        fi
    done
    CUR_DEST=""
    TAKES_DONE+=("$n")
    return 0
}

# ---------------- startup ----------------
printf '   %schecking the yambox ...%s' "$DIM" "$RST"
if ! timeout 8 $SSH "$YAMBOX" true 2>/dev/null; then
    printf '\r\033[K   %s✘ cannot reach the yambox (%s)%s\n' "$RED" "$YAMBOX" "$RST"
    echo   "     is it powered on? DHCP moves its IP - if it changed, update YAMBOX= in this script"
    exit 1
fi
printf '\r\033[K'

# anything holding a camera gets cleared here - previews on either host,
# and stray recordings from a previous session (asked about first)
timeout 10 $SSH "$YAMBOX" "pkill -f '[y]am_cam_server|[s]tream_cameras'" 2>/dev/null \
    && echo "   ${YEL}note:${RST} stopped the yambox camera preview server"
pkill -f '[y]am_cam_server|[s]tream_cameras|[l]ive_preview' 2>/dev/null \
    && echo "   ${YEL}note:${RST} stopped a local camera preview server"
if pgrep -f "[f]fmpeg.*$ZHI" > /dev/null 2>&1 \
   || timeout 10 $SSH "$YAMBOX" "pgrep -f '[f]fmpeg.*zhi_rec'" > /dev/null 2>&1; then
    printf '   %sA recording from a previous session is still running.%s Stop it? [y/N] ' "$YEL" "$RST"
    IFS= read -r a || exit 1
    case "${a,,}" in
        y|yes)
            pkill -INT -f "[f]fmpeg.*$ZHI" 2>/dev/null
            timeout 10 $SSH "$YAMBOX" "pkill -INT -f '[f]fmpeg.*zhi_rec'" 2>/dev/null
            sleep 4
            rm -f "$ZHI/.recording_state" "$ZHI/.recording_local_state"
            echo "   stopped."
            ;;
        *) echo "   leaving it alone - quit that session first, then rerun this."; exit 1 ;;
    esac
fi

HEAD_CAM=$(resolve_head_cam) || true
SIDE_CAM=$(resolve_side_cam) || true
if [ -z "${HEAD_CAM:-}" ] || [ -z "${SIDE_CAM:-}" ]; then
    [ -z "${HEAD_CAM:-}" ] && echo "   ${RED}✘ head camera not found on the yambox (usb port 0:3)${RST}"
    [ -z "${SIDE_CAM:-}" ] && echo "   ${RED}✘ third-view camera not found (D435, hermes usb 71:00.4 port 2)${RST}"
    echo "   yambox nodes:"; timeout 10 $SSH "$YAMBOX" 'ls /dev/v4l/by-path/ 2>/dev/null | grep -- -video-index0' 2>/dev/null | sed 's/^/     /'
    echo "   hermes nodes:"; ls /dev/v4l/by-path/ 2>/dev/null | grep -- -video-index0 | sed 's/^/     /'
    exit 1
fi
FREE=$(df -h --output=avail "$ZHI" 2>/dev/null | tail -1 | tr -d ' ')

printf '\n'
printf '   %s%sRECORDING MODE%s  %s(head + third view)%s\n' "$BLD" "$CYN" "$RST" "$DIM" "$RST"
printf '%s\n' "$HR"
printf '   head  %syambox 0:3%s  %s\n' "$GRN" "$RST" "${DIM}${HEAD_CAM}${RST}"
printf '   side  %sD435 local%s  %s\n' "$GRN" "$RST" "${DIM}${SIDE_CAM}${RST}"
printf '   640x480 @ 30fps, H.264  ·  %s free  ·  takes → %s\n' "${FREE:-?}" "$ZHI"
printf '%s\n' "$HR"
printf '   %sENTER%s start a take    %sENTER%s or %sq%s stop it    %sl%s log out\n' \
    "$BLD" "$RST" "$BLD" "$RST" "$BLD" "$RST" "$BLD" "$RST"

# ---------------- main loop: stays up until q / logout ----------------
TAKES_DONE=()
while :; do
    N=$(next_take)
    drain_input
    printf '\n   %s[take %s]%s  ENTER=record  l=logout › ' "$BLD" "$N" "$RST"
    IFS= read -r cmd || break
    case "${cmd,,}" in
        l|logout|quit|exit) break ;;
        q) printf '   %snothing is recording - ENTER records, l logs out%s\n' "$DIM" "$RST" ;;
        "")
            run_take "$N" || true
            case "$STOP_REASON" in quit|eof) break ;; esac
            ;;
        *)  printf '   %sjust press ENTER to record, or l to log out%s\n' "$DIM" "$RST" ;;
    esac
done

printf '\n%s\n' "$HR"
if [ ${#TAKES_DONE[@]} -gt 0 ]; then
    printf '   %s✔ session done%s - %d take(s): %s\n' "$GRN" "$RST" "${#TAKES_DONE[@]}" "${TAKES_DONE[*]}"
else
    printf '   %ssession done - no takes recorded%s\n' "$DIM" "$RST"
fi
printf '%s\n' "$HR"
