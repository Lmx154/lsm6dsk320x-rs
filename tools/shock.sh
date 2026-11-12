#!/usr/bin/env bash
# Runs the strike test of the high-g wake-up and shock generator and tells the person at the board
# what to do and when: a banner and a bell at each phase of the firmware (READY, STRIKE, STILL,
# DONE), a countdown during the strike window, then the verdict lines. The board's status LED shows
# the same phases (blue, green, red, off). The exit status is the run's: 0 pass, 1 fail.
#
# Usage: tools/shock.sh [window_seconds, default 30]
set -u
win="${1:-30}"
case "$win" in
'' | *[!0-9]* | 0)
    echo "usage: tools/shock.sh [window_seconds, default 30]" >&2
    exit 2
    ;;
esac
root="$(cd "$(dirname "$0")/.." && pwd)" || exit 1
cd "$root" || exit 1

tmp="$(mktemp -d)" || exit 1
marker="$tmp/start"
out="$tmp/run.out"
pid=""

cleanup() {
    if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
        kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null
    fi
    rm -rf "$tmp"
}
trap cleanup EXIT
trap 'echo; echo "interrupted"; exit 130' INT TERM

if [ -t 1 ]; then
    c_ready=$'\033[1;97;44m'
    c_strike=$'\033[1;30;42m'
    c_still=$'\033[1;97;41m'
    c_done=$'\033[1;30;47m'
    c_off=$'\033[0m'
else
    c_ready="" c_strike="" c_still="" c_done="" c_off=""
fi

# banner <colour> <text>
banner() {
    local pad="                                                            "
    local text=" $2"
    printf '\a\n%s%s%s\n%s%s%s\n%s%s%s\n' \
        "$1" "$pad" "$c_off" \
        "$1" "${text}${pad:${#text}}" "$c_off" \
        "$1" "$pad" "$c_off"
}

touch "$marker"
sleep 0.1
touch src/lib.rs hil/src/lib.rs hil/src/bin/lsm6dsk320x_shock.rs
export SHOCK_WINDOW_S="$win"

set -m
tools/run.sh lsm6dsk320x_shock "$((win + 120))" >"$out" 2>&1 &
pid=$!
set +m

echo "building and flashing the strike test (window ${win} s); the first banner comes when the board starts..."

log=""
seen=0
counting=0
strike_at=0
last_left=-1
events=0
phase_seen=""

show_counter() {
    local left=$((win - ($(date +%s) - strike_at)))
    [ "$left" -lt 0 ] && left=0
    local ev="$events"
    [ "$events" -ge 20 ] && ev="20+"
    printf '\r  STRIKE NOW - %3d s left - strikes registered: %s   ' "$left" "$ev"
}

handle() {
    case "$1" in
    *"PHASE READY:"*)
        phase_seen="$phase_seen R"
        banner "$c_ready" "GET READY - HANDS OFF"
        ;;
    *"PHASE STRIKE:"*)
        phase_seen="$phase_seen S"
        banner "$c_strike" "GO - STRIKE THE BOARD NOW"
        counting=1
        strike_at=$(date +%s)
        last_left=-1
        show_counter
        ;;
    *"PHASE STILL:"*)
        phase_seen="$phase_seen T"
        [ "$counting" -eq 1 ] && echo
        counting=0
        banner "$c_still" "STOP - PUT IT DOWN AND LEAVE IT STILL"
        ;;
    *"PHASE DONE"*)
        phase_seen="$phase_seen D"
        banner "$c_done" "FINISHED"
        ;;
    *"[INFO] event "*)
        events=$((events + 1))
        [ "$counting" -eq 1 ] && show_counter
        ;;
    esac
}

while :; do
    if [ -z "$log" ]; then
        log="$(find hil/logs -maxdepth 1 -name 'lsm6dsk320x_shock-*.log' -newer "$marker" 2>/dev/null | sort | tail -n 1)"
    fi
    if [ -n "$log" ]; then
        n="$(wc -l <"$log")"
        if [ "$n" -gt "$seen" ]; then
            while IFS= read -r line; do
                handle "$line"
            done < <(sed -n "$((seen + 1)),${n}p" "$log")
            seen="$n"
        fi
    fi
    if [ "$counting" -eq 1 ]; then
        left=$((win - ($(date +%s) - strike_at)))
        if [ "$left" -ne "$last_left" ]; then
            last_left="$left"
            show_counter
        fi
    fi
    kill -0 "$pid" 2>/dev/null || break
    sleep 0.2
done

wait "$pid"
status=$?
# Whatever was written between the last poll and the end of the run.
if [ -n "$log" ]; then
    n="$(wc -l <"$log")"
    if [ "$n" -gt "$seen" ]; then
        while IFS= read -r line; do
            handle "$line"
        done < <(sed -n "$((seen + 1)),${n}p" "$log")
    fi
fi
[ "$counting" -eq 1 ] && echo

echo
if [ -z "$log" ]; then
    echo "no log was written (build or flash failed):"
    tail -n 20 "$out"
    exit "$status"
fi
grep -E '\[(PASS|FAIL)\] (H[2-5]|no strike|End|health)|\[STAT\] (H3|summary)|RESULT' "$log" |
    sed 's/ (lsm6dsk320x_shock src.*//; s/ (hil_common hil.*//'
echo "phases seen:${phase_seen} (R S T D expected), strike events logged: ${events}"
echo "log: $log"
exit "$status"
