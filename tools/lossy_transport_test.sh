#!/usr/bin/env bash
# UDP vs TCP for SOFT_RTI under PACKET LOSS — the one place the two transports can differ.
#
# On a clean link they measure identically (confirmed: e2/e3 from a PC, UDP == TCP to 3 decimals),
# so a comparison that never injects loss cannot tell them apart. This injects loss on the RTI
# stream ports ONLY, on the robot's loopback, runs api_tests/external/e2_transport.py for each
# level, and ALWAYS removes the impairment on the way out.
#
#   sudo tools/lossy_transport_test.sh selftest              # apply 1% loss, show it, remove it
#   sudo tools/lossy_transport_test.sh run                   # levels 0 1 3 %, 1 ms delay each way
#   sudo tools/lossy_transport_test.sh run --levels "0 2"    # your own levels
#   sudo tools/lossy_transport_test.sh run --levels "3" --burst 20    # 3% average, in bursts of ~20 packets
#   sudo tools/lossy_transport_test.sh run --levels "3" --jitter 8    # +-8 ms delay variation
#   sudo tools/lossy_transport_test.sh status                # is anything applied right now?
#   sudo tools/lossy_transport_test.sh off                   # remove it (safe to run any time)
#   sudo tools/lossy_transport_test.sh run --levels "3 5 10" --recover   # auto Clear Errors + Enable between levels
#   tools/lossy_transport_test.sh run --dry-run              # print the commands, change nothing
#
# WHAT IT TOUCHES. `lo` only, and only packets whose source OR destination port is 30001
# (setpoints, UDP), 30002 (state, UDP) or 30003 (TCP, both directions). The executor (29000),
# the dashboard, SSH and ZMQ are not matched, so they are not degraded. It refuses to start if
# `lo` already has a non-default qdisc, so it cannot clobber someone else's setup.
#
# WHAT TO EXPECT — and what to be ready for. SOFT_RTI's watchdog is 3 command intervals
# (60 ms at 50 Hz). A lost UDP setpoint leaves a 40 ms gap: survivable. A lost TCP segment stalls
# every command behind it until retransmission; whether that fits in 60 ms is exactly what this
# measures. If it does not, the run prints "CONTROL LOST": RTI stops driving and holds. That is
# the design working, not a fault — but the arm is then in an error hold, so press Clear Errors
# on the dashboard before the next run. The arm moves J4/J5/J6 only, 15 deg/s, +-15 deg.
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEV=lo
PORTS_ANY="30001 30002 30003"     # matched as destination port
PORT_SRC=30003                    # TCP replies come FROM the robot's stream port
LEVELS="0 1 3"
DELAY_MS=1
JITTER_MS=0
BURST=0
RECOVER=0
DRY=0
CMD="${1:-}"; shift || true

while [ $# -gt 0 ]; do
  case "$1" in
    --levels)  LEVELS="$2"; shift 2 ;;
    --delay)   DELAY_MS="$2"; shift 2 ;;
    --jitter)  JITTER_MS="$2"; shift 2 ;;      # +-ms of random delay variation (Wi-Fi-like)
    --burst)   BURST="$2"; shift 2 ;;          # mean burst length in PACKETS (Gilbert-Elliott model)
    --corr)    echo "--corr was REMOVED: netem's correlation does not make bursts, it drives the loss rate" >&2
               echo "towards zero (3% with 50% correlation measured 0.000% loss). Use --burst N." >&2; exit 2 ;;
    --recover) RECOVER=1; shift ;;             # clear errors + ENABLE the arm before each level
    --dry-run) DRY=1; shift ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done

run() { if [ "$DRY" = 1 ]; then echo "  [dry-run] $*"; else "$@"; fi; }

need_root() {
  if [ "$DRY" = 0 ] && [ "$(id -u)" -ne 0 ]; then
    echo "needs root for tc: run with sudo (or add --dry-run to just print the commands)" >&2
    exit 2
  fi
}

is_default_qdisc() {
  # A pristine loopback is exactly:  qdisc noqueue 0: root refcnt 2
  local out; out="$(tc qdisc show dev "$DEV" 2>/dev/null)"
  [ "$(printf '%s\n' "$out" | grep -c .)" -le 1 ] && printf '%s' "$out" | grep -q 'noqueue'
}

clear_impairment() {
  [ "$DRY" = 1 ] && { echo "  [dry-run] tc qdisc del dev $DEV root"; return 0; }
  tc qdisc del dev "$DEV" root 2>/dev/null || true
}

apply_impairment() {   # $1 = loss percent
  local loss="$1"
  run tc qdisc add dev "$DEV" root handle 1: prio || return 1
  local dly="${DELAY_MS}ms"
  [ "$JITTER_MS" != "0" ] && dly="${DELAY_MS}ms ${JITTER_MS}ms"
  local lss="${loss}%"
  if [ "$BURST" != "0" ] && [ "$loss" != "0" ]; then
    # Gilbert-Elliott: a GOOD state (no loss) and a BAD state (every packet lost). The chain leaves
    # BAD with probability r per packet, so bursts average 1/r packets; enters BAD with probability
    # p, and the long-run loss is p/(p+r). Solve for p and r from the average loss and burst length.
    # NOT netem's `loss P% C%`: that "correlation" smooths its random generator, which pushes the
    # loss rate to ~0 (3%/50% measured 0.000%) instead of clustering it.
    local r p
    r="$(awk -v b="$BURST" 'BEGIN{printf "%.5f", 100.0 / b}')"
    p="$(awk -v l="$loss" -v r="$r" 'BEGIN{printf "%.5f", r * l / (100.0 - l)}')"
    lss="gemodel ${p}% ${r}%"
  fi
  # shellcheck disable=SC2086
  run tc qdisc add dev "$DEV" parent 1:3 handle 30: netem delay $dly loss $lss || return 1
  local p
  for p in $PORTS_ANY; do
    run tc filter add dev "$DEV" protocol ip parent 1:0 prio 1 u32 match ip dport "$p" 0xffff flowid 1:3 || return 1
  done
  run tc filter add dev "$DEV" protocol ip parent 1:0 prio 1 u32 match ip sport "$PORT_SRC" 0xffff flowid 1:3 || return 1
  if [ "$DRY" = 0 ]; then
    local n; n="$(tc filter show dev "$DEV" 2>/dev/null | grep -c 'flowid 1:3')"
    if [ "$n" -lt 4 ]; then echo "expected 4 filters on $DEV, found $n — aborting" >&2; return 1; fi
  fi
}

show_state() {
  echo "--- $DEV qdisc:"; tc qdisc show dev "$DEV" 2>&1 | sed 's/^/    /'
  echo "--- $DEV filters:"; tc filter show dev "$DEV" 2>&1 | grep -E "flowid|dport|sport|match" | sed 's/^/    /' | head -12
}

# Always leave the box clean, however this exits (Ctrl-C included).
cleanup() {
  local rc=$?
  trap - EXIT INT TERM
  if [ "$DRY" = 0 ] && [ "$(id -u)" -eq 0 ] && [ "${APPLIED:-0}" = 1 ]; then
    clear_impairment
    echo
    echo "impairment removed. $DEV now: $(tc qdisc show dev "$DEV" 2>/dev/null | tr '\n' ' ')"
  fi
  exit "$rc"
}
trap cleanup EXIT INT TERM

case "$CMD" in
  status)
    show_state ;;
  off)
    need_root
    clear_impairment
    echo "removed."; show_state ;;
  selftest)
    need_root
    if [ "$DRY" = 0 ] && ! is_default_qdisc; then
      echo "$DEV already has a non-default qdisc — refusing (run 'off' if it is ours):" >&2
      show_state; exit 1
    fi
    APPLIED=1
    apply_impairment 1 || { echo "FAILED to apply — see above" >&2; exit 1; }
    show_state
    echo
    echo "selftest OK: 1% loss + ${DELAY_MS} ms delay applied to ports $PORTS_ANY, removing now." ;;
  run)
    need_root
    if [ "$DRY" = 0 ] && ! is_default_qdisc; then
      echo "$DEV already has a non-default qdisc — refusing (run 'off' if it is ours):" >&2
      show_state; exit 1
    fi
    RUNAS="${SUDO_USER:-}"
    if [ "$DRY" = 0 ] && [ -z "$RUNAS" ]; then
      echo "run this with sudo from your own account (SUDO_USER is empty), so e2 runs as you, not root" >&2
      exit 2
    fi
    cd "$ROOT" || exit 1
    echo "levels: $LEVELS  (% loss, applied to BOTH directions of the RTI streams, +${DELAY_MS} ms each way, jitter ${JITTER_MS} ms, burst ${BURST} pkts)"
    echo "arm moves J4/J5/J6. If a run says CONTROL LOST, press Clear Errors on the dashboard first."
    for L in $LEVELS; do
      echo
      echo "================================================================================"
      echo "  LOSS ${L}%   (delay ${DELAY_MS} ms +-${JITTER_MS}, mean burst ${BURST} pkts)"
      echo "================================================================================"
      if [ "$RECOVER" = 1 ]; then
        # A previous level may have ended CONTROL LOST: the robot is then DISABLED in an error hold and
        # the next level cannot start. Recover it BEFORE applying loss, so the recovery itself runs on a
        # clean link. This ENABLES the motors — hence opt-in.
        echo "  recovering the arm (clear errors + enable) ..."
        if [ "$DRY" = 1 ]; then
          echo "  [dry-run] sudo -u <you> python3 tools/recover_robot.py"
        elif ! sudo -u "$RUNAS" -H env "PATH=$PATH" python3 tools/recover_robot.py; then
          echo "the arm could not be recovered (see above) — stopping" >&2
          break
        fi
      fi
      clear_impairment
      APPLIED=1
      apply_impairment "$L" || { echo "FAILED to apply ${L}% — stopping" >&2; exit 1; }
      if [ "$DRY" = 1 ]; then
        echo "  [dry-run] sudo -u <you> python3 api_tests/external/e2_transport.py --host 127.0.0.1 --go --transports udp,tcp"
      else
        E2ARGS=""
        [ "$RECOVER" = 1 ] && E2ARGS="--recover"   # also recover BETWEEN the two transports of a level
        # shellcheck disable=SC2086
        sudo -u "$RUNAS" -H env "PATH=$PATH" python3 api_tests/external/e2_transport.py \
            --host 127.0.0.1 --go --transports udp,tcp $E2ARGS
        rc=$?
        if [ "$rc" -eq 3 ]; then
          echo
          echo "e2 could not start (see ABORT above: usually the dashboard holds control for 30 s after"
          echo "a click, or the robot is not ENABLED). Stopping instead of repeating it for every level."
          break
        fi
      fi
      clear_impairment
      APPLIED=0
      # let the robot settle and any error latch clear before the next level
      [ "$DRY" = 1 ] || sleep 3
    done ;;
  *)
    sed -n '2,32p' "${BASH_SOURCE[0]}"
    exit 2 ;;
esac
