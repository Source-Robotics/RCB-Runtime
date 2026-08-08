"""Guided physical-failure harness for the CAN firmware flasher.

Runs a real flash, tells you EXACTLY when to pull power (or the CAN cable), then
diagnoses what the board did and whether it can be recovered. Exists because the
interesting failures are physical and have to be triggered by hand at a precise
moment, and because eyeballing log lines gives results you cannot compare
between runs.

BENCH RIG ONLY. Every phase here deliberately damages a flash in progress.

    # power cut mid-erase (~3.5 s window -- the widest, easiest to hit)
    python3 tools/flash_failure_test.py --node 5 --file fw.bin --phase erase

    # power cut mid-page-write
    python3 tools/flash_failure_test.py --node 5 --file fw.bin --phase pages

    # CAN cable pulled mid-flash (board keeps power -- different recovery path)
    python3 tools/flash_failure_test.py --node 5 --file fw.bin --phase cable

    # no interruption -- baseline, proves the harness itself is honest
    python3 tools/flash_failure_test.py --node 5 --file fw.bin --phase none

## Why there is no `commit` phase

The genuinely interesting window is the bootloader writing its OWN config page
(board id, app page count, app CRC) during WRITE_CRC. Killing the HOST there is
harmless -- proven 2026-08-04 -- because once WRITE_CRC is sent the bootloader
completes the write independently. Only cutting the BOARD's power can interrupt
it.

That window is a single STM32 page erase-and-write: roughly 20-50 ms, about 0.2%
of a 24 s flash. It is NOT hand-hittable; an earlier version of this harness
offered a `commit` phase and the banner fired simultaneously with completion.
Testing it needs power switching the Pi can control, so the cut can be triggered
programmatically the moment WRITE_CRC goes out. Add that here if the jig ever
gets a relay/MOSFET on board power.

What we can say without the test, from the memory map alone:

  - The bootloader's CODE (0x08000000-0x080022FF) is never written during an app
    flash, so no power cut can damage it.
  - Only the config page (0x08002C00) is at risk.
  - Lost app CRC   -> board waits in its bootloader -> reflash. Recoverable.
  - Lost board id  -> comes up on a different id, but PING is answered by EVERY
    board regardless of the id addressed, so it can always be discovered and
    then `set-id`. Recoverable.

i.e. the worst case is recoverable over CAN without an SWD probe, and that
follows from the layout rather than from luck.
"""

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import flash_motor_firmware as F

CLI = Path(__file__).resolve().parent / "flash_motor_firmware.py"

# progress stage -> when to shout. See flash_motor_firmware.flash_firmware().
# No "commit"/"finalising" entry on purpose — that window is ~20-50 ms and cannot
# be hit by hand. See the module docstring.
PHASE_STAGE = {
    "erase":  "erasing",
    "pages":  "flashing",
    "cable":  "flashing",
}


def banner(text, char="!"):
    line = char * 74
    print(f"\n{line}\n{text}\n{line}\n", flush=True)


def quiet_settle(seconds=18.0):
    """Wait with the bus COMPLETELY SILENT, then let the caller survey.

    Deliberately not a poll. The bootloader only hands over to the application
    after ~3 s of inactivity, and every probe resets that timer — polling holds
    the board in its bootloader indefinitely and then catches it mid-boot,
    reporting a healthy board as missing. Silence is what lets it settle:

      - flash succeeded -> BL times out, boots the app  -> survey finds the app
      - flash interrupted -> app CRC invalid, stays put -> survey finds the BL

    Both are correct end states; neither can be observed while we keep talking.
    """
    print(f"  letting the bus go silent for {seconds:.0f}s so the board can settle ...",
          end="", flush=True)
    time.sleep(seconds)
    print(" done.")


def survey(label):
    """Who is on the bus, in the bootloader, and is the link healthy?"""
    print(f"--- {label} ---")
    try:
        bus = F.open_bus()
    except Exception as exc:
        print(f"  could not open can0: {exc}")
        return None
    try:
        apps = {}
        for node in range(16):
            info = F.read_device_info(bus, node, timeout_s=0.3)
            if info:
                apps[node] = info["software_version"]
        F.flush_rx(bus)
        bls = F.scan_bootloader_boards(bus, reset_apps=False, listen_s=1.5)
    finally:
        F.close_bus(bus)
    print(f"  app nodes      : {apps if apps else 'none'}")
    print(f"  in bootloader  : {bls if bls else 'none'}")
    return {"apps": apps, "bls": bls}


def run_flash(node, firmware, phase, channel):
    """Run the real flasher, prompting at `phase`. Returns its exit code."""
    target = PHASE_STAGE.get(phase)
    proc = subprocess.Popen(
        [sys.executable, "-u", str(CLI), "flash", "--node", str(node),
         "--file", str(firmware), "--channel", channel, "--progress-json"],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    shouted = False
    try:
        for raw in proc.stdout:
            raw = raw.strip()
            if not raw:
                continue
            try:
                msg = json.loads(raw)
            except ValueError:
                continue
            if msg.get("type") == "progress":
                stage = msg.get("stage")
                page, total = msg.get("page"), msg.get("total_pages")
                detail = f" page {page + 1}/{total}" if page is not None and total else ""
                print(f"  [{stage}]{detail}", flush=True)
                if target and stage == target and not shouted:
                    shouted = True
                    what = ("CUT POWER TO THE BOARD NOW"
                            if phase != "cable" else "PULL THE CAN CABLE NOW")
                    banner(f"  >>> {what} <<<\n"
                           f"      (phase '{phase}' reached)")
            elif msg.get("type") == "result":
                print(f"  result: ok={msg.get('ok')} — {msg.get('message')}", flush=True)
        proc.wait()
    finally:
        if proc.poll() is None:
            proc.kill()
    if target and not shouted:
        print(f"  NOTE: phase '{phase}' was never reached — the flash failed earlier.")
    return proc.returncode


def main():
    ap = argparse.ArgumentParser(
        description="Guided physical-failure tests for the CAN flasher. BENCH RIG ONLY.")
    ap.add_argument("--node", type=int, required=True)
    ap.add_argument("--file", required=True)
    ap.add_argument("--phase", choices=["none", "erase", "pages", "cable"],
                    default="erase",
                    help="when to prompt you to cut power. 'erase' has the widest "
                         "window (~3.5 s) and is the easiest to hit; 'cable' prompts "
                         "during page writes but asks for the CAN cable instead, so "
                         "the board keeps power. There is deliberately no 'commit' "
                         "phase — see the module docstring.")
    ap.add_argument("--channel", default="can0")
    args = ap.parse_args()

    path = Path(args.file)
    if not path.is_file():
        sys.exit(f"No such firmware file: {path}")

    ok, reason = F.bus_available()
    if not ok:
        sys.exit(reason)

    print(f"\nFAILURE TEST — node {args.node}, phase '{args.phase}'")
    print("Bench rig only. This deliberately interrupts a flash in progress.\n")
    before = survey("BEFORE")
    if before is None:
        sys.exit("Could not survey the bus.")
    if args.node not in before["apps"] and args.node not in before["bls"]:
        print(f"  WARNING: node {args.node} is not answering as an app or a bootloader.")

    if args.phase != "none":
        input("Ready? Keep a hand on the power switch, then press Enter to start... ")

    code = run_flash(args.node, path, args.phase, args.channel)
    print(f"\n  flasher exit code: {code}")

    if args.phase != "none":
        banner("RESTORE POWER / RECONNECT THE CABLE NOW.\n"
               "Then press Enter (the harness waits for the board itself).", char="=")
        input()

    quiet_settle()
    after = survey("AFTER")
    if after is None:
        print("\n  Bus unreadable. If the link went bus-off, reset it with:")
        print("    sudo ip link set can0 down && sudo ip link set can0 up type can bitrate 1000000")
        return 1

    print("\n=== VERDICT ===")
    node = args.node
    if node in after["apps"]:
        print(f"  Node {node} is running its APPLICATION (sw={after['apps'][node]}).")
        print("  -> The board self-recovered. Nothing to do.")
    elif node in after["bls"]:
        print(f"  Node {node} is waiting in its BOOTLOADER, board id {node} INTACT.")
        print("  -> Expected, recoverable. Re-flash with:")
        print(f"     python3 tools/flash_motor_firmware.py flash --node {node} --file {path}")
    elif after["bls"]:
        print(f"  Node {node} is gone, but bootloader id(s) {after['bls']} answered.")
        print("  -> The board is alive but its CONFIG PAGE ID CHANGED. That is the")
        print("     damage this test was looking for. Recover with:")
        print(f"     python3 tools/flash_motor_firmware.py set-id --node {after['bls'][0]} --new-id {node}")
    else:
        lost = set(before["apps"]) - set(after["apps"])
        print(f"  Node {node} answers as neither app nor bootloader. Missing: {sorted(lost)}")
        print("  -> Power-cycle the board and retry the survey. If it stays silent,")
        print("     catch its cold-boot window:")
        print("     python3 tools/flash_motor_firmware.py scan --catch-boot --listen 60")

    gone = set(before["apps"]) - set(after["apps"]) - {node}
    if gone:
        print(f"\n  WARNING: unrelated node(s) also stopped answering: {sorted(gone)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
