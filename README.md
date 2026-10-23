# RCB Control Runtime

RCB Control Runtime is the Raspberry Pi runtime for a Source Robotics Robot
Control Box. It starts and supervises the real-time control loop, CAN motor
communication, command executor, dashboard, OLED/buttons, and control-board IO.

The runtime is designed for Raspberry Pi 5 + Ubuntu RT and can be configured for
multiple robot arms through XML files in `config/`, `robots/`, and `grippers/`.

> Safety: this software can move real robot hardware. Do the first startup with
> motors disabled or the robot mechanically safe, and verify E-stop, power, CAN,
> homing, and joint limits before enabling motion.

## Set up a new Pi — the checklist

The order that reproduces the reference robot (2026-10-16). Each step names the section that explains it;
`scripts/setup_system.sh` does the system-level part and checks all of it.

1. **Flash the real-time image** and check it — *OS image*.
2. **Boot settings**: CAN, SPI, UART and I2C in `/boot/firmware/config.txt`, no serial console on the UART, then
   reboot — *Boot configuration*, *Free the UART from the console*.
3. **Python 3.11 as `python3`** (Ubuntu 24.04 ships 3.12; 3.11 comes from the deadsnakes PPA) — *Python 3.11, not 3.12*.
4. **Clone the repo** to `~/Desktop/RCB-Runtime` (any folder works; the service is rendered for it).
5. **Python packages**: `./scripts/install.sh` — the repo's `.venv` from `requirements.txt`; the robot AND the
   tests run it — *Python packages: which Python runs the robot*.
6. **The system part**: `sudo bash scripts/setup_system.sh` — user groups, the password-free sudo rule, the
   real-time capability on `python3`, `autorun.service` (+ `LimitMEMLOCK=infinity`), the journal / syslog limits.
   Add `--allowed-ips "IP,IP"` for the PCs that may send commands (`*` = any: isolated networks only).
7. **Check everything**: `bash scripts/setup_system.sh --check` (exit 0 = nothing missing; run it with the arm
   disabled — it starts a Python that imports the runtime's packages).
8. **Reboot**, then read `logs/RTI.log` for `real-time setup OK`, `memory locked`, `interrupt threads ... on core 2
   alone` and the self-check; home the arm.
9. **Local material**: `docs/`, `TODO/` and the test nets in `dummy_test_scripts/` are NOT in git (only this README
   and `docs/pin_map.md` are) — copy them from the reference Pi, and back them up now and then.

## Supported Target

First public target:

- Raspberry Pi 5
- Ubuntu 24.04 with a PREEMPT_RT kernel — see **OS image** below
- Python **3.11** (see the note below; 24.04's default is 3.12)
- SocketCAN interface exposed as `can0` at 1 Mbit
- Mainboard/control PCB on `/dev/ttyAMA0`
- Optional front-panel SSD1306 OLED, buttons, and LEDs

### OS image

The runtime needs a real-time kernel. The reference robot runs this prebuilt image:

**https://github.com/ros-realtime/ros-realtime-rpi4-image/releases/tag/24.04.2_v6.8.4-rt11-raspi_ros2_jazzy**

Ubuntu 24.04.2 LTS with kernel `6.8.4-rt11-raspi`. Two things about it that are not
obvious from the name:

- **It says `rpi4` but it runs on a Raspberry Pi 5.** That is the exact image on the
  reference robot — a Pi 5 Model B, `uname -r` = `6.8.4-rt11-raspi`,
  `/sys/kernel/realtime` = `1`. Do not skip it thinking it is for the wrong board.
- **It bundles ROS 2 Jazzy, which this project does not use.** RCB Runtime has no ROS
  dependency at all; the image is used purely for its RT kernel. The ROS packages can be
  ignored or removed.

Verify after flashing:

```bash
uname -r                    # expect 6.8.4-rt11-raspi
cat /sys/kernel/realtime    # expect 1
```

`scripts/check_system.py` checks both, plus CAN, I2C, UART, GPIO and RT scheduling
permission.

### Boot configuration

**`can0`, `/dev/i2c-1` and `/dev/ttyAMA0` do not exist until you edit
`/boot/firmware/config.txt` and reboot.** Nothing in the runtime can create them, and
nothing points at that file when they are missing — you get a CAN init failure, or an
lgpio "GPIO busy", and no clue why.

`system_setup/boot_config.txt` is a reference copy of that file from the development
robot, with a header explaining which lines matter and why. Merge the lines you need
into your own config (do not overwrite it wholesale — the rest of the file is display
and audio settings for a specific Pi), then reboot.

The four that are not optional:

```
dtoverlay=mcp2515-can0,oscillator=16000000,interrupt=12   # creates can0
dtoverlay=spi0-1cs                                        # SPI for it, CS0 only
dtparam=i2c_arm=on                                        # OLED
enable_uart=1                                             # mainboard PCB link
```

`oscillator=` must match the crystal on your CAN board. 16 MHz here; 8 MHz boards are
common and give a silently wrong bitrate rather than an error.

### Free the UART from the console

`supervisor.py` owns `/dev/ttyAMA0` exclusively, at **256000 baud**, to talk to the
mainboard PCB. A stock Pi also puts the kernel console and a login prompt on that same
port at 115200. Nothing reports an error — the PCB just receives boot messages and getty
banners mixed into its protocol at the wrong baud, so the link reads as flaky hardware.

Two settings, neither of them in `config.txt`:

```bash
# 1. no serial console — remove console=serial0,115200 (or console=ttyAMA0,115200)
#    from /boot/firmware/cmdline.txt, leaving the rest of the line untouched.
#    That file must stay ONE line and has no comment syntax.
cat /proc/consoles                 # expect tty1 only

# 2. no login prompt on the port
sudo systemctl disable --now serial-getty@ttyAMA0.service
```

`system_setup/boot_cmdline.txt` is this robot's `cmdline.txt`, verbatim, for comparison.
Pi 5 does not create `/dev/serial0`, so the code opens `ttyAMA0` directly.

### Device permissions and sudo

Two more things the repo cannot give you. Both fail quietly rather than loudly.

**Group membership.** The runtime opens `/dev/ttyAMA0` (`root:dialout`), `/dev/i2c-1`
(`i2c`) and `/dev/gpiochip*` (`gpio`) as your normal user:

```bash
sudo usermod -aG dialout,i2c,gpio "$USER"   # then log out and back in
```

`scripts/check_system.py` reports these as **warnings, not failures**, so an install
without them still says it passed — and then the UART, OLED and GPIO silently do not work.

**Passwordless sudo for what the runtime runs.** Under systemd there is no tty, so every
`sudo` the runtime runs must be password-free or it fails:

- `start_can_interface()` (`hardware/can_hardware.py`): `sudo ip link set can0 ...`, `sudo ifconfig` — without it
  `can0` never comes up;
- `place_can_path()` (same file, at every RTI start since 2026-10-12): `sudo taskset -pc` on the CAN chip's interrupt
  threads (core 2 alone), `sudo tee .../power/control` (the SPI controller's power saving off) and
  `sudo systemctl set-property --runtime user.slice AllowedCPUs=...` (your sessions on cores 0-1) — without it RTI
  logs an ERROR and the CAN path shares cores with everything else (CAN freezes under load);
- the OLED menu's reboot / power off (`sudo -n`, does nothing without it).

The reference robot has blanket `NOPASSWD: ALL`. `scripts/setup_system.sh` checks each of those commands and, only if
one still asks for a password, writes this scoped rule to `/etc/sudoers.d/10-rcb-runtime` (validated with `visudo`,
then checked again):

```
USER ALL=(root) NOPASSWD: /usr/sbin/ip, /usr/sbin/ifconfig, /usr/sbin/reboot, /usr/sbin/poweroff
USER ALL=(root) NOPASSWD: /usr/bin/taskset ^-pc [0-9\,-]+ [0-9]+$
USER ALL=(root) NOPASSWD: /usr/bin/tee ^/sys/devices/[A-Za-z0-9._\:/-]+/power/control$
USER ALL=(root) NOPASSWD: /usr/bin/systemctl ^set-property --runtime user[.]slice AllowedCPUs\=[0-9\,-]+$
```

The arguments of the last three are regular expressions (sudo 1.9.10+): a plain `*` would let `tee` write any file.

**Real-time priority.** RTI asks for `SCHED_FIFO` 99 on core 3 and refuses to start without it
(`<realtime_required>` in `config/system.xml`). The reference robot grants it with a file capability on the
interpreter: `sudo setcap cap_sys_nice=eip $(readlink -f $(command -v python3))` (the setup script does it). Every
`python3` process may then raise its own priority; a side effect: the C library drops `TMPDIR` / `LD_*` from such a
process's environment (the test runner passes `RCB_TMPDIR` instead).

### Wiring and pin map

[docs/pin_map.md](docs/pin_map.md) maps every GPIO signal on the control box end to end:
the numbered terminal you wire into, the Pi header pin, the BCM number, and the constant
the code imports. Photo of the connector panel included.

Read it before wiring anything. Two things in there are not guessable: the top terminal row
runs **8 down to 1 left to right**, so terminal 1 is the rightmost, and each E-stop is a
**pair** of terminals rather than one.

### Python 3.11, not 3.12

Ubuntu 24.04 ships **Python 3.12** as the default `python3`. The reference robot runs
**3.11**, and every pinned version in `requirements.txt` was verified against it — the
`.venv` is built on `/usr/bin/python3.11`.

This is easy to miss and nothing warns you: `scripts/install.sh` builds its venv from
whatever `python3` resolves to, and its version gate is `>= 3.11`, so **3.12 passes
silently** and you end up with an environment nobody has tested.

```bash
sudo apt-get install -y python3.11 python3.11-venv python3.11-dev
sudo update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.11 1
sudo update-alternatives --set python3 /usr/bin/python3.11
python3 --version           # expect 3.11.x
```

Or leave the system default alone and point the installer at it explicitly:

```bash
PYTHON=/usr/bin/python3.11 ./scripts/install.sh
```

3.12 has not been tested. It will probably work, but you will be resolving a different
set of binary wheels than the pins were verified against — which matters here, since
several dependencies (pinocchio, roboticstoolbox, numpy) are ABI-sensitive.

Desktop or no-hardware use is currently for development only. The installer can
run without hardware, but `scripts/check_system.py` will report missing devices.

## Quick Install

> ⚠️ **NOT PROPERLY TESTED YET.** The install path below has not been validated on a
> clean machine. It is written from how the development robot was actually set up, not
> from a reproduced from-scratch install, so treat it as a starting point rather than a
> guaranteed recipe. Expect to fix things.
>
> One concrete reason for the warning: as of 2026-08-08 an audit found `pinokin` imported
> by `motion/cartesian_paths.py`, installed by hand into `.venv`, and **never listed in
> `requirements.txt`** — so a clean install would have produced a robot that could not plan
> Cartesian motion. That is now fixed, and `scripts/check_system.py` gained import checks
> for `ruckig`, `pinokin` and `toppra` (none of which it verified before, so it would have
> passed the broken install). Similar gaps may remain. If you hit one, the pin belongs in
> `requirements.txt` and the import check in `check_system.py`, in the same commit.

```bash
git clone https://github.com/Source-Robotics/RCB-Runtime.git
cd RCB-Runtime
./scripts/install.sh
./scripts/check_system.py
```

The installer creates `.venv`, installs `requirements.txt`, creates runtime
folders, and runs a system check. It asks before installing apt packages and does
not automatically enable or start a systemd service.

### Python packages: which Python runs the robot

**The robot runs on the repo's `.venv` — the same packages the tests run** (since 2026-10-16). `autorun.sh` starts
`<repo>/.venv/bin/python3 supervisor.py`, and the supervisor starts every runtime process with its own interpreter.
`.venv/bin/python3` is a link to the system's `/usr/bin/python3.11`, so the real-time capability set on that binary
applies (see *Real-time priority*); the venv takes no package from `~/.local`.

```bash
./scripts/install.sh                                  # .venv from requirements.txt (the robot, the tests, the tools)
bash scripts/setup_system.sh --check                  # imports the runtime's packages with .venv's python3
```

Before 2026-10-16 the scripts were started as plain `python3` — the system Python with `~/.local`, whose versions had
drifted from the pins (numpy 2.4.6 vs 2.3.5): the robot did not run what the tests ran. `autorun.sh` falls back to
the system `python3` only when the repo has no `.venv` (an update that came without one) and says so in the journal.

## Systemd Service

**The reference robot runs `autorun.service`** (in the repo since 2026-10-16: `systemd/autorun.service.in` and
`systemd/autorun.sh.in`; `scripts/setup_system.sh` installs them). It starts `<repo>/../autorun.sh` as the robot's
user, which starts `<repo>/.venv/bin/python3 supervisor.py` — and first, if an update was staged next to the repo (the folder
`<repo>_PENDING` and the flag file `ready_to_swap.txt`), swaps it in, keeping the old folder as `<repo>_BACKUP`.
Two drop-ins in `/etc/systemd/system/autorun.service.d/`: `memlock.conf` (`LimitMEMLOCK=infinity`: RTI locks its
memory, so a memory shortage cannot stall the loop) and, optionally, `allowed-ips.conf` (`RCB_ALLOWED_IPS`; the
reference robot sets it to `*` in a drop-in named `allow-any-ip.conf` — systemd reads drop-ins in name order, the last
one wins; the setup script names every one that sets it).

```bash
sudo systemctl restart autorun.service      # every process; RTI restarting clears homing — re-home
sudo systemctl stop autorun.service         # the arm goes limp ~0.1 s later (drive watchdog): support or park it
journalctl -u autorun.service -f
```

The older template below (`rcb-runtime.service`: `.venv` + `LimitRTPRIO` instead of the capability) has never been
validated on a clean machine. **Never enable both** — two supervisors would start; the setup script's check says so.

> ⚠️ **NOT PROPERLY TESTED YET** (the `rcb-runtime.service` template).

To install that template instead:

```bash
./scripts/install_service.sh
```

The service is rendered with your current repo path and user, so it does not
hardcode `/home/robot/Desktop`.

Useful commands:

```bash
sudo systemctl start rcb-runtime.service
sudo systemctl stop rcb-runtime.service
sudo systemctl restart rcb-runtime.service
sudo systemctl status rcb-runtime.service
journalctl -u rcb-runtime.service -f
```

## Manual Run

> Since 2026-10-12 the reference robot runs everything under the service: RTI confines interactive sessions to cores
> 0-1 at its start, so an RTI started from a shell cannot get its real-time core (it refuses to start). Restart pieces
> with the service, or `tools/restart_gui_and_executor.sh` for the executor and the dashboard. The by-hand setup below
> is how development was done before; it still works for the executor and the dashboard.

Earlier, the runtime processes were started **by hand**, each in its own terminal, over **VS Code Remote-SSH**:

```bash
source .venv/bin/activate

python3 RTI.py               # terminal 1 — real-time loop, owns CAN + shared memory
python3 command_executor.py  # terminal 2 — TCP command hub + trajectory planner
python3 nicegui_test.py      # terminal 3 — dashboard on :8081
```

Why by hand rather than through the supervisor or the service:

- **You see each process's own output.** A managed child's stderr goes to a pipe, and a
  crash in one becomes silence rather than a traceback — which has cost real debugging time.
  Run the one you are working on standalone and the actual error is on screen.
- **They restart independently.** Planner-side edits need only `command_executor`;
  GUI edits only `nicegui_test.py`. **Restarting `RTI.py` CLEARS HOMING**, so you re-home
  after touching RTI-side code and not otherwise.
- **Order matters:** `RTI.py` first (it creates the shared-memory segments the other two
  attach to), then the executor and GUI in any order.

If the systemd service is enabled it will also be starting these, so stop it first:

```bash
sudo systemctl stop autorun.service    # or rcb-runtime.service, whichever is installed
```

## Run Under the Supervisor

The alternative to starting the three processes yourself: one command that launches and
monitors all of them, plus the front-panel hardware.

```bash
source .venv/bin/activate
python supervisor.py
```

Use this for an unattended or demo boot. For active development prefer the three-terminal
setup above — a managed child's stderr goes to a pipe, so a crash shows up as silence
instead of a traceback.

`supervisor.py` starts and monitors the managed runtime processes:

| Process | Role |
|---|---|
| `RTI.py` | Real-time control loop, CAN owner, shared-memory writer |
| `command_executor.py` | High-level command dispatch |
| `nicegui_test.py` | NiceGUI dashboard |
| `self_collision_worker.py` | Checks the measured pose against the arm's collision pieces (~100 Hz) |
| `recorder.py` | Incident dumps and telemetry captures from RTI's shared-memory trace |
| `supervisor.py` | OLED/buttons/LEDs, UART PCB IO, process supervision |

`nicegui_test.py` is the operator dashboard (the earlier `gui.py` was removed on 2026-10-03; it was launched by nothing).

Only `RTI.py` is intended to request real-time scheduling and a pinned core. The
service grants permission for that child process but does not make every process
real-time. The core plan is `config/cpu_cores.py`: the real-time loop on core 3, the
CAN chip's interrupt threads alone on core 2 (RTI places them at every start), every
other process and interactive session on cores 0-1. Start and restart the runtime
with the service, not RTI.py from a shell: RTI confines interactive sessions to
cores 0-1, so an RTI started from one cannot get its real-time core.

## Configuration

Important files:

- `config/system.xml`: selects the active robot, gripper, and Kt source
- `robots/<name>.xml`: robot geometry, limits, gearing, torque constants, timing
- `grippers/<name>.xml`: gripper configuration
- `data_recipe/*.xml`: UDP telemetry payload definitions
- `config/supervisor_settings.json`: local runtime OLED/menu settings, ignored by git

Runtime output is written to `logs/` and transient files to `temp_data/`. These
folders are kept in the repo with `.gitkeep`, while generated contents are
ignored.

### Configuration file formats

Each XML configuration file may name the format it is written in with a
`schema` attribute on its root element, for example `<robot schema="1">`. A
file without the attribute is format 1. The formats this runtime reads are listed
in `config/schema_versions.py`, one number per kind of file: system, robot,
gripper, homing, cell, filters and recipe.

- **A file in a newer format is refused.** It was written for a newer runtime,
  and reading it with this one could misread a field. A system, robot, gripper or
  homing file stops the runtime at start with the reason in its log. A cell file
  is refused like a broken one. A filters file falls back to the working filters.
  A recipe is skipped.
- **A file in an older format is read**, with one warning per file in the log.
- The runtime's version is in `VERSION`. The dashboard's Versions card and
  `robot.get_versions()` show it, together with how many configuration files were
  read and which of them are in an older format.

### Client and protocol versions

The Python client in `robotclient/` has the runtime's version
(`robotclient.__version__`). Three protocols carry a version number each: the
command protocol to the executor, the real-time command packet, and the telemetry
packet format. A number changes only when its protocol changes incompatibly. A new
command or a new field does not change it.

- `robot.connect(ip)` compares the command protocol's number with the robot's
  before it sends anything. A different number raises `VersionMismatch`. A robot
  that reports no number, from before version 0.9.0, gives a `VersionWarning` and
  the connection goes ahead. `allow_mismatch=True` skips the check.
- The real-time client compares the real-time packet and telemetry numbers the
  same way before it pairs with the robot.
- The robot logs each connected client's version, and `robot.get_versions()` lists
  the connected clients with their versions.
- Commands that carry pose angles are refused with `CLIENT_TOO_OLD` when the client
  declares no command protocol. Pose angles changed their meaning in 0.9.0, and an
  older client would turn the tool to another orientation. A client that sends raw
  JSON adds `"protocol": 1` to each message.

### Hardware versions

- **Drive firmware:** the runtime supports drive firmware 103 or newer, listed per
  driver type in `hardware/firmware_support.py`. A drive outside that range gets a
  warning in the log and on the Versions card. The first-run check fails it. The
  robot keeps running either way.
- **Hardware revisions:** the robot XML and each gripper XML can name their hardware
  revision with `<hardware_revision>`, for example `v1.0`. The Versions card and
  `robot.get_versions()` show it.
- **Description package:** the robot XML names the oldest description package its
  models are made from, with `<description_package min="2.0"/>`. The Versions card
  marks an older package, and the self-collision geometry tool refuses to use one.
- **Control box:** the control box board does not report its own version yet, so
  the PCB version stays declared in `config/system.xml`.

### Releases

What changed in each version is in [CHANGELOG.md](CHANGELOG.md). Which client,
protocol, firmware and hardware versions work with each release is in
[COMPATIBILITY.md](COMPATIBILITY.md). A release is a git tag `vMAJOR.MINOR.PATCH`.
Its GitHub release notes are its changelog section, printed by
`tools/release_notes.py`.

**Before you change anything, read [VERSIONING.md](VERSIONING.md).** It says which
version to change for which kind of change, and how a release is made, including
the documentation.

## Tools

Hardware debug utilities live in `tools/`. For example, `tools/debug_read_kt.py`
can query motor Kt values directly over CAN while the runtime is stopped.

`tools/flash_motor_firmware.py` updates motor-driver firmware over CAN via the
STM32F103 bootloader, with `tools/flash_motor_gui.py` as a web front-end on port
8083. Like the other bus-owning tools it refuses to run while RTI is alive — see
[docs/firmware_flashing.md](docs/firmware_flashing.md).

## Dependency Policy

Use `requirements.txt` for deployment. It contains the pinned versions verified
on the robot. `pyproject.toml` exists for project metadata and lightweight tooling;
it is not the primary deployment lock source.

Custom Source Robotics packages are installed from PyPI as part of
`requirements.txt`.

**If you add an import, add the pin and the import check in the same commit.** This has
been missed twice — `ruckig` (2026-07-12) and `pinokin` (2026-08-08) were both imported by
running code while present only as hand-installed packages, so `requirements.txt` described
an environment that could not actually run the robot. Both times the package was already
installed on the development machine, which is exactly why nobody noticed.

- pin it in `requirements.txt`
- add it to `check_imports()` in `scripts/check_system.py`, marked `True` if the robot
  cannot run without it

Check versions with `.venv/bin/python -m pip list`. A bare `python3` reads the system
environment, which carries different versions and is **not** what runs the robot.

## Security Model — read before exposing this to a network

> ⚠️ **There is no authentication.** Not weak authentication — none. Access control is an
> IP allowlist and nothing else: no keys, no HMAC, no session token, no TLS. Anyone who can
> reach the robot's ports from a spoofable source address can command the arm.

| service | port | binds | who may talk to it |
|---|---|---|---|
| command hub (TCP, JSON) | 29000 | `0.0.0.0` | source IP must be in `ALLOWED_IPS` |
| RTI command stream (UDP) | 30001 | `0.0.0.0` | source IP must match the paired PC |
| RTI state stream (UDP) | → 30002 | — | unicast to the paired PC |
| telemetry broadcast (UDP) | → 30003 | — | **the entire subnet**, unauthenticated |
| operator dashboard (HTTP) | 8081 | `0.0.0.0` | **anyone who can reach it** — no login, no allowlist |
| motor tuner / firmware flasher (HTTP, tools) | 8082 / 8083 | `0.0.0.0` | anyone, while the tool runs |

Consequences worth stating plainly:

- **UDP source addresses are trivially spoofed.** The RTI filter checks the sender's IP,
  which stops accidents and stops nothing else. An attacker on the same L2 segment can
  inject motion commands.
- **Telemetry is broadcast in the clear** to `255.255.255.255` by default — full robot
  state to every device on the network. Set `TELEMETRY_DESTINATION` in
  `config/networking.py` to your dashboard's IP to make it unicast (this also fixes the
  WiFi degradation documented there).
- **`ALLOWED_IPS` gates the TCP hub only.** The RTI UDP return path follows whatever
  address the PC's TCP socket reports, so it needs no entry.
- **The dashboard has no login.** Anyone who can open `http://<robot>:8081` can enable the
  arm, jog it, run programs and change settings — the allowlist does not apply to it. Its
  web libraries also carry published security advisories from time to time (the
  development check `tools/run_nets.py --audit` lists them); on an isolated network they
  are not reachable by an attacker.

**Run the robot on an isolated network — the recommendation for every installation**: the
robot, its control PC and nothing else (a dedicated switch, or a direct cable). Do not expose
ports 8081, 29000 or 30001 to a shared LAN, and never to the internet. If you need real
authentication, the natural place is an HMAC over the command packet plus a session key issued at
`rti_connect` — not built, and there is a note in `config/networking.py` where it would go.

`ALLOWED_IPS` ships with the development addresses. Add your own control PC without
editing tracked source:

```bash
RCB_ALLOWED_IPS=192.168.0.42 python3 command_executor.py
```

## Public Repo Notes

For the first public release, internal planning markdown files and manual dummy
test scripts are ignored by `.gitignore`. Keep user-facing setup information in
this `README.md` or in explicitly published docs added later.

## License

GPL-3.0, see `LICENSE`.

## Contact

Source Robotics - petar@source-robotics.com
