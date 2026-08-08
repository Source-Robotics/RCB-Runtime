# RCB Control Runtime

RCB Control Runtime is the Raspberry Pi runtime for a Source Robotics Robot
Control Box. It starts and supervises the real-time control loop, CAN motor
communication, command executor, dashboard, OLED/buttons, and control-board IO.

The runtime is designed for Raspberry Pi 5 + Ubuntu RT and can be configured for
multiple robot arms through XML files in `config/`, `robots/`, and `grippers/`.

> Safety: this software can move real robot hardware. Do the first startup with
> motors disabled or the robot mechanically safe, and verify E-stop, power, CAN,
> homing, and joint limits before enabling motion.

## Supported Target

First public target:

- Raspberry Pi 5
- Ubuntu with a PREEMPT_RT capable kernel
- Python 3.11+
- SocketCAN interface exposed as `can0` at 1 Mbit
- Mainboard/control PCB on `/dev/ttyAMA0`
- Optional front-panel SSD1306 OLED, buttons, and LEDs

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

**`.venv` is what runs the robot.** The system Python (`~/.local`) carries different
versions of several packages, so check dependencies with `.venv/bin/python -m pip list`
— a bare `python3` will show you an environment the robot does not use, and has already
caused two "it's installed" / "it isn't installed" mix-ups.

## Systemd Service

> ⚠️ **NOT PROPERLY TESTED YET.** The unit template renders and the commands below are
> correct in form, but the service has not been through a full boot-to-running validation
> on a clean machine. Day-to-day development on the reference robot does **not** use it —
> see *Manual Run (how development is actually done)* below.

After a successful manual install, install the service template:

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

## Manual Run (how development is actually done)

On the reference robot the three runtime processes are started **by hand**, each in its own
terminal, over **VS Code Remote-SSH**. This is the normal working setup, not a fallback:

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
| `supervisor.py` | OLED/buttons/LEDs, UART PCB IO, process supervision |

`nicegui_test.py` is the current operator dashboard. `gui.py` is kept for now as a simpler legacy dashboard while the public runtime settles.

Only `RTI.py` is intended to request real-time scheduling and a pinned core. The
service grants permission for that child process but does not make every process
real-time.

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

## Public Repo Notes

For the first public release, internal planning markdown files and manual dummy
test scripts are ignored by `.gitignore`. Keep user-facing setup information in
this `README.md` or in explicitly published docs added later.

## License

GPL-3.0, see `LICENSE`.

## Contact

Source Robotics - petar@source-robotics.com
