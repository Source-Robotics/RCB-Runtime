# Compatibility

Which versions of everything around the RCB runtime work with each runtime release. One row per release, newest
first. What changed in each release is in `CHANGELOG.md`.

| Runtime | Client | Command protocol | RTI protocol | Telemetry format | Config formats | Drive firmware | Control box PCB | Arm and grippers | Description package | OS image | Docs |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 1.1.1 | 1.1.x | 2 | 1 | 1 | 1 | 103 or newer | v1, v2 | PAR6 v1.0; MSG v1.0; SSG48 v1.0 | 2.0 or newer | Ubuntu 24.04.2 LTS, kernel 6.8.4-rt11-raspi | 1.1 |
| 1.1.0 | 1.1.x | 2 | 1 | 1 | 1 | 103 or newer | v1, v2 | PAR6 v1.0; MSG v1.0; SSG48 v1.0 | 2.0 or newer | Ubuntu 24.04.2 LTS, kernel 6.8.4-rt11-raspi | 1.1 |
| 1.0.0 | 1.0.x | 2 | 1 | 1 | 1 | 103 or newer | v1, v2 | PAR6 v1.0; MSG v1.0; SSG48 v1.0 | 2.0 or newer | Ubuntu 24.04.2 LTS, kernel 6.8.4-rt11-raspi | 1.0 |
| 0.10.0 | 0.10.x | 1 | 1 | 1 | 1 | 103 or newer | v1, v2 | PAR6 v1.0; MSG v1.0; SSG48 v1.0 | 2.0 or newer | Ubuntu 24.04.2 LTS, kernel 6.8.4-rt11-raspi | 0.10 |
| 0.9.0 | 0.9.x | 1 | 1 | 1 | 1 | 103 or newer | v1, v2 | PAR6 v1.0; MSG v1.0; SSG48 v1.0 | 2.0 or newer | Ubuntu 24.04.2 LTS, kernel 6.8.4-rt11-raspi | not published yet |

## What each column means

- **Runtime:** the `VERSION` file, and the git tag `v<version>`.
- **Client:** the Python client in `robotclient/` that ships with that runtime, `robotclient.__version__`. Any client
  that speaks the same command protocol number works. A client checks that number when it connects and refuses a
  different one.
- **Command protocol:** the newline JSON the executor speaks on TCP port 29000. Its number goes up only when a
  command or field changes incompatibly.
- **RTI protocol:** the binary command packet of real-time streaming. Same rule.
- **Telemetry format:** the `tag:value,...` state and telemetry packets. Same rule.
- **Config formats:** the newest format number this runtime reads, for every kind of configuration file: system,
  robot, gripper, homing, cell, filters and recipe. A file without a `schema` attribute is format 1. A file in a newer
  format is refused. A file in an older one is read with a warning.
- **Drive firmware:** the joint and gripper drive firmware this runtime supports, for both drive types, stepfoc and
  spectral-bldc. A drive outside the range gets a warning, and the first-run check fails it.
- **Control box PCB:** the PCB versions `config/system.xml` may declare. The version decides how the E-stop channels
  are read.
- **Arm and grippers:** the hardware revisions this release was made and tested for.
- **Description package:** the oldest PAR6 description package, with its URDF and meshes, that the robot's models
  and self-collision geometry are made from.
- **OS image:** the operating system and real-time kernel the release was tested on.
- **Docs:** the documentation version for this release.

The runtime's tests compare the current runtime's row with the code. They fail when a protocol number, a format,
the firmware range or a revision changes without this table.
