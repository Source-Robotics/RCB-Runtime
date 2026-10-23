# Versioning: what to do when you change something

Every part of the robot carries a version: the runtime, the Python client, three protocols, the configuration file
formats, the supported drive firmware, the hardware revisions. This file says which one to change when you change
code, and how a release is made. Follow it on every change, whether you are a person or an AI assistant.

What changed in each release is in [CHANGELOG.md](CHANGELOG.md). Which versions work together is in
[COMPATIBILITY.md](COMPATIBILITY.md). The documentation is at
[docs.source-robotics.com/par6](https://docs.source-robotics.com/par6/), from the repository
[Source-Robotics/par6](https://github.com/Source-Robotics/par6).

## What carries a version

| Part | Where it is set | Form | It changes when |
|---|---|---|---|
| Runtime | `VERSION` | `MAJOR.MINOR.PATCH` | a release is made, see below |
| Python client | `robotclient/version.py` `__version__` | the runtime's version | always together with `VERSION` |
| Command protocol | `communication/protocol.py` `PROTOCOL_VERSION`, and its copy in `robotclient/version.py` | integer | a command or field changes incompatibly |
| Real-time command packet | `communication/rti_protocol.py` `RTI_PROTOCOL_VERSION`, and its copy | integer | the packet changes incompatibly |
| Telemetry format | `utility/serialization.py` `TELEMETRY_FORMAT`, and its copy | integer | the `tag:value,...` layout changes incompatibly |
| Configuration file formats | `config/schema_versions.py` `CURRENT`, one number per kind of file | integer | a kind of file changes incompatibly |
| Supported drive firmware | `hardware/firmware_support.py` `SUPPORTED` | a range per driver type | the runtime needs, or is tested with, other firmware |
| Arm and gripper hardware | `<hardware_revision>` in `robots/<robot>.xml` and `grippers/<gripper>.xml` | `vMAJOR.MINOR` | a new hardware revision is built |
| Description package | `<description_package min="...">` in `robots/<robot>.xml` | the oldest version accepted | the models need a newer package |
| Control box PCB | `<control_box><pcb_version>` in `config/system.xml`, read by `hardware/control_box.py` | integer | a new PCB revision exists |

**The runtime's version** follows these rules:

- **PATCH:** a fix that no program and no configuration file can see.
- **MINOR:** new commands, fields or configuration keys that keep older programs and files working.
- **MAJOR:** a protocol or a configuration file format changed incompatibly.

**An incompatible change** is one that would make an existing program or file wrong: a command, field or key
renamed or removed, a value that changes its meaning or its unit, a changed default, a key that becomes required.
Adding something optional is never incompatible.

## On every change

Go through this list before you commit:

1. **Is the change visible to a user**, meaning a program, a configuration file, the dashboard or the robot's
   behaviour? Then add one line under `## [Unreleased]` in `CHANGELOG.md`, in plain words, under **Added**,
   **Changed**, **Fixed** or **Removed**. Internal refactors and tests get no line.
2. **Did a protocol change incompatibly?** Raise its number on the robot side and in `robotclient/version.py`
   together, and say so under **Changed**. The next release is a MAJOR one. A new command or optional field needs no
   new number.
3. **Did a kind of configuration file change incompatibly?** Raise its number in `config/schema_versions.py`. Make
   older files of that kind read correctly, preferably with an upgrade step that turns the old layout into the new one
   before the parser sees it. Set `schema="N"` on the shipped files of that kind. Note it under **Changed**. A new
   optional key needs no new number.
4. **Does the runtime now need other drive firmware, or was it tested with newer firmware?** Update
   `hardware/firmware_support.py`.
5. **Is there a new hardware revision of the arm or a gripper, or does the robot need a newer description
   package?** Update the robot or gripper XML.
6. **Did anything in the newest row of `COMPATIBILITY.md` change?** Update that row in the same change. The tests
   compare it with the code.
7. **Do not change `VERSION`** in an ordinary change. It changes only when a release is made.

## Making a release

1. **Choose the number** from what is under `## [Unreleased]`: any incompatible change makes a MAJOR release, any
   new feature a MINOR one, and only fixes a PATCH.
2. **Set the number** in `VERSION` and in `robotclient/version.py` (`__version__`).
3. **Close the changelog section:** rename `## [Unreleased]` to `## [X.Y.Z] - YYYY-MM-DD`, using the day the tag is
   made, and add a new empty `## [Unreleased]` above it.
4. **Add a row to `COMPATIBILITY.md`** at the top of the table for `X.Y.Z`, with every column filled in.
5. **Test it:** the full test suite (`tools/run_nets.py`), then the simulated robot (`tools/sim_run.py`) and the
   smoke routine on an arm (`tools/arm_smoke.py`) when the runtime itself changed.
6. **Commit and tag:** commit the release, then make an annotated tag whose message is the release notes:
   ```bash
   tools/release_notes.py X.Y.Z > /tmp/notes.md
   git tag -a vX.Y.Z -F /tmp/notes.md
   ```
7. **Publish:** push the branch and the tag, then create the GitHub release from the tag with the same notes:
   ```bash
   gh release create vX.Y.Z --title "X.Y.Z" --notes-file /tmp/notes.md
   ```
8. **Update the documentation**, in the repository `Source-Robotics/par6`:
   1. Set `runtime.tag` in `versions.yml` to `vX.Y.Z`. For a MINOR or MAJOR release, also set `docs_version` to
      `"X.Y"`.
   2. Run `tools/sync_runtime.py`. It regenerates the compatibility table, the release notes, the error codes and
      the telemetry tags from the tag, and copies the source of the command reference: the client's docstrings, the
      command rules, the error catalog and the per-move settings (that repository's `BUILDING_THE_REFERENCE.md`).
      Its build check fails when a per-move setting or a value of a choice is not described yet.
   3. Update the guides for what changed and give each changed guide a new revision letter.
   4. Push. The site publishes the version by itself.

## What the tests check

The runtime's test suite fails when the numbers disagree:

- `VERSION` and the newest released section of `CHANGELOG.md` name the same version.
- The newest row of `COMPATIBILITY.md` matches the code: the client series, the three protocol numbers, the
  configuration formats, the firmware range, the PCB versions, the hardware revisions, the description package
  minimum, and the OS image.
- The client's copies of the protocol numbers equal the robot's, and `robotclient.__version__` equals `VERSION`.
- Every shipped configuration file is in a format the runtime reads.
