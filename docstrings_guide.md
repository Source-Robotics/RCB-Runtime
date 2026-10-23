# Docstring guide

How every docstring in this repository is written. The documentation site builds the Python API reference from the
docstrings of `robotclient`, and editors show them on hover, so a docstring is user documentation. Follow this guide
exactly, whether you are a person or an AI assistant.

**Scope:**

- **Required in full** for every public function, class, method and property of `robotclient`: everything without a
  leading underscore. These form the API reference.
- **The same format** for the rest of the repository. There the sections are required only where they help, but
  never use another style.

The format is the **Google style**, read by the documentation generator (mkdocstrings with Griffe), with an
**imperative first line** as PEP 257 asks, and **Markdown** inside, never reStructuredText.

## The template

```python
def move_linear(self, x: float, y: float, z: float, rx: float, ry: float, rz: float,
                timeout: float = DEFAULT_MOVE_TIMEOUT, *, angle_unit: str = "rad",
                **options: Any) -> str:
    """Move the tool in a straight line to a pose.

    The tool centre point travels on a straight line in the base frame while
    its orientation turns smoothly to the target.

    Args:
        x: Target position along the base X axis, in metres.
        y: Target position along the base Y axis, in metres.
        z: Target position along the base Z axis, in metres.
        rx: Turn about the base X axis, in radians, or degrees with
            `angle_unit="deg"`. The orientation is R = Rx(rx) Ry(ry) Rz(rz).
        ry: Turn about the tool's new Y axis, in the same unit.
        rz: Turn about the tool's own Z axis, in the same unit.
        timeout: Seconds to wait for the move to finish, in sync mode only.
        angle_unit: `"rad"` or `"deg"`: the unit of every angle in this call.
        **options: Settings for this move only, overriding the defaults:
            `speed` and `accel` (a fraction of the limit up to 1, or a percent
            above 1), `duration` (seconds), `profile`, `jerk`, `ramp_fraction`,
            `blend`, `blend_mode`, `blend_continuity`, `ik_solver`,
            `singularity_strategy`, `samples` and `dry_run`. A setting left out
            takes its default. See Per-move settings.

    Returns:
        The command id. The call returns once the robot has accepted the move;
        call `wait_idle()` to wait until it has finished. In sync mode the call
        returns when the move has finished.

    Raises:
        CommandRejected: The robot refused the move, for example outside EXEC
            mode, before homing or with an invalid argument. In sync mode also
            when the move fails after it was accepted.
        RobotTimeout: The robot did not accept the move within 5 s. In sync
            mode also when the move did not finish within `timeout`.
        RobotConnectionError: The client is not connected.
        ValueError: `angle_unit` is neither `"rad"` nor `"deg"`.

    Note:
        Moves along a Cartesian path run at no more than 20 % of the speed
        limit; a higher `speed` has no further effect.

    Examples:
        >>> x, y, z, rx, ry, rz = robot.get_pose()
        >>> robot.move_linear(x, y, z - 0.05, rx, ry, rz)
        >>> robot.wait_idle()
    """
```

## Complete, always

Document **everything** the code accepts and does. Never a subset:

- **Every parameter,** including `*args`, `**kwargs` and keyword-only ones.
- **Every option a `**options` takes.** If the robot accepts 13 settings, the docstring names all 13. Settings
  the robot refuses are not named.
- **Every allowed value** of a choice, such as every profile name.
- **Every exception** a caller should handle.

When the docstring and the code disagree, the code is right: fix the docstring in the same change. The test suite
checks every `**options` list against the robot's own list of settings.

## The parts, in this order

1. **Summary line** (required).
2. **Description** (optional).
3. **`Args:`** (required when the function takes arguments).
4. **`Defaults:`** (required for a command that changes a default, such as `set_joint_speed()`).
5. **`Returns:`** (required unless it returns `None`). Generators use **`Yields:`** instead.
6. **`Raises:`** (required when a caller should expect an exception).
7. **`Errors:`** (required for a queued command: the codes it can fail with once the robot accepted it).
8. **`Note:`** or **`Warning:`** (optional).
9. **`See also:`** (optional).
10. **`Examples:`** (required for every public `robotclient` function and method).

`Defaults:`, `Errors:` and `See also:` are this project's own sections, not part of the Google style. The
documentation build reads them; editors show them as plain text.

Each section heading ends with a colon and stands on its own line, with a blank line above it and none below it.
Section content is indented 4 spaces deeper than the heading; a continuation line 4 spaces deeper again.

### 1. Summary line

- **One line,** at most 72 characters, ending with a period, right after the opening `"""`.
- **Imperative mood:** "Move the tool…", "Return the joint angles…", "Wait until…". Not "Moves", not "This method
  moves".
- **What the call does for the user,** not how it is built.
- **Properties are the exception:** they read like an attribute. "The runtime version the robot reported.", not
  "Return the runtime version."

### 2. Description

One to three short paragraphs: what happens, what the robot does, when the call returns, and what to use instead
when this is the wrong call. Leave out implementation details unless they change how the call is used.

### 3. `Args:`

- **Every parameter, in signature order,** including `*args` and `**kwargs` with their stars, and keyword-only
  parameters. Leave out `self` and `cls`.
- **`name: Description.`** The description starts with a capital letter and is a full sentence ending in a period.
- **No types.** Types come only from the type hints, which the generator reads. Never write `x (float): …`.
- **A unit for every number,** from the table below.
- **Defaults** are shown by the generator from the signature. Mention a default only when its meaning needs words,
  for example "0 means no limit".
- **Allowed values** of a string or a choice are listed in backticks: `` `"rad"` or `"deg"` ``.

### 4. `Defaults:`

For a command that changes a default: the value it has when the robot software starts, and anything else that sets
it. The page shows it as "Default values".

```python
    Defaults:
        100 %, set each time the robot software starts.
```

### 5. `Returns:`

What the value means, with its unit. Not its type: the type comes from the return annotation. Leave the section out
when the function returns `None`. For a tuple, say what each element is, in order.

### 6. `Raises:`

Every exception a caller should be ready for, as `ExceptionName: When it is raised.`:

1. **The robot's exceptions first,** in this order: `CommandRejected`, `MotionFailed`, `HomingFailed`,
   `GripFailed`, `RobotTimeout`, `RobotConnectionError`, `VersionMismatch`.
2. **Then the argument errors:** `ValueError`, `TypeError`.

Do not list exceptions that only a broken program could cause.

### 7. `Errors:`

For a queued command: every code from the robot's error catalog (`utility/error_catalog.py`) that this command can
fail with after the robot accepted it, one per line, as `CODE: When it happens.` Name the code, never its number:
the documentation looks up the number and the title.

```python
    Errors:
        JOINT_LIMIT: A target angle is outside its joint's limits.
        DURATION_INFEASIBLE: `duration` is shorter than the limits allow.
```

Leave out what every queued command shares; the documentation adds it from the robot's rules: the refusals when it
is sent (`REJECTED`, `CONTROL_HELD`, `MOTION_ABORTED`, `BAD_OPTIONS`, `QUEUE_FULL`), `PLAN_FAIL` and `SEND_TIMEOUT`,
and the ways a running move ends early (`ABORTED`, `STOPPED`, `SETTLE_TIMEOUT`). Find the command's own codes in the
planner's code path for it, every one of them (see "Complete, always").

### 8. `Note:` and `Warning:`

- **`Note:`** for behaviour that is easy to miss, such as a limit or a side effect.
- **`Warning:`** for anything that can damage the robot or hurt someone, such as motion without a check, or switching
  a safety function off.

At most one of each. They render as boxes in the documentation.

### 9. `See also:`

Related public methods, as `` `name()` ``, separated by commas. Methods the text already names are linked anyway;
list the ones a reader would look for next.

### 10. `Examples:`

- **Lines in Python console form,** each starting with `>>> `, continued with `... `. Show output only when it
  explains something.
- **Every example runs on a real robot,** given a connected, enabled and homed `robot` in the right mode. Build poses
  from `robot.get_pose()`, or use the standard start pose `move_joints(90, -105, 165, 0, -30, 180)`. Moves stay small:
  a few centimetres, a few degrees.
- **The common case first,** then at most two variations.
- **End with the wait** a real program needs, usually `robot.wait_idle()`.

## Writing rules

- **Plain English:** short sentences, active voice, present tense, no jargon a robot user would not know.
- **Markdown,** never reStructuredText. Code, names and values go in single backticks: `` `speed` ``,
  `` `wait_idle()` ``, `` `"deg"` ``. Never write ` ``double`` ` backticks, `:param:` or `:class:`.
- **Names as the code spells them:** methods with their parentheses, such as `wait_idle()`. Parameters bare, such as
  `angle_unit`.
- **No other companies' product names,** as everywhere in published files.
- **Lines** at most 88 characters wide inside a docstring, so that editor tooltips stay readable.

## Units

| Quantity | Unit in docstrings | Notes |
|---|---|---|
| Cartesian position, offsets, radius | metres | |
| Pose and TCP angles | radians, or degrees with `angle_unit="deg"` | say which, every time |
| Joint angles at the Python API | degrees | |
| Joint speed, acceleration | degrees per second, degrees per second² | |
| Time | seconds | |
| Blend radius, tolerances on paths | millimetres | |
| Speed and acceleration options | a fraction of the limit up to 1, or a percent above 1 | |
| Force | newtons | |
| Torque | newton-metres | |
| Mass | kilograms | |
| Current | milliamperes | |
| Gripper position | the gripper's 0 to 255 range | |

## What never goes into a docstring

- **Which modes and states allow a command.** The documentation build reads that from the robot's own command rules
  (`communication/protocol.py`) and shows it as "Allowed when". A docstring copy would go stale.
- **The wire protocol name.** It is generated too.
- **Version history,** such as "added in 0.9". That belongs in `CHANGELOG.md`.
- **Types.** They come from the type hints.

## Classes, properties, aliases, modules

- **A class docstring** starts with what one instance is: "A connection to one robot.". Do not write "This class". It
  lists public attributes, not properties, under `Attributes:`.
- **Constructor arguments** are documented in `__init__`'s docstring. The documentation shows them with the class.
- **An exception class** says what the exception represents, not when it is raised, and lists its attributes.
- **An alias,** an older name kept for compatibility, has no docstring of its own. It is a class-level assignment,
  `old_name = new_name`, under a comment that names it an alias. When the command's wire ID changes too, the old ID
  goes into `communication/protocol.COMMAND_ALIASES`, so raw and older clients keep working, and the pair goes into
  `robotclient/version.WIRE_NAMES_BEFORE_RENAME`, so the new client keeps working with an older robot (it sends that
  robot the ID it knows; the test keeps the two tables equal). The documentation lists the old names on the group's
  overview page.
- **A method's name is its wire command ID.** One spelling for the call, the message and the log line (the test
  `test_command_aliases` checks every method).
- **A module docstring** has a one-line summary, then what the module contains.
- **Private names,** with a leading underscore, get a short docstring when the code is not obvious. They are not in
  the documentation.

## Checks

The runtime's test suite checks every documented group of `robotclient` against this guide:

- a summary line: one line, imperative, ending with a period
- `Args:` naming exactly the signature's parameters, in order
- `Returns:` present unless the return type is `None`
- `Raises:` and `Examples:` present
- no types in `Args:`, no reStructuredText, no line over 88 characters
- a `**options` description naming every per-move setting the robot accepts, and none it refuses
- every code in `Errors:` in the error catalog; every name in `See also:` a public method

When you add or change a public method, update its docstring in the same change.

## Checklist for a new or changed public method

1. The summary line is imperative, at most 72 characters, and ends with a period.
2. Every parameter is in `Args:`, in order, with a unit, without a type.
3. `Returns:` says what the value means, unless the return is `None`.
4. `Raises:` lists what a caller should handle, robot exceptions first.
5. The example runs on a real robot, ends with the wait it needs, and uses the start pose or `get_pose()`.
6. No mode rules, protocol names, version history or types are written in.
7. The test suite passes.
