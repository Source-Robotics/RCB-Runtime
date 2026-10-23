"""Block program -> Python: the workspace the editor saves becomes a `robotclient` script.

Why here and not in the browser (TODO/blockly_plan.md, D1): the browser sends block DATA, never code. Every value
that reaches the script is checked here and written with repr() — numbers must be finite numbers inside the block's
range, names strings, dropdown values one of the block's own options — so a page can make the robot do only what the
blocks can express. Anything else is REFUSED with the block it concerns (`CodegenError.block_id`, which the editor
selects): a block type with no entry in STATEMENTS / VALUES, an empty socket, a value of the wrong kind for its
socket, a number out of range, a point the program does not have.

The input is the dict `Blockly.serialization.workspaces.save()` returns. Only the stack under the one enabled
"when the program starts" block (`rcb_start`) becomes code; loose blocks elsewhere are ignored, and so are disabled
blocks with everything inside them (Blockly's own generators skip them the same way).

Generated code calls `rt.block('<id>')` before every statement (the editor highlights the block the robot is on)
and starts every loop body with `rt.loop()` (where stop / pause land, and what keeps an empty `repeat forever` from
spinning a core). `rt` is blockly/runtime.py; the points table is written in as the literal `P`, every variable
starts at 0.

Units: blocks show mm and degrees; the API takes metres and radians for a pose (degrees for joints). `rt.pose()`
converts; a relative move's mm are written as metres here. Speeds are written as FRACTIONS (50 % -> speed=0.5): the
executor reads a per-command speed <= 1.0 as a fraction and anything above as a percent (motion/trajectory_planner.py
`_speed_scale_for`), so "1 %" written as speed=1 would run at 100 %.

Value kinds (a socket names the kinds it takes, and Blockly's own connection checks use the same names):
    Number, Boolean, String, Any (a variable)    Joints (deg list), Pose (m/rad list), Point (a {joints_deg, pose} dict)
"""
import math
import re

from blockly.programs import ProgramError, check_points

MAX_BLOCKS = 5000
MAX_DEPTH = 60              # sockets / statement inputs inside each other; Python itself stops at 20 nested loops
MAX_ID_LEN = 64
MAX_NAME_LEN = 100
MAX_TEXT_LEN = 500
MAX_VARIABLES = 200
JOINT_SANITY_DEG = 1000.0   # garbage filter only — the joint LIMITS are the robot XML's, checked by the executor
NUMBER_SANITY = 1e12
GRIPPER_OPEN, GRIPPER_CLOSED = 1, 254       # move_gripper's raw scale (docs/Robotclient_commands.md)
INDENT = "    "

KIND_NAMES = {"Number": "a number", "Boolean": "a yes / no value", "String": "text", "Any": "a variable",
              "Joints": "joint angles", "Pose": "a pose", "Point": "a point"}
NUMERIC = ("Number", "Any")
TRUTHY = ("Boolean", "Number", "Any")
PLAIN = ("Number", "Boolean", "String", "Any")


class CodegenError(Exception):
    """A program that cannot become Python; `block_id` is the block to show (None: the program as a whole)."""

    def __init__(self, block_id, message):
        super().__init__(message)
        self.block_id = block_id
        self.message = message


def generate(workspace, name="program", points=None):
    """The Python script for one block program; `points` is the program's points table (blockly/programs.py
    format), written into the script as the literal `P`, so a downloaded script needs no program file.
    Raises CodegenError."""
    if not isinstance(name, str) or not 0 < len(name) <= MAX_NAME_LEN:
        raise CodegenError(None, f"the program needs a name of 1-{MAX_NAME_LEN} characters")
    try:
        points = check_points(points or {})
    except ProgramError as e:
        raise CodegenError(None, str(e)) from None
    ctx = {"ids": set(), "points": points, "vars": _variables(workspace), "loops": 0}
    starts = [b for b in _top_blocks(workspace) if b.get("type") == "rcb_start" and not _disabled(b)]
    if not starts:
        raise CodegenError(None, "add a 'when the program starts' block: only the blocks under it run")
    if len(starts) > 1:
        raise CodegenError(_id(starts[1]), "only one 'when the program starts' block may be enabled")
    _visit(starts[0], ctx)
    body = _statements(_next(starts[0]), 0, ctx)
    lines = [f"# Generated from the block program {name!r} by blockly/codegen.py - edit the blocks, not this file.",
             "from blockly import runtime as rt",
             "",
             "robot = rt.start()",
             *_points_literal(points),
             *[f"{py} = 0" for py in ctx["vars"].values()],
             "",
             *body,
             "rt.finish()"]
    code = "\n".join(lines) + "\n"
    try:
        compile(code, "<block program>", "exec")
    except SyntaxError as e:
        if "nested blocks" in str(e):
            raise CodegenError(None, "loops are nested more than 20 deep") from None
        raise CodegenError(None, f"internal error: the generated code does not compile ({e})") from None
    return code


# ── statements: block -> list of lines ──────────────────────────────────────────────────────────────────────────

def _move_joints(block, depth, ctx):
    code, kind, literal = _value(block, "TARGET", "the target", ("Joints", "Point", "Pose"), depth, ctx)
    speed = _speed(block)
    if kind == "Joints":
        target = ", ".join(map(repr, literal))
    elif kind == "Point":
        target = f"*{code}['joints_deg']"
    else:       # a pose: the planner's own IK only searches near its start, is_reachable searches the whole range
        target = f"*rt.reach(robot, {code})"
    return [f"robot.move_joints({target}, speed={speed})"]


def _move_linear(block, depth, ctx):
    code, kind, _ = _value(block, "TARGET", "the target of a straight-line move", ("Point", "Pose"), depth, ctx)
    target = f"*{code}['pose']" if kind == "Point" else f"*{code}"
    return [f"robot.move_linear({target}, speed={_speed(block)})"]


def _move_relative(block, depth, ctx):
    d = [_number(block, n, n[1].lower(), -2000, 2000) / 1000.0 for n in ("DX", "DY", "DZ")]
    frame = _option(block, "FRAME", ("world", "tool"))
    return [f"robot.move_linear_rel_{frame}({d[0]!r}, {d[1]!r}, {d[2]!r}, speed={_speed(block)})"]


def _turn_joint(block, depth, ctx):
    joint = _option(block, "JOINT", ("1", "2", "3", "4", "5", "6"))
    deg = _number(block, "DEG", "the angle", -720, 720)
    return [f"rt.turn_joint(robot, {joint}, {deg!r}, speed={_speed(block)})"]


def _wait(block, depth, ctx):
    return [f"rt.wait({_number(block, 'SECONDS', 'the time', 0, 86400)!r})"]


def _gripper_open(block, depth, ctx):
    return ["robot.open_gripper()"]


def _gripper_close(block, depth, ctx):
    return ["robot.close_gripper()"]


def _gripper_move(block, depth, ctx):
    pct = _number(block, "PERCENT", "the closing", 0, 100)
    return [f"robot.move_gripper({round(GRIPPER_OPEN + pct * (GRIPPER_CLOSED - GRIPPER_OPEN) / 100.0)})"]


def _set_output(block, depth, ctx):
    pin = _option(block, "PIN", ("1", "2", "3"))
    on = _option(block, "STATE", ("ON", "OFF")) == "ON"
    return [f"robot.set_digital_output({pin}, {on})"]


def _wait_input(block, depth, ctx):
    pin = _option(block, "PIN", ("1", "2", "3"))
    on = _option(block, "STATE", ("ON", "OFF")) == "ON"
    timeout = _number(block, "TIMEOUT", "the time limit", 0.1, 3600)
    return [f"rt.wait_input(robot, {pin}, {on}, {timeout!r})"]


def _log(block, depth, ctx):
    code, _, _ = _value(block, "TEXT", "what to log", PLAIN, depth, ctx)
    return [f"rt.log({code})"]


def _halt(block, depth, ctx):
    return ["rt.halt()"]


def _repeat(block, depth, ctx):
    code, _kind, literal = _value(block, "TIMES", "the number of times", NUMERIC, depth, ctx)
    times = repr(int(literal)) if literal is not None else f"int({code})"
    return [f"for _ in range({times}):", *_loop_body(block, depth, ctx)]


def _while_until(block, depth, ctx):
    mode = _option(block, "MODE", ("WHILE", "UNTIL"))
    cond, _, _ = _value(block, "BOOL", "the condition", TRUTHY, depth, ctx)
    return [f"while {cond if mode == 'WHILE' else f'not {cond}'}:", *_loop_body(block, depth, ctx)]


def _for(block, depth, ctx):
    var = _variable(block, ctx)
    ends = [_value(block, n, what, NUMERIC, depth, ctx)[0]
            for n, what in (("FROM", "the start"), ("TO", "the end"), ("BY", "the step"))]
    return [f"for {var} in rt.count({', '.join(ends)}):", *_loop_body(block, depth, ctx)]


def _flow(block, depth, ctx):
    flow = _option(block, "FLOW", ("BREAK", "CONTINUE"))
    if ctx["loops"] == 0:
        raise CodegenError(_id(block), "this block only works inside a loop")
    return ["break" if flow == "BREAK" else "continue"]


def _if(block, depth, ctx):
    state = block.get("extraState") or {}
    elifs = state.get("elseIfCount", 0) if isinstance(state, dict) else 0
    has_else = state.get("hasElse", False) if isinstance(state, dict) else False
    if isinstance(elifs, bool) or not isinstance(elifs, int) or not 0 <= elifs <= 50 or not isinstance(has_else, bool):
        raise CodegenError(_id(block), "this 'if' block is damaged")
    lines = []
    for i in range(elifs + 1):
        cond, _, _ = _value(block, f"IF{i}", "the condition", TRUTHY, depth, ctx)
        lines.append(f"{'if' if i == 0 else 'elif'} {cond}:")
        lines += [INDENT + ln for ln in (_body(block, f"DO{i}", depth, ctx) or ["pass"])]
    if has_else:
        lines.append("else:")
        lines += [INDENT + ln for ln in (_body(block, "ELSE", depth, ctx) or ["pass"])]
    return lines


def _set_variable(block, depth, ctx):
    var = _variable(block, ctx)
    code, _, _ = _value(block, "VALUE", "the value", PLAIN, depth, ctx)
    return [f"{var} = {code}"]


def _change_variable(block, depth, ctx):
    var = _variable(block, ctx)
    code, _, _ = _value(block, "DELTA", "the change", NUMERIC, depth, ctx)
    return [f"{var} += {code}"]


STATEMENTS = {
    "rcb_move_joints": _move_joints,
    "rcb_move_linear": _move_linear,
    "rcb_move_relative": _move_relative,
    "rcb_turn_joint": _turn_joint,
    "rcb_wait": _wait,
    "rcb_gripper_open": _gripper_open,
    "rcb_gripper_close": _gripper_close,
    "rcb_gripper_move": _gripper_move,
    "rcb_set_output": _set_output,
    "rcb_wait_input": _wait_input,
    "rcb_log": _log,
    "rcb_halt": _halt,
    "controls_repeat_ext": _repeat,
    "controls_whileUntil": _while_until,
    "controls_for": _for,
    "controls_flow_statements": _flow,
    "controls_if": _if,
    "variables_set": _set_variable,
    "math_change": _change_variable,
}


# ── values: block -> (python expression, kind, the constant value or None) ─────────────────────────────────────

def _point(block, depth, ctx):
    name = (block.get("fields") or {}).get("NAME")
    if not isinstance(name, str) or not name:
        raise CodegenError(_id(block), "pick a point")
    if name not in ctx["points"]:
        raise CodegenError(_id(block), f"this program has no point {name[:40]!r} — teach it, or pick another")
    return f"P[{name!r}]", "Point", None


def _joints(block, depth, ctx):
    angles = [_number(block, f"J{i}", f"J{i}", -JOINT_SANITY_DEG, JOINT_SANITY_DEG) for i in range(1, 7)]
    return "[" + ", ".join(map(repr, angles)) + "]", "Joints", angles


def _pose(block, depth, ctx):
    xyz = [_number(block, n, n.lower(), -5000, 5000) for n in ("X", "Y", "Z")]
    rpy = [_number(block, n, n.lower(), -720, 720) for n in ("RX", "RY", "RZ")]
    return f"rt.pose({', '.join(map(repr, xyz + rpy))})", "Pose", None


def _here(block, depth, ctx):
    return "rt.here(robot)", "Point", None


def _shift(block, depth, ctx):
    code, kind, _ = _value(block, "TARGET", "what to shift", ("Point", "Pose"), depth, ctx)
    base = f"{code}['pose']" if kind == "Point" else code
    d = [_number(block, n, n[1].lower(), -5000, 5000) for n in ("DX", "DY", "DZ")]
    return f"rt.shifted({base}, {', '.join(map(repr, d))})", "Pose", None


def _gripper_holding(block, depth, ctx):
    return "rt.holding(robot)", "Boolean", None


def _input(block, depth, ctx):
    return f"robot.get_digital_input({_option(block, 'PIN', ('1', '2', '3'))})", "Boolean", None


def _math_number(block, depth, ctx):
    v = _number(block, "NUM", "the number", -NUMBER_SANITY, NUMBER_SANITY)
    return repr(v), "Number", v


def _arithmetic(block, depth, ctx):
    op = _option(block, "OP", ("ADD", "MINUS", "MULTIPLY", "DIVIDE", "POWER"))
    a = _value(block, "A", "the first number", NUMERIC, depth, ctx)[0]
    b = _value(block, "B", "the second number", NUMERIC, depth, ctx)[0]
    if op == "POWER":
        return f"rt.power({a}, {b})", "Number", None
    return f"({a} {dict(ADD='+', MINUS='-', MULTIPLY='*', DIVIDE='/')[op]} {b})", "Number", None


def _compare(block, depth, ctx):
    op = _option(block, "OP", ("EQ", "NEQ", "LT", "LTE", "GT", "GTE"))
    a = _value(block, "A", "the first value", PLAIN, depth, ctx)[0]
    b = _value(block, "B", "the second value", PLAIN, depth, ctx)[0]
    return f"({a} {dict(EQ='==', NEQ='!=', LT='<', LTE='<=', GT='>', GTE='>=')[op]} {b})", "Boolean", None


def _logic(block, depth, ctx):
    op = _option(block, "OP", ("AND", "OR"))
    a = _value(block, "A", "the first condition", TRUTHY, depth, ctx)[0]
    b = _value(block, "B", "the second condition", TRUTHY, depth, ctx)[0]
    return f"({a} {op.lower()} {b})", "Boolean", None


def _negate(block, depth, ctx):
    return f"(not {_value(block, 'BOOL', 'the condition', TRUTHY, depth, ctx)[0]})", "Boolean", None


def _boolean(block, depth, ctx):
    return repr(_option(block, "BOOL", ("TRUE", "FALSE")) == "TRUE"), "Boolean", None


def _text(block, depth, ctx):
    text = (block.get("fields") or {}).get("TEXT")
    if not isinstance(text, str) or len(text) > MAX_TEXT_LEN:
        raise CodegenError(_id(block), f"a text is at most {MAX_TEXT_LEN} characters")
    return repr(text), "String", text


def _get_variable(block, depth, ctx):
    return _variable(block, ctx), "Any", None


VALUES = {
    "rcb_point": _point,
    "rcb_joints": _joints,
    "rcb_pose": _pose,
    "rcb_here": _here,
    "rcb_shift": _shift,
    "rcb_gripper_holding": _gripper_holding,
    "rcb_input": _input,
    "math_number": _math_number,
    "math_arithmetic": _arithmetic,
    "logic_compare": _compare,
    "logic_operation": _logic,
    "logic_negate": _negate,
    "logic_boolean": _boolean,
    "text": _text,
    "variables_get": _get_variable,
}


# ── walking the workspace ───────────────────────────────────────────────────────────────────────────────────────

def _points_literal(points):
    if not points:
        return []
    return ["P = {", *[f"    {pname!r}: {{'joints_deg': {p['joints_deg']!r}, 'pose': {p['pose']!r}}},"
                       for pname, p in points.items()], "}"]


def _variables(workspace):
    """{variable id: python name}: `v_` + the name made an ASCII identifier, numbered on a clash. The prefix keeps
    them clear of Python keywords and of `rt`, `robot` and `P`."""
    found = workspace.get("variables", []) if isinstance(workspace, dict) else []
    if not isinstance(found, list) or len(found) > MAX_VARIABLES:
        raise CodegenError(None, "the program's variables are damaged")
    out, used = {}, set()
    for v in found:
        if not isinstance(v, dict) or not isinstance(v.get("id"), str) or not isinstance(v.get("name"), str):
            raise CodegenError(None, "the program's variables are damaged")
        base = "v_" + (re.sub(r"\W", "_", v["name"], flags=re.ASCII) or "_")[:40]
        py, n = base, 2
        while py in used:
            py, n = f"{base}_{n}", n + 1
        used.add(py)
        out[v["id"]] = py
    return out


def _variable(block, ctx):
    field = (block.get("fields") or {}).get("VAR")
    vid = field.get("id") if isinstance(field, dict) else None
    if vid not in ctx["vars"]:
        raise CodegenError(_id(block), "this block's variable does not exist")
    return ctx["vars"][vid]


def _top_blocks(workspace):
    blocks = workspace.get("blocks", {}) if isinstance(workspace, dict) else None
    top = blocks.get("blocks", []) if isinstance(blocks, dict) else None
    if not isinstance(top, list) or not all(isinstance(b, dict) for b in top):
        raise CodegenError(None, "this is not a block program")
    return top


def _statements(block, depth, ctx):
    """One stack, top to bottom. Walks `next` in a loop: a long program is a deep JSON chain, not deep recursion."""
    lines = []
    while block is not None:
        bid = _visit(block, ctx)
        if depth > MAX_DEPTH:
            raise CodegenError(bid, "blocks are nested too deeply")
        if not _disabled(block):
            emit = STATEMENTS.get(block.get("type"))
            if emit is None:
                raise CodegenError(bid, _unknown(block))
            lines.append(f"rt.block({bid!r})")
            lines += emit(block, depth, ctx)
        block = _next(block)
    return lines


def _body(block, name, depth, ctx):
    """The statements in a block's statement input (empty list for an empty input)."""
    child = _input(block, name)
    return _statements(child, depth + 1, ctx) if child is not None else []


def _loop_body(block, depth, ctx):
    """A loop's body, indented, starting with rt.loop(); break / continue are allowed inside it."""
    ctx["loops"] += 1
    try:
        return [INDENT + "rt.loop()", *[INDENT + ln for ln in _body(block, "DO", depth, ctx)]]
    finally:
        ctx["loops"] -= 1


def _value(block, name, what, accept, depth, ctx):
    child = _input(block, name)
    if child is None or _disabled(child):
        raise CodegenError(_id(block), f"{what} is empty")
    cid = _visit(child, ctx)
    if depth > MAX_DEPTH:
        raise CodegenError(cid, "blocks are nested too deeply")
    emit = VALUES.get(child.get("type"))
    if emit is None:
        raise CodegenError(cid, _unknown(child))
    code, kind, literal = emit(child, depth + 1, ctx)
    if kind not in accept:
        raise CodegenError(cid, f"{KIND_NAMES[kind]} cannot be {what} here")
    return code, kind, literal


def _input(block, name):
    inputs = block.get("inputs", {})
    slot = inputs.get(name) if isinstance(inputs, dict) else None
    if slot is None:
        return None
    child = (slot.get("block") or slot.get("shadow")) if isinstance(slot, dict) else slot
    if child is not None and not isinstance(child, dict):
        raise CodegenError(_id(block), "this block is damaged (an input is not a block)")
    return child


def _next(block):
    nxt = block.get("next")
    if nxt is None:
        return None
    child = (nxt.get("block") or nxt.get("shadow")) if isinstance(nxt, dict) else nxt
    if child is not None and not isinstance(child, dict):
        raise CodegenError(_id(block), "this block is damaged (the next block is not a block)")
    return child


def _number(block, field, what, lo, hi):
    fields = block.get("fields", {})
    v = fields.get(field) if isinstance(fields, dict) else None
    try:
        ok = not isinstance(v, bool) and isinstance(v, (int, float)) and math.isfinite(v)
    except OverflowError:       # an int too large for a float
        ok = False
    if not ok:
        raise CodegenError(_id(block), f"{what} is not a number")
    if not lo <= v <= hi:
        raise CodegenError(_id(block), f"{what} must be between {lo:g} and {hi:g}")
    return v


def _option(block, field, allowed):
    """A dropdown's value, which must be one of the block's own options."""
    fields = block.get("fields", {})
    v = fields.get(field) if isinstance(fields, dict) else None
    if v not in allowed:
        raise CodegenError(_id(block), f"this block's {field.lower()} is not one it offers")
    return v


def _speed(block):
    """The block's speed as the FRACTION the executor reads (see the module docstring)."""
    return repr(_number(block, "SPEED", "the speed", 1, 100) / 100.0)


def _visit(block, ctx):
    """Every block generated is counted and its id checked once: ids must be unique, the highlight depends on it."""
    bid = _id(block)
    if bid in ctx["ids"]:
        raise CodegenError(bid, "two blocks have the same id")
    ctx["ids"].add(bid)
    if len(ctx["ids"]) > MAX_BLOCKS:
        raise CodegenError(None, f"the program has more than {MAX_BLOCKS} blocks")
    return bid


def _id(block):
    bid = block.get("id")
    if not isinstance(bid, str) or not 0 < len(bid) <= MAX_ID_LEN:
        raise CodegenError(None, "a block has no valid id")
    return bid


def _disabled(block):
    return block.get("enabled") is False or bool(block.get("disabledReasons"))


def _unknown(block):
    kind = block.get("type")
    return f"this editor cannot run blocks of type {str(kind)[:40]!r}"
