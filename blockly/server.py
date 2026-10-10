"""The block editor's web side: its page and a small HTTP API, mountable on ANY FastAPI app.

A host GUI integrates it in two lines (TODO/blockly_plan.md §4) — NiceGUI's `app` is a FastAPI app:

    from blockly.server import mount
    mount(app)                          # then show an iframe with src="/blockly/"

or it runs on its own:

    python3 -m blockly.server [--host 0.0.0.0] [--port 8084] [--robot 127.0.0.1]   # from the repo root

API, all JSON under <prefix>/api. Success is {"ok": true, ...}; a request that cannot be done is {"ok": false,
"message": "..."} with HTTP 400 (bad input), 404 (no such program) or 409 (not now: a program is running / none is):
    POST   codegen                 {"workspace", "name", "points"} -> {"ok", "python"} or {"ok": false, "block", "message"}
                                   (a program that cannot run is an ANSWER, HTTP 200 — it names the block to show)
    POST   check                   {"workspace", "name", "points"} -> {"result": "passed" | "limited" | "failed",
                                   "error": {block, code, message} | null, "summary", "seconds"} — the real planner,
                                   offline, from where the arm is (blockly/check.py); 409 while busy
    GET    programs                -> {"programs": [{"name", "modified", "bytes"}]}
    GET    programs/<name>         -> {"program": {...}}
    PUT    programs/<name>         {"workspace", "points"} -> {"modified"}
    DELETE programs/<name>
    GET    robot/position          -> {"joints_deg": [6], "pose": [x, y, z (m), rx, ry, rz (rad)]}
    POST   run                     {"name"} -> the run snapshot (the program must be saved: the robot runs the file)
    POST   run/move_here           {"name", "point": {"joints_deg", "pose"}} -> the run snapshot: a joint move to the
                                   point at 10 %, a one-block program the SERVER builds ("Move here" in the Points list)
    POST   run/pause | run/resume | run/step | run/stop       (stop also works with no program running)
    GET    run/events?since=N      -> {"events": [...], "gap", "running", "program", "state", "block", "error", "exit", "seq"}
"""
import argparse
import json
import os
import threading

from fastapi import APIRouter, Request
from fastapi.responses import JSONResponse
from fastapi.staticfiles import StaticFiles
from starlette.concurrency import run_in_threadpool

from blockly import programs, runs
from blockly.codegen import CodegenError, generate

WEB_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "web")
DEFAULT_PORT = 8084     # next to the motor tuner (8082) and the firmware flasher (8083); the dashboard is 8081


def mount(app, prefix="/blockly", robot_host=None, robot_port=None):
    """Serve the editor page at <prefix>/ and its API at <prefix>/api/ on `app` (any FastAPI app).

    The API goes in FIRST: the static mount matches every path under the prefix, so a route added after it would
    never be reached.
    """
    runs.configure(host=robot_host, port=robot_port)
    router = APIRouter(prefix=prefix + "/api")
    router.add_api_route("/codegen", codegen, methods=["POST"])
    router.add_api_route("/check", check, methods=["POST"])
    router.add_api_route("/programs", list_programs, methods=["GET"])
    router.add_api_route("/programs/{name}", get_program, methods=["GET"])
    router.add_api_route("/programs/{name}", put_program, methods=["PUT"])
    router.add_api_route("/programs/{name}", delete_program, methods=["DELETE"])
    router.add_api_route("/robot/position", robot_position, methods=["GET"])
    router.add_api_route("/run", run_start, methods=["POST"])
    router.add_api_route("/run/events", run_events, methods=["GET"])
    router.add_api_route("/run/move_here", run_move_here, methods=["POST"])     # before /run/{action}
    router.add_api_route("/run/{action}", run_control, methods=["POST"])
    app.include_router(router)
    app.mount(prefix, FreshStaticFiles(directory=WEB_DIR, html=True), name="blockly")


class FreshStaticFiles(StaticFiles):
    """The editor's files with `Cache-Control: no-cache`: the browser keeps its copy but asks every time whether it
    changed (a cheap 304 when not). Without it a browser re-used its cached editor page inside the dashboard's iframe
    after an update — the new "Move here" button did not appear (2026-10-05): with no cache header, browsers guess a
    freshness time, and a reload re-checks only the top page, not the frames inside it."""

    async def get_response(self, path, scope):
        response = await super().get_response(path, scope)
        response.headers["Cache-Control"] = "no-cache"
        return response


# ── endpoints. Plain `def`s run in FastAPI's worker threads; the async ones read the raw body (see _parse) ─────

async def codegen(request: Request):
    return await run_in_threadpool(codegen_reply, await request.body())


def codegen_reply(raw):
    body, bad = _parse(raw)
    if bad is not None:
        return bad
    try:
        return {"ok": True, "python": generate(body.get("workspace"), body.get("name", "program"),
                                               body.get("points"))}
    except CodegenError as e:
        return {"ok": False, "block": e.block_id, "message": e.message}


_check_lock = threading.Lock()


async def check(request: Request):
    return await run_in_threadpool(check_reply, await request.body())


def check_reply(raw):
    """A Check: one at a time (it loads the planner — blockly/check.py)."""
    body, bad = _parse(raw)
    if bad is not None:
        return bad
    if not _check_lock.acquire(blocking=False):
        return _no("a Check is already running", 409)
    try:
        return {"ok": True, **runs.check(body.get("name") or "program", body.get("workspace"), body.get("points"))}
    except programs.ProgramError as e:
        return _no(e, 400)
    except runs.RunError as e:
        return _no(e, 409)
    finally:
        _check_lock.release()


def list_programs():
    return {"ok": True, "programs": programs.list_programs()}


def get_program(name: str):
    try:
        return {"ok": True, "program": programs.load_program(name)}
    except programs.ProgramError as e:
        return _no(e, 404 if "there is no program" in str(e) else 400)


async def put_program(name: str, request: Request):
    return await run_in_threadpool(_save, name, await request.body())


def _save(name, raw):
    body, bad = _parse(raw)
    if bad is not None:
        return bad
    if runs.running() and runs.snapshot()["program"] == name:
        return _no("this program is running — stop it before saving over it", 409)
    try:
        saved = programs.save_program(name, body)
    except programs.ProgramError as e:
        return _no(e, 400)
    return {"ok": True, "modified": saved["modified"]}


def delete_program(name: str):
    if runs.running() and runs.snapshot()["program"] == name:
        return _no("this program is running — stop it first", 409)
    try:
        programs.delete_program(name)
    except programs.ProgramError as e:
        return _no(e, 404 if "there is no program" in str(e) else 400)
    return {"ok": True}


def robot_position():
    try:
        return {"ok": True, **runs.position()}
    except runs.RunError as e:
        return _no(e, 409)


async def run_start(request: Request):
    body, bad = _parse(await request.body())
    if bad is not None:
        return bad
    try:
        return {"ok": True, **await run_in_threadpool(runs.start, body.get("name"))}
    except (runs.RunError, programs.ProgramError) as e:
        return _no(e, 409)


async def run_move_here(request: Request):
    body, bad = _parse(await request.body())
    if bad is not None:
        return bad
    try:
        return {"ok": True, **await run_in_threadpool(runs.move_here, body.get("name"), body.get("point"))}
    except programs.ProgramError as e:
        return _no(e, 400)
    except runs.RunError as e:
        return _no(e, 409)


def run_control(action: str):
    if action == "stop":
        return {"ok": True, **runs.stop()}
    if action not in ("pause", "resume", "step"):
        return _no(f"unknown action {action[:20]!r}", 404)
    try:
        getattr(runs, action)()
    except runs.RunError as e:
        return _no(e, 409)
    return {"ok": True}


def run_events(since: int = 0):
    return {"ok": True, **runs.events_since(since)}


# ── helpers ─────────────────────────────────────────────────────────────────────────────────────────────────────

def _parse(raw):
    """(the JSON object, None) or (None, the error response). Parsed HERE, not by FastAPI: Blockly nests every next
    block inside the one before it, and Python's json gives up (RecursionError) near 490 blocks in one stack — that
    must be an answer, not an HTTP 500."""
    try:
        body = json.loads(raw)
    except RecursionError:
        return None, {"ok": False, "block": None,
                      "message": "the program is nested too deeply to read (one stack of about 490 blocks or more)"}
    except ValueError:
        return None, _no("the request is not JSON", 400)
    if not isinstance(body, dict):
        return None, _no("the request is not a JSON object", 400)
    return body, None


def _no(message, status):
    return JSONResponse({"ok": False, "block": None, "message": str(message)}, status_code=status)


def main():
    ap = argparse.ArgumentParser(description="The RCB block editor on its own port (no dashboard needed).")
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    ap.add_argument("--robot", default="127.0.0.1", help="the robot's command_executor host")
    args = ap.parse_args()

    import uvicorn
    from fastapi import FastAPI
    from fastapi.responses import RedirectResponse

    app = FastAPI(title="RCB block editor", docs_url=None, redoc_url=None)
    mount(app, robot_host=args.robot)
    app.add_api_route("/", lambda: RedirectResponse("/blockly/"), include_in_schema=False)
    uvicorn.run(app, host=args.host, port=args.port, log_level="warning")


if __name__ == "__main__":
    main()
