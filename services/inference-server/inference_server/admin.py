"""
Lightweight HTTP admin endpoint for the inference server.

Runs on a separate port (default: TCP port + 1) using raw asyncio —
no framework dependencies. Supports:

    GET  /status   → JSON stats (frames, epochs, workers, uptime)
    POST /restart  → git pull + os.execv to restart with updated code
    POST /shutdown → clean server shutdown
    POST /print    → run print_receipt with dimension values

All endpoints return JSON. No auth — this is for local-network admin only.
"""

from __future__ import annotations

import asyncio
import json
import logging
import os
import subprocess
import sys
import time
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .server import InferenceServer

log = logging.getLogger(__name__)

_start_time = time.monotonic()


def _json_response(status: int, body: dict) -> bytes:
    """Build a minimal HTTP/1.1 response with JSON body."""
    payload = json.dumps(body).encode()
    reason = {200: "OK", 202: "Accepted", 400: "Bad Request", 404: "Not Found", 405: "Method Not Allowed"}
    lines = [
        f"HTTP/1.1 {status} {reason.get(status, 'Unknown')}",
        "Content-Type: application/json",
        f"Content-Length: {len(payload)}",
        "Connection: close",
        "",
        "",
    ]
    return "\r\n".join(lines).encode() + payload


async def _handle_admin(
    reader: asyncio.StreamReader,
    writer: asyncio.StreamWriter,
    server: InferenceServer,
) -> None:
    """Parse one HTTP request and dispatch to the right handler."""
    try:
        request_line = await asyncio.wait_for(reader.readline(), timeout=5.0)
    except (asyncio.TimeoutError, ConnectionError):
        writer.close()
        return

    if not request_line:
        writer.close()
        return

    parts = request_line.decode(errors="replace").strip().split()
    if len(parts) < 2:
        writer.write(_json_response(400, {"error": "bad request"}))
        await writer.drain()
        writer.close()
        return

    method, path = parts[0].upper(), parts[1]

    # Read headers and extract Content-Length
    content_length = 0
    while True:
        line = await reader.readline()
        if line in (b"\r\n", b"\n", b""):
            break
        header = line.decode(errors="replace").strip().lower()
        if header.startswith("content-length:"):
            try:
                content_length = int(header.split(":", 1)[1].strip())
            except ValueError:
                pass

    # Read body if present
    body = b""
    if content_length > 0:
        body = await asyncio.wait_for(reader.readexactly(content_length), timeout=5.0)

    if path == "/status":
        if method != "GET":
            resp = _json_response(405, {"error": "use GET"})
        else:
            resp = _json_response(200, _build_status(server))
    elif path == "/restart":
        if method != "POST":
            resp = _json_response(405, {"error": "use POST"})
        else:
            resp = await _handle_restart(server)
    elif path == "/print":
        if method != "POST":
            resp = _json_response(405, {"error": "use POST"})
        else:
            resp = await _handle_print(body)
    elif path == "/shutdown":
        if method != "POST":
            resp = _json_response(405, {"error": "use POST"})
        else:
            resp = _json_response(202, {"status": "shutting down"})
            writer.write(resp)
            await writer.drain()
            writer.close()
            server.shutdown()
            return
    else:
        resp = _json_response(404, {"error": "not found", "endpoints": ["/status", "/restart", "/shutdown", "/print"]})

    writer.write(resp)
    await writer.drain()
    writer.close()


def _build_status(server: InferenceServer) -> dict:
    uptime = time.monotonic() - _start_time
    return {
        "status": "running",
        "uptime_secs": round(uptime, 1),
        "echo_mode": server.echo_mode,
        "workers": server.num_workers,
        "workers_loaded": server._workers_loaded,
        "frames_received": server.frames_received,
        "epochs_queued": server.epochs_queued,
        "epochs_processed": server.epochs_processed,
        "epochs_dropped": server.epochs_dropped,
        "client_connected": server._writer is not None,
    }


_VALID_DIMS = {
    "absorption": ("deep", "surface"),
    "attunement": ("porous", "boundaried"),
    "unknown": ("lean_in", "hold_back"),
    "witnessed": ("approach", "withdraw"),
}


async def _handle_print(body: bytes) -> bytes:
    """Run print_receipt with dimension values from JSON body."""
    try:
        data = json.loads(body)
    except (json.JSONDecodeError, ValueError):
        return _json_response(400, {"error": "invalid JSON body"})

    if not isinstance(data, dict):
        return _json_response(400, {"error": "expected JSON object"})

    # Validate and build CLI args
    cli_args = []
    for dim, valid_values in _VALID_DIMS.items():
        val = data.get(dim)
        if val is not None:
            if val not in valid_values:
                return _json_response(400, {
                    "error": f"invalid value '{val}' for {dim}",
                    "valid": list(valid_values),
                })
            cli_args.extend([f"--{dim}", val])

    if not cli_args:
        return _json_response(400, {"error": "at least one dimension required"})

    # Find print-glyph project relative to this file
    repo_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    print_glyph_dir = os.path.join(repo_root, "print-glyph")

    cmd = ["uv", "run", "--project", print_glyph_dir,
           "python", "-m", "print_glyph.print_receipt"] + cli_args
    log.info("Print: %s", " ".join(cmd))

    try:
        result = await asyncio.to_thread(
            subprocess.run, cmd,
            capture_output=True, text=True, timeout=60,
        )
        output = result.stdout.strip()
        if result.stderr.strip():
            output += "\n" + result.stderr.strip()

        if result.returncode == 0:
            log.info("Print done: %s", output)
            return _json_response(200, {"status": "printed", "output": output})
        else:
            log.warning("Print failed: %s", output)
            return _json_response(200, {"status": "print_failed", "output": output})
    except subprocess.TimeoutExpired:
        return _json_response(200, {"status": "print_timeout"})
    except FileNotFoundError:
        return _json_response(200, {"status": "uv_not_found", "output": "uv not on PATH"})


async def _handle_restart(server: InferenceServer) -> bytes:
    """Git pull in the repo root, then re-exec the current process."""
    # Find repo root (walk up from this file)
    repo_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

    # Step 1: git pull
    log.info("Admin restart: running git pull in %s", repo_root)
    try:
        result = subprocess.run(
            ["git", "pull", "--ff-only"],
            cwd=repo_root,
            capture_output=True,
            text=True,
            timeout=30,
        )
        git_output = result.stdout.strip()
        git_ok = result.returncode == 0
        if not git_ok:
            git_output = result.stderr.strip() or git_output
            log.warning("git pull failed: %s", git_output)
            return _json_response(200, {
                "status": "git_pull_failed",
                "git_output": git_output,
                "restarting": False,
            })
        log.info("git pull: %s", git_output)
    except subprocess.TimeoutExpired:
        return _json_response(200, {
            "status": "git_pull_timeout",
            "restarting": False,
        })

    # Step 2: Clean shutdown then re-exec
    log.info("Admin restart: shutting down server and re-exec'ing")
    response = _json_response(202, {
        "status": "restarting",
        "git_output": git_output,
    })

    # Schedule the actual restart after the response is sent
    asyncio.get_running_loop().call_later(0.5, _do_restart, server)
    return response


def _do_restart(server: InferenceServer) -> None:
    """Shut down workers and re-exec the process with the same arguments."""
    log.info("Executing restart...")
    server.shutdown()

    # Reconstruct the original command line.
    # When launched as `python -m inference_server.server`, sys.argv[0] is the
    # path to server.py but we need `-m inference_server.server` for relative
    # imports to work.  Detect this by checking if argv[0] ends with our module.
    argv0 = sys.argv[0]
    if argv0.endswith("inference_server/server.py") or argv0.endswith("inference_server\\server.py"):
        argv = [sys.executable, "-m", "inference_server.server"] + sys.argv[1:]
    else:
        argv = [sys.executable] + sys.argv

    log.info("Re-exec: %s", " ".join(argv))
    os.execv(sys.executable, argv)


async def start_admin_server(
    server: InferenceServer,
    port: int,
    host: str = "0.0.0.0",
) -> asyncio.Server:
    """Start the admin HTTP server. Returns the asyncio.Server handle."""

    async def handler(reader: asyncio.StreamReader, writer: asyncio.StreamWriter) -> None:
        await _handle_admin(reader, writer, server)

    admin_server = await asyncio.start_server(handler, host, port)
    addrs = ", ".join(str(s.getsockname()) for s in admin_server.sockets)
    log.info("Admin HTTP server listening on %s", addrs)
    return admin_server
