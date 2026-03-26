"""Tests for the admin HTTP endpoint."""

import asyncio
import json

import pytest
import pytest_asyncio

from inference_server.server import InferenceServer
from inference_server.admin import start_admin_server


async def _http_request(port: int, method: str, path: str) -> tuple[int, dict]:
    """Send a minimal HTTP request and parse the JSON response."""
    reader, writer = await asyncio.open_connection("127.0.0.1", port)
    request = f"{method} {path} HTTP/1.1\r\nHost: localhost\r\n\r\n"
    writer.write(request.encode())
    await writer.drain()

    response = await asyncio.wait_for(reader.read(4096), timeout=5.0)
    writer.close()

    # Parse status code from first line
    first_line = response.split(b"\r\n")[0].decode()
    status = int(first_line.split()[1])

    # Parse JSON body (after blank line)
    body_start = response.find(b"\r\n\r\n")
    body = json.loads(response[body_start + 4:])
    return status, body


@pytest_asyncio.fixture
async def admin_server():
    """Start an InferenceServer in echo mode with admin endpoint."""
    server = InferenceServer(port=19100, echo_mode=True, admin_port=19101)
    admin = await start_admin_server(server, 19101)
    yield server, 19101
    admin.close()
    await admin.wait_closed()


class TestAdminStatus:
    @pytest.mark.asyncio
    async def test_status_returns_json(self, admin_server):
        server, port = admin_server
        status, body = await _http_request(port, "GET", "/status")
        assert status == 200
        assert body["status"] == "running"
        assert body["echo_mode"] is True
        assert "uptime_secs" in body
        assert "frames_received" in body

    @pytest.mark.asyncio
    async def test_status_rejects_post(self, admin_server):
        _, port = admin_server
        status, body = await _http_request(port, "POST", "/status")
        assert status == 405


class TestAdminNotFound:
    @pytest.mark.asyncio
    async def test_unknown_path(self, admin_server):
        _, port = admin_server
        status, body = await _http_request(port, "GET", "/nonexistent")
        assert status == 404
        assert "endpoints" in body
