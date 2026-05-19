"""Tests for FollowServer HTTP endpoints."""

import json
import socket
import threading
import time
from http.client import HTTPConnection

import pytest

from robot_follow.follow_api.state import FollowTargetState, SharedDetectionState
from robot_follow.servers import FollowServer


def _find_free_port():
    """Find an available port on localhost."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(('127.0.0.1', 0))
        s.listen(1)
        port = s.getsockname()[1]
    return port


class TestFollowServer:
    """Test FollowServer HTTP API."""

    @pytest.fixture
    def server(self):
        """Create and start a FollowServer on a test port."""
        target_state = FollowTargetState()
        shared_state = SharedDetectionState()
        port = _find_free_port()
        server = FollowServer(
            target_state=target_state,
            shared_state=shared_state,
            host="127.0.0.1",
            port=port
        )
        server.start()
        time.sleep(0.1)  # Give server time to start
        yield server
        server.stop()
        time.sleep(0.1)  # Give port time to be released

    def test_post_follow_sets_target(self, server):
        """POST /follow/<id> should set the target ID."""
        # Make ID 42 available in shared state
        server.shared_state.update(None, available_ids={42})
        
        conn = HTTPConnection("127.0.0.1", server.port)
        conn.request("POST", "/follow/42")
        response = conn.getresponse()
        data = json.loads(response.read().decode())
        
        assert response.status == 200
        assert data["status"] == "success"
        assert data["following_id"] == 42
        assert server.target_state.get_target() == 42

    def test_post_follow_updates_existing_target(self, server):
        """POST /follow/<id> should update existing target."""
        server.target_state.set_target(10)
        # Make ID 99 available
        server.shared_state.update(None, available_ids={10, 99})
        
        conn = HTTPConnection("127.0.0.1", server.port)
        conn.request("POST", "/follow/99")
        response = conn.getresponse()
        data = json.loads(response.read().decode())
        
        assert response.status == 200
        assert data["following_id"] == 99
        assert server.target_state.get_target() == 99

    def test_post_follow_clear_unsets_target(self, server):
        """POST /follow/clear should clear the target."""
        server.target_state.set_target(25)
        
        conn = HTTPConnection("127.0.0.1", server.port)
        conn.request("POST", "/follow/clear")
        response = conn.getresponse()
        data = json.loads(response.read().decode())
        
        assert response.status == 200
        assert data["status"] == "success"
        assert data["following_id"] is None
        assert server.target_state.get_target() is None

    def test_get_status_returns_current_target(self, server):
        """GET /status should return current target and last_seen."""
        server.target_state.set_target(17)
        
        conn = HTTPConnection("127.0.0.1", server.port)
        conn.request("GET", "/status")
        response = conn.getresponse()
        data = json.loads(response.read().decode())
        
        assert response.status == 200
        assert data["following_id"] == 17
        assert "last_seen" in data
        assert data["last_seen"] is not None

    def test_get_status_none_when_no_target(self, server):
        """GET /status should return None when no target set."""
        conn = HTTPConnection("127.0.0.1", server.port)
        conn.request("GET", "/status")
        response = conn.getresponse()
        data = json.loads(response.read().decode())
        
        assert response.status == 200
        assert data["following_id"] is None
        assert data["last_seen"] is None

    def test_get_status_includes_available_ids(self, server):
        """GET /status should include available_ids when shared_state provided."""
        server.shared_state.update(None, available_ids={1, 2, 3})
        
        conn = HTTPConnection("127.0.0.1", server.port)
        conn.request("GET", "/status")
        response = conn.getresponse()
        data = json.loads(response.read().decode())
        
        assert response.status == 200
        assert "available_ids" in data
        assert set(data["available_ids"]) == {1, 2, 3}

    def test_post_wrong_method_returns_error(self, server):
        """POST to /status should return 404."""
        conn = HTTPConnection("127.0.0.1", server.port)
        conn.request("POST", "/status")
        response = conn.getresponse()
        
        assert response.status == 404

    def test_get_wrong_path_returns_error(self, server):
        """GET to unknown path should return 404."""
        conn = HTTPConnection("127.0.0.1", server.port)
        conn.request("GET", "/nonexistent")
        response = conn.getresponse()
        
        assert response.status == 404

    def test_post_follow_invalid_id_returns_error(self, server):
        """POST /follow/<invalid> should return 400."""
        conn = HTTPConnection("127.0.0.1", server.port)
        conn.request("POST", "/follow/notanumber")
        response = conn.getresponse()
        # Server returns HTML error page, not JSON
        assert response.status == 400

    def test_post_follow_unavailable_id_returns_404(self, server):
        """POST /follow/<id> should return 404 if ID not available."""
        # Only IDs 1,2,3 are available, but we try to follow 99
        server.shared_state.update(None, available_ids={1, 2, 3})

        conn = HTTPConnection("127.0.0.1", server.port)
        conn.request("POST", "/follow/99")
        response = conn.getresponse()
        data = json.loads(response.read().decode())

        assert response.status == 404
        assert data["status"] == "error"
        assert "not found" in data["message"]
        assert set(data["available_ids"]) == {1, 2, 3}

    def test_post_follow_stale_id_with_single_available_returns_404(self, server):
        """03-15 revert pin: stale id + single visible person → strict 404 (no server-side recovery).

        The rejected 03-13 widening (commit 72add07) would have recovered to the single
        available id. Per operator decision (see feedback_click_to_follow_id_resolution.md),
        the right layer for stale-id resolution is the client (nearest-center match in
        handleFollow); the server stays strict so a future re-introduction of that
        widening will be caught by this test.
        """
        # Exactly one person is visible under id=45; client (incorrectly) sends stale id=1.
        # Server MUST NOT recover. Client owns this resolution now.
        server.shared_state.update(None, available_ids={45})

        conn = HTTPConnection("127.0.0.1", server.port)
        conn.request("POST", "/follow/1")
        response = conn.getresponse()
        data = json.loads(response.read().decode())

        assert response.status == 404
        assert data["status"] == "error"
        assert "not found" in data["message"]
        assert set(data["available_ids"]) == {45}
        assert server.target_state.get_target() is None

    def test_post_follow_stale_id_with_zero_available_returns_404(self, server):
        """F3 closure (03-13): zero available — no recovery candidate, preserve 404."""
        # No person in view — server cannot recover; preserve the existing 404 path.
        server.shared_state.update(None, available_ids=set())

        conn = HTTPConnection("127.0.0.1", server.port)
        conn.request("POST", "/follow/1")
        response = conn.getresponse()
        data = json.loads(response.read().decode())

        assert response.status == 404
        assert data["status"] == "error"
        assert "not found" in data["message"]
        assert data["available_ids"] == []
        assert server.target_state.get_target() is None

    def test_post_follow_stale_id_with_multiple_available_returns_404(self, server):
        """F3 closure (03-13): multiple available — ambiguous, preserve 404."""
        # Two visible persons — server cannot disambiguate which one the operator
        # meant; preserve the existing 404 path so the operator re-clicks the
        # (now-fresh) bbox they want.
        server.shared_state.update(None, available_ids={45, 67})

        conn = HTTPConnection("127.0.0.1", server.port)
        conn.request("POST", "/follow/1")
        response = conn.getresponse()
        data = json.loads(response.read().decode())

        assert response.status == 404
        assert data["status"] == "error"
        assert "not found" in data["message"]
        assert set(data["available_ids"]) == {45, 67}
        assert server.target_state.get_target() is None

    def test_post_follow_with_bbox_body_uses_body_h_for_setpoint(self, server):
        """03-15 F4b: POST body with bbox.h is written directly to controller_config.

        Bypasses the SharedUIState lookup that races with SharedDetectionState and
        produced 'bbox height n/a' INFO lines in live SITL. The client (post-03-13)
        already resolved the right detection and knows its bbox; just send it.
        """
        from robot_follow.follow_api.config import ControllerConfig
        from robot_follow.servers.follow_server import FollowServerHandler

        cfg = ControllerConfig()
        cfg.target_bbox_height = 0.25  # baseline — will be overwritten
        FollowServerHandler.controller_config = cfg
        try:
            server.shared_state.update(None, available_ids={45})

            conn = HTTPConnection("127.0.0.1", server.port)
            # Value picked inside capture_bbox_setpoint_from_height's clamp range [0.10, 0.25]
            body = json.dumps({"bbox": {"h": 0.18}}).encode("utf-8")
            conn.request(
                "POST",
                "/follow/45",
                body=body,
                headers={"Content-Type": "application/json", "Content-Length": str(len(body))},
            )
            response = conn.getresponse()
            data = json.loads(response.read().decode())

            assert response.status == 200
            assert data["status"] == "success"
            assert data["following_id"] == 45
            assert data["target_bbox_height"] == pytest.approx(0.18)
            assert cfg.target_bbox_height == pytest.approx(0.18)
        finally:
            FollowServerHandler.controller_config = None

    def test_post_follow_without_body_falls_back_to_ui_state_lookup(self, server):
        """03-15 F4b: POST without body falls back to _capture_bbox_for_id.

        Backward compat for curl-style callers and older clients. Without ui_state
        wired in the test fixture, _capture_bbox_for_id returns None and the
        response carries target_bbox_height: None (still 200; the lock succeeds).
        """
        from robot_follow.servers.follow_server import FollowServerHandler

        # No controller_config, no ui_state — both branches degrade to None.
        FollowServerHandler.controller_config = None
        FollowServerHandler.ui_state = None
        server.shared_state.update(None, available_ids={45})

        conn = HTTPConnection("127.0.0.1", server.port)
        conn.request("POST", "/follow/45")
        response = conn.getresponse()
        data = json.loads(response.read().decode())

        assert response.status == 200
        assert data["status"] == "success"
        assert data["following_id"] == 45
        assert data["target_bbox_height"] is None
        assert server.target_state.get_target() == 45

    def test_cors_headers_present(self, server):
        """Response should include CORS headers."""
        conn = HTTPConnection("127.0.0.1", server.port)
        conn.request("GET", "/status")
        response = conn.getresponse()

        headers = dict(response.getheaders())
        assert "Access-Control-Allow-Origin" in headers
        assert headers["Access-Control-Allow-Origin"] == "*"
