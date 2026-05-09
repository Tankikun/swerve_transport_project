"""
test_server_endpoints.py — Smoke test for interface/server.py HTTP endpoints.

Pure-Python; no ROS, no rclpy, no real network. Uses Flask's test_client
to exercise the routes directly. Runs in <2 seconds. Designed to catch
regressions in the server's contract with ros_pose_bridge.py and the
GUI without needing a robot.

Run from the repo root or from interface/:

    python3 -m pytest interface/test_server_endpoints.py -v
    # or, no pytest needed:
    python3 interface/test_server_endpoints.py

Covers:
  * /pose POST/GET demux by robot_id
  * /set_initial_pose queue + per-robot seq increment
  * /set_initial_pose/<id>/ack: happy path, no-pending-hint, stale-ack
  * /pose_hint_status/<id>: queued-but-not-acked, acked, never-queued
  * Multi-robot isolation: tb3_0 and tb3_1 hints don't bleed across
"""

from __future__ import annotations

import json
import sys
import time
import unittest
from pathlib import Path

# Make the sibling server.py importable when run as a script.
HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import server                                     # noqa: E402


class ServerEndpointTests(unittest.TestCase):

    def setUp(self):
        # Reset module-global state between tests so they don't leak
        # state into each other (Flask app is reused across tests).
        server.latest_poses.clear()
        server.pending_initial_poses.clear()
        server.latest_goal       = None
        # Stuff a tiny map so /map doesn't 500 if something queries it.
        server.map_data = {
            "metadata": {"point_count": 0, "bounds": None,
                         "axis_convention": "z-up"},
            "points":   [], "colors": [],
        }
        self.app = server.app
        self.app.testing = True
        self.c = self.app.test_client()

    # ── /pose ─────────────────────────────────────────────────────────────

    def test_pose_post_get_demux(self):
        body0 = {"robot_id": "tb3_0", "localized": True, "x": 1.0, "y": 2.0,
                 "yaw_rad": 0.0, "yaw_deg": 0.0, "frame": "map",
                 "last_match_age_sec": 0.5}
        body1 = {"robot_id": "tb3_1", "localized": True, "x": 3.0, "y": 4.0,
                 "yaw_rad": 0.5, "yaw_deg": 28.6, "frame": "map",
                 "last_match_age_sec": 0.6}
        self.assertEqual(200, self.c.post("/pose", json=body0).status_code)
        self.assertEqual(200, self.c.post("/pose", json=body1).status_code)

        r0 = self.c.get("/pose/tb3_0").get_json()
        r1 = self.c.get("/pose/tb3_1").get_json()
        self.assertTrue(r0["available"])
        self.assertAlmostEqual(r0["x"], 1.0)
        self.assertEqual(r0["robot_id"], "tb3_0")
        self.assertTrue(r1["available"])
        self.assertAlmostEqual(r1["x"], 3.0)

    def test_pose_get_unknown_robot(self):
        r = self.c.get("/pose/tb3_77").get_json()
        self.assertFalse(r["available"])
        self.assertEqual(r["robot_id"], "tb3_77")

    # ── /set_initial_pose queue + ack happy path ──────────────────────────

    def test_initialpose_queue_then_ack(self):
        # 1) GUI POSTs a hint.
        post = self.c.post("/set_initial_pose/tb3_0",
                           json={"x": 0.5, "y": -0.2, "yaw_rad": 0.1})
        self.assertEqual(200, post.status_code)
        seq = post.get_json()["seq"]
        self.assertEqual(seq, 1)

        # 2) Bridge polls — sees the hint, not yet acked.
        poll = self.c.get("/set_initial_pose/tb3_0").get_json()
        self.assertTrue(poll["available"])
        self.assertEqual(poll["seq"], 1)
        self.assertEqual(poll["acked_seq"], 0)

        # 3) GUI checks status — acked: false.
        st = self.c.get("/pose_hint_status/tb3_0").get_json()
        self.assertTrue(st["available"])
        self.assertEqual(st["seq"], 1)
        self.assertEqual(st["acked_seq"], 0)
        self.assertFalse(st["acked"])
        self.assertIsNone(st["age_since_ack_s"])
        self.assertIsNotNone(st["age_since_queue_s"])

        # 4) Bridge POSTs ack after publishing /initialpose.
        ack = self.c.post("/set_initial_pose/tb3_0/ack",
                          json={"seq": 1})
        self.assertEqual(200, ack.status_code)
        self.assertEqual(ack.get_json()["status"], "acked")

        # 5) GUI checks status again — now acked.
        st2 = self.c.get("/pose_hint_status/tb3_0").get_json()
        self.assertEqual(st2["seq"], 1)
        self.assertEqual(st2["acked_seq"], 1)
        self.assertTrue(st2["acked"])
        self.assertIsNotNone(st2["age_since_ack_s"])

    def test_seq_increments_per_robot(self):
        for _ in range(3):
            self.c.post("/set_initial_pose/tb3_0",
                        json={"x": 0, "y": 0, "yaw_rad": 0})
        for _ in range(2):
            self.c.post("/set_initial_pose/tb3_1",
                        json={"x": 1, "y": 1, "yaw_rad": 0})
        # Each robot has its own seq counter, independent of the other.
        self.assertEqual(self.c.get("/pose_hint_status/tb3_0").get_json()["seq"], 3)
        self.assertEqual(self.c.get("/pose_hint_status/tb3_1").get_json()["seq"], 2)

    def test_ack_for_unknown_robot(self):
        ack = self.c.post("/set_initial_pose/tb3_99/ack", json={"seq": 1})
        self.assertEqual(200, ack.status_code)
        self.assertEqual(ack.get_json()["status"], "no_pending_hint")

    def test_stale_ack_does_not_overwrite_newer(self):
        # Two hints in a row.
        self.c.post("/set_initial_pose/tb3_0",
                    json={"x": 0, "y": 0, "yaw_rad": 0})
        self.c.post("/set_initial_pose/tb3_0",
                    json={"x": 1, "y": 1, "yaw_rad": 0})
        # Bridge acks #2 first (network reordering).
        ack2 = self.c.post("/set_initial_pose/tb3_0/ack",
                           json={"seq": 2}).get_json()
        self.assertEqual(ack2["status"], "acked")
        # Then a late ack for #1 arrives — must NOT roll the ack back.
        ack1 = self.c.post("/set_initial_pose/tb3_0/ack",
                           json={"seq": 1}).get_json()
        self.assertEqual(ack1["status"], "stale_ack")
        self.assertEqual(ack1["current_acked_seq"], 2)
        # Status reflects #2 acked, not #1.
        st = self.c.get("/pose_hint_status/tb3_0").get_json()
        self.assertEqual(st["acked_seq"], 2)
        self.assertTrue(st["acked"])

    def test_ack_missing_seq_returns_400(self):
        self.c.post("/set_initial_pose/tb3_0",
                    json={"x": 0, "y": 0, "yaw_rad": 0})
        r = self.c.post("/set_initial_pose/tb3_0/ack", json={})
        self.assertEqual(400, r.status_code)

    def test_pose_hint_status_never_queued(self):
        st = self.c.get("/pose_hint_status/tb3_never").get_json()
        self.assertFalse(st["available"])
        self.assertEqual(st["seq"], 0)
        self.assertEqual(st["acked_seq"], 0)
        self.assertFalse(st["acked"])

    def test_per_robot_isolation(self):
        # Hint for tb3_0; bridge for tb3_1 acks (wrong target).
        self.c.post("/set_initial_pose/tb3_0",
                    json={"x": 0, "y": 0, "yaw_rad": 0})
        # tb3_1 has no pending hint → ack returns no_pending_hint, doesn't
        # accidentally bleed into tb3_0.
        ack = self.c.post("/set_initial_pose/tb3_1/ack",
                          json={"seq": 1}).get_json()
        self.assertEqual(ack["status"], "no_pending_hint")
        # tb3_0 still un-acked.
        st = self.c.get("/pose_hint_status/tb3_0").get_json()
        self.assertFalse(st["acked"])

    def test_legacy_default_queue_still_works(self):
        # POST without robot_id → "default" bucket; legacy GET returns it.
        post = self.c.post("/set_initial_pose",
                           json={"x": 1, "y": 2, "yaw_rad": 0})
        self.assertEqual(200, post.status_code)
        self.assertEqual(post.get_json()["robot_id"], "default")
        legacy = self.c.get("/set_initial_pose").get_json()
        self.assertTrue(legacy["available"])
        self.assertEqual(legacy["robot_id"], "default")
        # Per-robot status endpoint also reads from "default" by id.
        st = self.c.get("/pose_hint_status/default").get_json()
        self.assertTrue(st["available"])

    def test_age_since_queue_is_nondecreasing(self):
        self.c.post("/set_initial_pose/tb3_0",
                    json={"x": 0, "y": 0, "yaw_rad": 0})
        st1 = self.c.get("/pose_hint_status/tb3_0").get_json()
        time.sleep(0.05)
        st2 = self.c.get("/pose_hint_status/tb3_0").get_json()
        self.assertGreaterEqual(st2["age_since_queue_s"],
                                st1["age_since_queue_s"])


if __name__ == "__main__":
    # Allow running without pytest. unittest's verbosity=2 mirrors `-v`.
    unittest.main(verbosity=2)
