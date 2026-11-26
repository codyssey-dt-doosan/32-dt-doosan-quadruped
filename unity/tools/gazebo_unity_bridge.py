#!/usr/bin/env python3
"""Lightweight native ROS 2 <-> Unity <-> Three.js bridge (rosbridge WS :9090 + MJPEG :8090).

Runs directly inside conda ros_env (no Docker / rosbridge_suite package needed).
Bridges Gazebo MPC (/odom, /joint_states, /cmd_vel, ...) with Unity HDRP mirror
and Three.js monitoring dashboard (roslib.js + web_video_server MJPEG compatible).
"""
import argparse
import asyncio
import base64
import json
import math
import signal
import threading
import time
from typing import Any

from aiohttp import WSMsgType, web
import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Bool, Float32, String


def ros_stamp(stamp) -> dict:
    return {"sec": int(stamp.sec), "nanosec": int(stamp.nanosec)}


def odom_to_dict(msg: Odometry) -> dict:
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    lv = msg.twist.twist.linear
    av = msg.twist.twist.angular
    return {
        "header": {"stamp": ros_stamp(msg.header.stamp), "frame_id": msg.header.frame_id},
        "child_frame_id": msg.child_frame_id,
        "pose": {
            "pose": {
                "position": {"x": float(p.x), "y": float(p.y), "z": float(p.z)},
                "orientation": {"x": float(q.x), "y": float(q.y), "z": float(q.z), "w": float(q.w)},
            }
        },
        "twist": {
            "twist": {
                "linear": {"x": float(lv.x), "y": float(lv.y), "z": float(lv.z)},
                "angular": {"x": float(av.x), "y": float(av.y), "z": float(av.z)},
            }
        },
    }


def joint_state_to_dict(msg: JointState) -> dict:
    return {
        "header": {"stamp": ros_stamp(msg.header.stamp), "frame_id": msg.header.frame_id},
        "name": list(msg.name),
        "position": [float(x) for x in msg.position],
        "velocity": [float(x) for x in msg.velocity],
        "effort": [float(x) for x in msg.effort],
    }


def pose_stamped_to_dict(msg: PoseStamped) -> dict:
    p = msg.pose.position
    q = msg.pose.orientation
    return {
        "header": {"stamp": ros_stamp(msg.header.stamp), "frame_id": msg.header.frame_id},
        "pose": {
            "position": {"x": float(p.x), "y": float(p.y), "z": float(p.z)},
            "orientation": {"x": float(q.x), "y": float(q.y), "z": float(q.z), "w": float(q.w)},
        },
    }


def render_thermal_jpeg(pixels: np.ndarray, alarm_c: float = 80.0, quality: int = 75) -> bytes | None:
    if pixels.size == 0 or not np.isfinite(pixels).all():
        return None
    tmax = float(pixels.max())
    norm = np.uint8(np.clip((pixels - 10.0) / 90.0, 0.0, 1.0) * 255)
    heat = cv2.resize(cv2.applyColorMap(norm, cv2.COLORMAP_INFERNO), (320, 240), interpolation=cv2.INTER_NEAREST)
    bar = cv2.applyColorMap(np.linspace(255, 0, 200, dtype=np.uint8).reshape(200, 1), cv2.COLORMAP_INFERNO)
    heat[20:220, 302:312] = bar
    cv2.rectangle(heat, (301, 19), (312, 220), (200, 200, 200), 1)
    hot = tmax >= alarm_c
    label = f"MAX {tmax:.1f}C" + (" HOT" if hot else "")
    cv2.putText(heat, label, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(heat, label, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (60, 220, 255) if not hot else (60, 80, 255), 2, cv2.LINE_AA)
    ok, buf = cv2.imencode(".jpg", heat, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
    return buf.tobytes() if ok else None


class FrameStore:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._frames: dict[str, tuple[bytes, int, float]] = {}

    def put(self, topic: str, jpeg: bytes, source_priority: int = 1) -> None:
        now = time.monotonic()
        with self._lock:
            cur = self._frames.get(topic)
            # Let Unity frames (priority 2) take precedence over Gazebo fallback (priority 1) while fresh
            if cur is not None and cur[1] > source_priority and (now - cur[2]) < 2.0:
                return
            self._frames[topic] = (jpeg, source_priority, now)

    def get(self, topic: str) -> tuple[bytes | None, float]:
        with self._lock:
            item = self._frames.get(topic)
            if not item:
                return None, 0.0
            return item[0], item[2]

    def topics(self) -> list[str]:
        with self._lock:
            return sorted(self._frames.keys())


class BridgeHub:
    def __init__(self, loop: asyncio.AbstractEventLoop) -> None:
        self.loop = loop
        self.clients: dict[web.WebSocketResponse, dict[str, dict[str, Any]]] = {}
        self.frames = FrameStore()
        self.ros_node: "GazeboUnityBridgeNode | None" = None
        self._last_sent: dict[tuple[int, str], float] = {}

    def register(self, ws: web.WebSocketResponse) -> None:
        self.clients[ws] = {}

    def unregister(self, ws: web.WebSocketResponse) -> None:
        self.clients.pop(ws, None)
        ws_id = id(ws)
        for k in [k for k in self._last_sent if k[0] == ws_id]:
            self._last_sent.pop(k, None)

    def subscribe(self, ws: web.WebSocketResponse, topic: str, throttle_ms: float = 0.0) -> None:
        if ws in self.clients:
            self.clients[ws][topic] = {"throttle_s": max(0.0, float(throttle_ms) / 1000.0)}

    def unsubscribe(self, ws: web.WebSocketResponse, topic: str) -> None:
        if ws in self.clients:
            self.clients[ws].pop(topic, None)

    def broadcast_from_thread(self, topic: str, msg_dict: dict, exclude_ws: web.WebSocketResponse | None = None) -> None:
        if self.loop.is_closed():
            return
        asyncio.run_coroutine_threadsafe(self.broadcast(topic, msg_dict, exclude_ws), self.loop)

    async def broadcast(self, topic: str, msg_dict: dict, exclude_ws: web.WebSocketResponse | None = None) -> None:
        now = time.monotonic()
        payload = None
        dead = []
        for ws, subs in list(self.clients.items()):
            if ws is exclude_ws or topic not in subs:
                continue
            throttle_s = subs[topic]["throttle_s"]
            key = (id(ws), topic)
            if throttle_s > 0.0 and (now - self._last_sent.get(key, 0.0)) < throttle_s:
                continue
            if payload is None:
                payload = json.dumps({"op": "publish", "topic": topic, "msg": msg_dict}, separators=(",", ":"))
            try:
                await ws.send_str(payload)
                self._last_sent[key] = now
            except Exception:
                dead.append(ws)
        for ws in dead:
            self.unregister(ws)

    def handle_unity_publish(self, ws: web.WebSocketResponse, topic: str, msg: dict) -> None:
        if topic == "/cmd_vel" and self.ros_node is not None:
            lin = msg.get("linear") or {}
            ang = msg.get("angular") or {}
            self.ros_node.publish_cmd_vel(float(lin.get("x", 0.0)), float(lin.get("y", 0.0)), float(ang.get("z", 0.0)))
            return

        if topic == "/camera/image":
            raw_b64 = msg.get("data")
            w, h = int(msg.get("width", 0)), int(msg.get("height", 0))
            if raw_b64 and w > 0 and h > 0:
                try:
                    raw = base64.b64decode(raw_b64)
                    arr = np.frombuffer(raw, dtype=np.uint8)
                    if arr.size == w * h * 3:
                        bgr = cv2.cvtColor(arr.reshape(h, w, 3), cv2.COLOR_RGB2BGR)
                        ok, buf = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
                        if ok:
                            self.frames.put("/camera/image", buf.tobytes(), source_priority=2)
                except Exception:
                    pass
            return

        if topic == "/thermal/image":
            raw_b64 = msg.get("data")
            w, h = int(msg.get("width", 0)), int(msg.get("height", 0))
            if raw_b64 and w > 0 and h > 0:
                try:
                    raw = base64.b64decode(raw_b64)
                    dtype = ">f4" if msg.get("is_bigendian") else "<f4"
                    arr = np.frombuffer(raw, dtype=dtype)
                    if arr.size == w * h:
                        pixels = arr.reshape(h, w)
                        jpg = render_thermal_jpeg(pixels)
                        if jpg:
                            self.frames.put("/thermal/color", jpg, source_priority=2)
                            self.frames.put("/thermal/image", jpg, source_priority=2)
                        hot = bool(float(pixels.max()) >= 80.0)
                        asyncio.ensure_future(self.broadcast("/inspection/thermal_alert", {"data": hot}, exclude_ws=ws))
                except Exception:
                    pass
            return

        if topic == "/gas/concentration":
            asyncio.ensure_future(self.broadcast(topic, msg, exclude_ws=ws))
            return

        if topic in ("/mission/status", "/inspection/gauge", "/inspection/thermal_alert"):
            asyncio.ensure_future(self.broadcast(topic, msg, exclude_ws=ws))


class GazeboUnityBridgeNode(Node):
    def __init__(self, hub: BridgeHub) -> None:
        super().__init__("gazebo_unity_bridge")
        self.hub = hub
        self.hub.ros_node = self
        self._last_odom = 0.0
        self._last_joint = 0.0
        self._odom_count = 0
        self._joint_count = 0

        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", 10)

        self.create_subscription(Odometry, "/odom", self._on_odom, qos_profile_sensor_data)
        self.create_subscription(JointState, "/joint_states", self._on_joints, qos_profile_sensor_data)
        self.create_subscription(String, "/mission/status", lambda m: hub.broadcast_from_thread("/mission/status", {"data": m.data}), 10)
        self.create_subscription(String, "/fall_recovery/status", lambda m: hub.broadcast_from_thread("/fall_recovery/status", {"data": m.data}), 10)
        self.create_subscription(PoseStamped, "/patrol/goal", lambda m: hub.broadcast_from_thread("/patrol/goal", pose_stamped_to_dict(m)), 10)
        self.create_subscription(PoseStamped, "/return_to_home/goal", lambda m: hub.broadcast_from_thread("/return_to_home/goal", pose_stamped_to_dict(m)), 10)
        self.create_subscription(PoseStamped, "/source_seeking/goal", lambda m: hub.broadcast_from_thread("/source_seeking/goal", pose_stamped_to_dict(m)), 10)
        self.create_subscription(Float32, "/inspection/gauge", lambda m: hub.broadcast_from_thread("/inspection/gauge", {"data": float(m.data)}), 10)
        self.create_subscription(Bool, "/inspection/thermal_alert", lambda m: hub.broadcast_from_thread("/inspection/thermal_alert", {"data": bool(m.data)}), 10)
        self.create_subscription(Float32, "/gas/concentration", lambda m: hub.broadcast_from_thread("/gas/concentration", {"data": float(m.data)}), 10)
        # Fallback Gazebo camera streams (overridden automatically whenever Unity streams higher-priority frames)
        self.create_subscription(Image, "/camera/image", self._on_gz_rgb, qos_profile_sensor_data)
        self.create_subscription(Image, "/thermal/image", self._on_gz_thermal, qos_profile_sensor_data)

        self.create_timer(5.0, self._log_stats)

    def publish_cmd_vel(self, vx: float, vy: float, wz: float) -> None:
        if not (math.isfinite(vx) and math.isfinite(vy) and math.isfinite(wz)):
            return
        msg = Twist()
        msg.linear.x = max(-1.5, min(1.5, vx))
        msg.linear.y = max(-0.5, min(0.5, vy))
        msg.angular.z = max(-2.0, min(2.0, wz))
        self.pub_cmd.publish(msg)

    def _on_odom(self, msg: Odometry) -> None:
        now = time.monotonic()
        if now - self._last_odom < 0.02:  # cap at 50 Hz
            return
        self._last_odom = now
        self._odom_count += 1
        self.hub.broadcast_from_thread("/odom", odom_to_dict(msg))

    def _on_joints(self, msg: JointState) -> None:
        now = time.monotonic()
        if now - self._last_joint < 0.02:  # cap at 50 Hz
            return
        self._last_joint = now
        self._joint_count += 1
        self.hub.broadcast_from_thread("/joint_states", joint_state_to_dict(msg))

    def _on_gz_rgb(self, msg: Image) -> None:
        if msg.encoding not in ("rgb8", "bgr8") or msg.width == 0 or msg.height == 0:
            return
        arr = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        if arr.size != msg.width * msg.height * 3:
            return
        img = arr.reshape(msg.height, msg.width, 3)
        if msg.encoding == "rgb8":
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 65])
        if ok:
            self.hub.frames.put("/camera/image", buf.tobytes(), source_priority=1)

    def _on_gz_thermal(self, msg: Image) -> None:
        if msg.width == 0 or msg.height == 0:
            return
        if msg.encoding == "32FC1":
            dtype = ">f4" if msg.is_bigendian else "<f4"
            arr = np.frombuffer(bytes(msg.data), dtype=dtype)
            if arr.size == msg.width * msg.height:
                jpg = render_thermal_jpeg(arr.reshape(msg.height, msg.width))
                if jpg:
                    self.hub.frames.put("/thermal/color", jpg, source_priority=1)
                    self.hub.frames.put("/thermal/image", jpg, source_priority=1)
        elif msg.encoding in ("rgb8", "bgr8"):
            arr = np.frombuffer(bytes(msg.data), dtype=np.uint8)
            if arr.size == msg.width * msg.height * 3:
                img = arr.reshape(msg.height, msg.width, 3)
                if msg.encoding == "rgb8":
                    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 65])
                if ok:
                    self.hub.frames.put("/thermal/image", buf.tobytes(), source_priority=1)
                    self.hub.frames.put("/thermal/color", buf.tobytes(), source_priority=1)

    def _log_stats(self) -> None:
        self.get_logger().info(
            f"WS clients={len(self.hub.clients)} | odom={self._odom_count} joints={self._joint_count} | streams={self.hub.frames.topics()}"
        )


def make_ws_app(hub: BridgeHub) -> web.Application:
    app = web.Application()

    async def ws_handler(request: web.Request) -> web.WebSocketResponse:
        ws = web.WebSocketResponse(max_msg_size=16 * 1024 * 1024)
        await ws.prepare(request)
        hub.register(ws)
        try:
            async for msg in ws:
                if msg.type != WSMsgType.TEXT:
                    continue
                try:
                    data = json.loads(msg.data)
                except ValueError:
                    continue
                op = data.get("op")
                topic = data.get("topic")
                if op == "subscribe" and topic:
                    hub.subscribe(ws, topic, data.get("throttle_rate", 0))
                elif op == "unsubscribe" and topic:
                    hub.unsubscribe(ws, topic)
                elif op == "publish" and topic and isinstance(data.get("msg"), dict):
                    hub.handle_unity_publish(ws, topic, data["msg"])
        finally:
            hub.unregister(ws)
        return ws

    app.router.add_get("/", ws_handler)
    return app


def make_video_app(hub: BridgeHub) -> web.Application:
    app = web.Application()

    async def index(_: web.Request) -> web.Response:
        items = "".join(f'<li><a href="/stream?topic={t}">{t}</a></li>' for t in hub.frames.topics())
        return web.Response(text=f"<html><body><h1>Gazebo-Unity MJPEG Streams</h1><ul>{items}</ul></body></html>", content_type="text/html")

    async def snapshot(request: web.Request) -> web.Response:
        topic = request.query.get("topic", "/camera/image")
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            jpg, _ = hub.frames.get(topic)
            if jpg is not None:
                return web.Response(body=jpg, content_type="image/jpeg", headers={"Access-Control-Allow-Origin": "*"})
            await asyncio.sleep(0.05)
        return web.Response(status=404, text=f"No frame on {topic}")

    async def stream(request: web.Request) -> web.StreamResponse:
        topic = request.query.get("topic", "/camera/image")
        resp = web.StreamResponse(
            status=200,
            headers={
                "Content-Type": "multipart/x-mixed-replace; boundary=frame",
                "Cache-Control": "no-cache",
                "Access-Control-Allow-Origin": "*",
            },
        )
        await resp.prepare(request)
        last_ts = 0.0
        try:
            while True:
                jpg, ts = hub.frames.get(topic)
                if jpg is not None and ts != last_ts:
                    last_ts = ts
                    header = (
                        b"--frame\r\n"
                        b"Content-Type: image/jpeg\r\n"
                        + f"Content-Length: {len(jpg)}\r\n\r\n".encode("ascii")
                    )
                    await resp.write(header + jpg + b"\r\n")
                await asyncio.sleep(0.05)
        except (asyncio.CancelledError, ConnectionResetError, BrokenPipeError):
            pass
        return resp

    app.router.add_get("/", index)
    app.router.add_get("/snapshot", snapshot)
    app.router.add_get("/stream", stream)
    return app


async def run_servers(args: argparse.Namespace) -> None:
    loop = asyncio.get_running_loop()
    hub = BridgeHub(loop)

    rclpy.init()
    node = GazeboUnityBridgeNode(hub)
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    # 종료 시 열린 WS(Unity·대시보드)·MJPEG 스트림 핸들러를 1 s만 기다리고 끊는다 — 기본 60 s × 2라 포트가 남았다
    ws_runner = web.AppRunner(make_ws_app(hub), shutdown_timeout=1.0)
    await ws_runner.setup()
    ws_site = web.TCPSite(ws_runner, args.host, args.ws_port)
    await ws_site.start()

    vid_runner = web.AppRunner(make_video_app(hub), shutdown_timeout=1.0)
    await vid_runner.setup()
    vid_site = web.TCPSite(vid_runner, args.host, args.video_port)
    await vid_site.start()

    node.get_logger().info(f"Bridge ready: ws://{args.host}:{args.ws_port} | video http://{args.host}:{args.video_port}")

    stop_event = asyncio.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, stop_event.set)
    await stop_event.wait()

    await ws_runner.cleanup()
    await vid_runner.cleanup()
    node.destroy_node()
    rclpy.shutdown()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--ws-port", type=int, default=9090)
    parser.add_argument("--video-port", type=int, default=8090)
    args = parser.parse_args()
    asyncio.run(run_servers(args))


if __name__ == "__main__":
    main()
