// roslib.js 로 rosbridge(ROS 2)에 붙어 카드 + 3D 트윈을 갱신한다.
// rosbridge 미연결 시에는 웨이포인트를 도는 목업 데이터로 화면을 채운다.
import { Twin3D, WORLDS } from "./twin3d.js?v=4";

const $ = (id) => document.getElementById(id);
const els = {
  conn: $("conn-badge"),
  pose: $("pose"),
  rpy: $("rpy"),
  twist: $("twist"),
  mission: $("mission"),
  goal: $("goal"),
  gauge: $("gauge"),
  thermal: $("thermal"),
  gas: $("gas"),
  rth: $("rth"),
  gasBar: $("gas-bar"),
  world: $("world-select"),
  wsUrl: $("ws-url"),
  follow: $("btn-follow"),
  reset: $("btn-reset"),
  clear: $("btn-clear"),
};

const params = new URLSearchParams(location.search);
const twin = new Twin3D($("twin3d"));
let world = params.get("world") in WORLDS ? params.get("world") : els.world.value;
els.world.value = world;
twin.setWorld(world);

els.world.addEventListener("change", () => {
  world = els.world.value;
  twin.setWorld(world);
  syncThermalTopic();
  demo.reset();
});
els.follow.addEventListener("click", () => {
  twin.follow = !twin.follow;
  els.follow.classList.toggle("active", twin.follow);
});
els.reset.addEventListener("click", () => twin.resetView());
els.clear.addEventListener("click", () => twin.clearTrail());

// ---------- 공통 UI 갱신 ----------
function quatToRpy(q) {
  const roll = Math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y));
  const pitch = Math.asin(Math.max(-1, Math.min(1, 2 * (q.w * q.y - q.z * q.x))));
  const yaw = Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
  return { roll, pitch, yaw };
}
const deg = (r) => ((r * 180) / Math.PI).toFixed(1);

function onOdom(p, q, vx) {
  const { roll, pitch, yaw } = quatToRpy(q);
  els.pose.textContent = `${p.x.toFixed(2)}, ${p.y.toFixed(2)}, ${p.z.toFixed(2)}`;
  els.rpy.textContent = `${deg(roll)}°, ${deg(pitch)}°, ${deg(yaw)}°`;
  els.twist.textContent = `${vx.toFixed(2)} m/s`;
  twin.setPose(p.x, p.y, p.z, yaw);
  twin.setSpeed(vx);
}

function onMission(status) {
  els.mission.textContent = status;
  twin.setGaugeReading(/gauge|inspect|dwell|read/i.test(status));
  if (/return|home|rth/i.test(status)) els.rth.textContent = "복귀 중";
}

function onGoal(kind, p) {
  twin.setGoal(kind, p.x, p.y);
  if (kind === "patrol") els.goal.textContent = `${p.x.toFixed(1)}, ${p.y.toFixed(1)}`;
  if (kind === "rth") els.rth.textContent = `복귀 → ${p.x.toFixed(1)}, ${p.y.toFixed(1)}`;
}

function onThermal(on) {
  els.thermal.textContent = on ? "과열" : "정상";
  twin.setThermalAlert(on);
}

function setGas(value) {
  const v = Number(value) || 0;
  els.gas.textContent = v.toFixed(2);
  els.gasBar.style.width = `${Math.min(100, v)}%`;
  els.gasBar.style.background = v > 50 ? "#c4453c" : v > 20 ? "#d4a017" : "#3dba84";
  twin.setGas(v);
}

function setBadge(text, on) {
  els.conn.textContent = text;
  els.conn.classList.toggle("on", on);
  els.conn.classList.toggle("off", !on);
}

// ---------- roslib.js ----------
const defaultUrl = params.get("ws") || `ws://${location.hostname || "localhost"}:9090`;
els.wsUrl.value = defaultUrl;
let ros = null;
let topics = [];
let retryTimer = null;

function subscribe(name, messageType, cb, throttle_rate = 0) {
  const t = new ROSLIB.Topic({ ros, name, messageType, throttle_rate, queue_length: 1 });
  t.subscribe(cb);
  topics.push(t);
}

function connect(url) {
  clearTimeout(retryTimer);
  if (ros) {
    ros.removeAllListeners?.();
    ros.close();
  }
  setBadge("rosbridge 연결 중…", false);
  ros = new ROSLIB.Ros({ url });

  ros.on("connection", () => {
    demo.stop();
    twin.clearTrail();
    setBadge("rosbridge 연결", true);
    els.conn.title = url;
    topics = [];
    subscribe("/odom", "nav_msgs/msg/Odometry", (m) =>
      onOdom(m.pose.pose.position, m.pose.pose.orientation, m.twist.twist.linear.x), 50);
    subscribe("/mission/status", "std_msgs/msg/String", (m) => onMission(m.data));
    subscribe("/patrol/goal", "geometry_msgs/msg/PoseStamped", (m) => onGoal("patrol", m.pose.position));
    subscribe("/return_to_home/goal", "geometry_msgs/msg/PoseStamped", (m) => onGoal("rth", m.pose.position));
    subscribe("/source_seeking/goal", "geometry_msgs/msg/PoseStamped", (m) => onGoal("seek", m.pose.position));
    subscribe("/inspection/gauge", "std_msgs/msg/Float32", (m) => (els.gauge.textContent = m.data.toFixed(1)));
    subscribe("/inspection/thermal_alert", "std_msgs/msg/Bool", (m) => onThermal(m.data));
    subscribe("/gas/concentration", "std_msgs/msg/Float32", (m) => setGas(m.data), 100);
    subscribe("/fall_recovery/status", "std_msgs/msg/String", (m) => {
      // fall_recovery 상태: idle | fallen | recovering | recovered
      if (m.data === "fallen" || m.data === "recovering") els.mission.textContent = `낙상: ${m.data}`;
    });
  });

  const onDown = () => {
    topics.forEach((t) => t.unsubscribe());
    topics = [];
    setBadge("rosbridge 대기 (목업)", false);
    demo.start();
    retryTimer = setTimeout(() => connect(els.wsUrl.value), 3000);
  };
  ros.on("close", onDown);
  ros.on("error", () => {}); // close 가 뒤따른다
}

els.wsUrl.addEventListener("change", () => connect(els.wsUrl.value.trim()));

// ---------- 로봇 카메라: web_video_server MJPEG 스트림 ----------
// 영상은 rosbridge(JSON) 대신 web_video_server 가 JPEG 로 압축해 HTTP 로 보낸다.
const videoUrl = $("video-url");
videoUrl.value = params.get("video") || `http://${location.hostname || "localhost"}:8090`;
const cams = [...document.querySelectorAll(".cam")].map((fig) => ({
  fig,
  img: fig.querySelector("img"),
  state: fig.querySelector(".cam-state"),
  topic: fig.dataset.topic,
  retry: null,
}));

function startCam(cam) {
  clearTimeout(cam.retry);
  const base = videoUrl.value.trim().replace(/\/$/, "");
  // web_video_server 는 topic 을 URL 디코딩하지 않으므로 인코딩하지 않는다 (%2F 면 "Invalid topic name")
  const url = `${base}/stream?topic=${cam.topic}&type=mjpeg&quality=60`;
  cam.fig.classList.remove("live");
  cam.fig.classList.add("off");
  cam.state.textContent = "연결 중…";
  const fail = () => {
    clearTimeout(cam.retry);
    cam.fig.classList.remove("live");
    cam.fig.classList.add("off");
    cam.state.textContent = "신호 없음";
    cam.img.onload = cam.img.onerror = null;
    cam.img.removeAttribute("src"); // 열린 스트림 연결을 닫는다
    cam.retry = setTimeout(() => startCam(cam), 3000);
  };
  cam.img.onload = () => {
    clearTimeout(cam.retry);
    cam.fig.classList.add("live");
    cam.fig.classList.remove("off");
    cam.state.textContent = cam.topic;
  };
  cam.img.onerror = fail;
  // 오류여도 서버가 연결을 끊지 않는 경우가 있어 첫 프레임이 8초 안에 없으면 다시 붙는다
  cam.retry = setTimeout(fail, 8000);
  // 같은 URL 재요청 시 캐시를 피하려고 t 를 붙인다
  cam.img.src = `${url}&t=${Date.now()}`;
}

function syncThermalTopic() {
  const thermal = cams[1];
  if (!thermal || !thermal.fig.dataset.unityTopic) return;
  const next = world === "unity" ? thermal.fig.dataset.unityTopic : thermal.fig.dataset.topic;
  if (thermal.topic !== next) {
    thermal.topic = next;
    startCam(thermal);
  }
}

if (cams[1]?.fig.dataset.unityTopic && world === "unity") {
  cams[1].topic = cams[1].fig.dataset.unityTopic;
}
cams.forEach(startCam);
videoUrl.addEventListener("change", () => cams.forEach(startCam));

const overlayBtn = $("btn-cam-overlay");
overlayBtn.addEventListener("click", () => {
  const front = cams[0];
  const on = !overlayBtn.classList.contains("active");
  overlayBtn.classList.toggle("active", on);
  front.topic = on ? front.fig.dataset.altTopic : front.fig.dataset.topic;
  startCam(front);
});

// ---------- 목업: rosbridge 없을 때 웨이포인트 순찰 재생 ----------
const demo = {
  timer: null,
  reset() {
    const w = WORLDS[world];
    this.wps = [...w.waypoints, w.waypoints[0]];
    this.i = 1;
    this.x = this.wps[0].x;
    this.y = this.wps[0].y;
    this.yaw = 0;
    this.dwell = 0;
    if (this.timer) twin.clearTrail();
  },
  start() {
    if (this.timer) return;
    this.reset();
    let last = performance.now();
    this.timer = setInterval(() => {
      const now = performance.now();
      this.step((now - last) / 1000);
      last = now;
    }, 50);
  },
  stop() {
    clearInterval(this.timer);
    this.timer = null;
  },
  step(dt) {
    const w = WORLDS[world];
    const target = this.wps[this.i];
    let v = 0;
    if (this.dwell > 0) {
      this.dwell -= dt;
      onMission(`inspect:${target.name}`);
      if (this.dwell <= 0) this.i = (this.i + 1) % this.wps.length || 1;
    } else {
      const dx = target.x - this.x;
      const dy = target.y - this.y;
      const d = Math.hypot(dx, dy);
      const want = Math.atan2(dy, dx);
      let e = Math.atan2(Math.sin(want - this.yaw), Math.cos(want - this.yaw));
      this.yaw += Math.max(-2 * dt, Math.min(2 * dt, e));
      v = Math.abs(e) < 0.4 ? Math.min(1.2, d) : 0.1;
      this.x += v * dt * Math.cos(this.yaw);
      this.y += v * dt * Math.sin(this.yaw);
      onMission(`patrol → ${target.name}`);
      onGoal("patrol", target);
      if (d < 0.15) {
        this.dwell = target.name === "home" ? 0.5 : 2.5;
        if (target.name === "home") twin.clearTrail();
      }
    }
    onOdom({ x: this.x, y: this.y, z: 0.4 }, { x: 0, y: 0, z: Math.sin(this.yaw / 2), w: Math.cos(this.yaw / 2) }, v);
    const p = w.plume || { x: 8, y: -6, sigma: 3, peak: 100 };
    setGas(p.peak * Math.exp(-((this.x - p.x) ** 2 + (this.y - p.y) ** 2) / (2 * p.sigma ** 2)));
    els.gauge.textContent = this.dwell > 0 ? (42 + 3 * Math.sin(now() / 700)).toFixed(1) : els.gauge.textContent;
  },
};
const now = () => performance.now();

setGas(0);
if (typeof ROSLIB === "undefined") {
  setBadge("roslib.js 로드 실패 (목업)", false);
  demo.start();
} else {
  connect(defaultUrl);
}
