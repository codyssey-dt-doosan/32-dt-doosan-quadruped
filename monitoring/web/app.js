const els = {
  conn: document.getElementById("conn-badge"),
  pose: document.getElementById("pose"),
  rpy: document.getElementById("rpy"),
  twist: document.getElementById("twist"),
  mission: document.getElementById("mission"),
  goal: document.getElementById("goal"),
  gauge: document.getElementById("gauge"),
  thermal: document.getElementById("thermal"),
  gas: document.getElementById("gas"),
  rth: document.getElementById("rth"),
  gasBar: document.getElementById("gas-bar"),
};

function setGas(value) {
  const v = Number(value) || 0;
  els.gas.textContent = v.toFixed(2);
  els.gasBar.style.width = `${Math.min(100, v)}%`;
  els.gasBar.style.background = v > 50 ? "#c4453c" : v > 20 ? "#d4a017" : "#3dba84";
}

function connectRosbridge() {
  const url = "ws://localhost:9090";
  let ws;
  try {
    ws = new WebSocket(url);
  } catch (err) {
    return;
  }
  ws.onopen = () => {
    els.conn.textContent = "rosbridge 연결";
    els.conn.classList.add("on");
    els.conn.classList.remove("off");
    const advertise = (topic, type) =>
      ws.send(JSON.stringify({ op: "subscribe", topic, type }));
    advertise("/odom", "nav_msgs/msg/Odometry");
    advertise("/mission/status", "std_msgs/msg/String");
    advertise("/patrol/goal", "geometry_msgs/msg/PoseStamped");
    advertise("/inspection/gauge", "std_msgs/msg/Float32");
    advertise("/inspection/thermal_alert", "std_msgs/msg/Bool");
    advertise("/gas/concentration", "std_msgs/msg/Float32");
  };
  ws.onmessage = (ev) => {
    const msg = JSON.parse(ev.data);
    if (msg.op !== "publish") return;
    const m = msg.msg;
    if (msg.topic === "/odom") {
      const p = m.pose.pose.position;
      els.pose.textContent = `${p.x.toFixed(2)}, ${p.y.toFixed(2)}, ${p.z.toFixed(2)}`;
      const t = m.twist.twist.linear;
      els.twist.textContent = `${t.x.toFixed(2)} m/s`;
    }
    if (msg.topic === "/mission/status") els.mission.textContent = m.data;
    if (msg.topic === "/patrol/goal") {
      const p = m.pose.position;
      els.goal.textContent = `${p.x.toFixed(1)}, ${p.y.toFixed(1)}`;
    }
    if (msg.topic === "/inspection/gauge") els.gauge.textContent = m.data.toFixed(1);
    if (msg.topic === "/inspection/thermal_alert") {
      els.thermal.textContent = m.data ? "과열" : "정상";
    }
    if (msg.topic === "/gas/concentration") setGas(m.data);
  };
  ws.onclose = () => {
    els.conn.textContent = "rosbridge 대기 (목업)";
    els.conn.classList.remove("on");
    els.conn.classList.add("off");
    setTimeout(connectRosbridge, 3000);
  };
}

connectRosbridge();
setGas(0);
