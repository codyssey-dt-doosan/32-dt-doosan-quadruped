"""주행 로그(CSV + JSON)를 플룸 농도 지도 위에 그린다.

  ros2 run source_seeking plot_run /tmp/gas_run --world factory      # -> /tmp/gas_run.png
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.colors import LogNorm  # noqa: E402
from matplotlib.patches import Circle, Rectangle  # noqa: E402

from plume_sim.plume import GasField, GasFieldParams, load_params  # noqa: E402
from source_seeking.offline_sim import WorldParams, boxes, config_path  # noqa: E402

STATE_COLORS = {"SEARCH": "#8a8a8a", "TRACK": "#1f6fd1", "ESCAPE": "#e8850c", "SOURCE_FOUND": "#1a9e3a",
                "RETURN_HOME": "#8e3fc4", "DONE": "#222222", "WAIT": "#cccccc"}


def read_csv(path):
    with open(path, encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    num = lambda k: np.array([float(r[k]) if r[k] != "" else np.nan for r in rows])  # noqa: E731
    d = {k: num(k) for k in ("t", "x", "y", "conc", "conc_f", "battery", "return_threshold", "d_home")}
    d["state"] = [r["state"] for r in rows]
    return d


def plot(prefix, world="factory", out=None):
    d = read_csv(prefix + ".csv")
    meta = {}
    if os.path.exists(prefix + ".json"):
        with open(prefix + ".json", encoding="utf-8") as f:
            meta = json.load(f)
    world = meta.get("world", world)
    gp = load_params(config_path("plume_sim", f"plume_{world}.yaml"), "plume_sim", GasFieldParams)
    field = GasField(gp)
    obst = boxes(load_params(config_path("source_seeking", f"world_{world}.yaml"), "offline_sim", WorldParams).obstacles)

    xs = [b[0] for b in obst] + [b[2] for b in obst] + list(d["x"])
    ys = [b[1] for b in obst] + [b[3] for b in obst] + list(d["y"])
    x0, x1, y0, y1 = np.nanmin(xs) - 0.5, np.nanmax(xs) + 0.5, np.nanmin(ys) - 0.5, np.nanmax(ys) + 0.5
    aspect = (y1 - y0) / (x1 - x0)
    fig = plt.figure(figsize=(12, 3.2 + 8.5 * aspect + 2.4))
    top = 2.4 / fig.get_figheight()
    ax = fig.add_axes([0.06, top + 0.06, 0.88, 1 - top - 0.11])
    gx, gy = np.meshgrid(np.linspace(x0, x1, 300), np.linspace(y0, y1, max(40, int(300 * aspect))))
    gc = np.vectorize(field.mean)(gx, gy)
    im = ax.pcolormesh(gx, gy, gc, cmap="YlOrRd", norm=LogNorm(vmin=max(gp.background, 0.3), vmax=gc.max()),
                       shading="auto", alpha=0.85)
    fig.colorbar(im, ax=ax, fraction=0.025, pad=0.01, label="mean concentration (ppm)")
    for (a, b, c, e) in obst:
        ax.add_patch(Rectangle((a, b), c - a, e - b, color="#3b3b3b", alpha=0.85, lw=0))
    L = 0.08 * (x1 - x0)
    ax.annotate("", xy=(x0 + 1 + L * math.cos(gp.wind_direction) + L, y1 - 0.4 + L * 0.3 * math.sin(gp.wind_direction)),
                xytext=(x0 + 1 + L, y1 - 0.4), arrowprops=dict(arrowstyle="->", lw=2, color="#0a58a5"))
    ax.text(x0 + 0.6, y1 - 0.3, "wind %.1f m/s" % gp.wind_speed, color="#0a58a5", fontsize=9, va="top")

    st = d["state"]
    for i in range(1, len(st)):
        ax.plot(d["x"][i - 1:i + 1], d["y"][i - 1:i + 1], color=STATE_COLORS.get(st[i], "k"), lw=1.8)
    for q in meta.get("tabu", []):
        ax.add_patch(Circle(q, 0.35, fill=False, ec="#d62728", lw=2))
        ax.text(q[0] + 0.4, q[1] + 0.4, "tabu", color="#d62728", fontsize=9)
    ax.plot(gp.source_x, gp.source_y, marker="*", ms=20, color="#ffd400", mec="k", ls="", label="true leak")
    if meta.get("source_estimate"):
        s = meta["source_estimate"]
        err = math.hypot(s[0] - gp.source_x, s[1] - gp.source_y)
        ax.plot(*s, marker="X", ms=12, color="#1a9e3a", mec="k", ls="", label="found (error %.2f m)" % err)
    if meta.get("home"):
        ax.plot(*meta["home"], marker="s", ms=10, color="w", mec="k", ls="", label="home")
    for k, c in STATE_COLORS.items():
        if k in st and k not in ("WAIT", "DONE"):
            ax.plot([], [], color=c, lw=3, label=k)
    ax.set_xlim(x0, x1)
    ax.set_ylim(y0, y1)
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.legend(loc="lower left", fontsize=8, ncol=4, framealpha=0.9)
    final = meta.get("final_state", st[-1])
    ax.set_title("module3 gas safety: world=%s, mode=%s, final=%s" % (world, meta.get("mode", "?"), final))

    ax2 = fig.add_axes([0.06, 0.05, 0.88, top - 0.02])
    t = d["t"] - d["t"][0]
    i = 0
    while i < len(st):
        j = i
        while j < len(st) and st[j] == st[i]:
            j += 1
        ax2.axvspan(t[i], t[min(j, len(t) - 1)], color=STATE_COLORS.get(st[i], "w"), alpha=0.15, lw=0)
        i = j
    ax2.plot(t, d["conc"], color="#e0a0a0", lw=0.6, label="raw conc")
    ax2.plot(t, d["conc_f"], color="#c0392b", lw=1.5, label="filtered conc")
    ax2.set_ylabel("ppm")
    ax2.set_xlabel("time (s)")
    ax3 = ax2.twinx()
    ax3.plot(t, d["battery"], color="#1a9e3a", lw=1.5, label="battery %")
    ax3.plot(t, d["return_threshold"], color="#1a9e3a", lw=1.2, ls="--", label="return threshold %")
    ax3.set_ylabel("battery %")
    ax3.set_ylim(0, 105)
    h1, l1 = ax2.get_legend_handles_labels()
    h2, l2 = ax3.get_legend_handles_labels()
    ax2.legend(h1 + h2, l1 + l2, loc="upper left", fontsize=8, ncol=4)
    ax2.set_xlim(t[0], t[-1])

    out = out or prefix + ".png"
    fig.savefig(out, dpi=110)
    plt.close(fig)
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("prefix", help="로그 접두사 (.csv 제외)")
    ap.add_argument("--world", default="factory")
    ap.add_argument("--out")
    a = ap.parse_args()
    print(plot(a.prefix, a.world, a.out))


if __name__ == "__main__":
    main()
