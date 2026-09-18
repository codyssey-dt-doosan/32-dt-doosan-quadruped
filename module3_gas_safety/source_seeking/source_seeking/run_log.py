"""주행 로그 CSV + 요약 JSON (오프라인 시뮬레이터와 노드가 같이 쓰고 plot_run이 읽는다)."""

from __future__ import annotations

import csv
import json
import os

COLUMNS = ["t", "x", "y", "yaw", "conc", "conc_f", "battery", "return_threshold", "d_home", "state"]


class RunLog:
    def __init__(self, prefix: str) -> None:
        self.prefix = prefix
        os.makedirs(os.path.dirname(os.path.abspath(prefix)), exist_ok=True)
        self._f = open(prefix + ".csv", "w", newline="", encoding="utf-8")
        self._w = csv.writer(self._f)
        self._w.writerow(COLUMNS)

    @staticmethod
    def state_of(seeker, ret) -> str:
        return "RETURN_HOME" if ret is not None and ret.state == "RETURNING" else \
            "DONE" if ret is not None and ret.state == "HOME" else seeker.state

    def row(self, t, x, y, yaw, conc, seeker, ret, battery) -> None:
        f = lambda v: "" if v is None else "%.4f" % v  # noqa: E731
        self._w.writerow([f(t), f(x), f(y), f(yaw), f(conc), f(seeker.c_f), f(battery),
                          f(ret.threshold if ret else None), f(ret.home_path_length() if ret else None),
                          self.state_of(seeker, ret)])

    @staticmethod
    def events(seeker, ret) -> list:
        ev = [e for e in seeker.events if e[2] != "blocked by obstacle"]
        if ret is not None and ret.t_start is not None:
            ev.append((ret.t_start, "RETURN_HOME", ret.return_reason))
        if ret is not None and ret.t_home is not None:
            ev.append((ret.t_home, "DONE", ret.reason))
        return sorted(ev, key=lambda e: e[0])

    def write_summary(self, seeker, ret, extra=None) -> None:
        doc = {"home": ret.home if ret else seeker.start, "source_estimate": seeker.source, "tabu": seeker.tabu,
               "final_state": self.state_of(seeker, ret),
               "return_reason": ret.return_reason if ret else None,
               "blocked_count": sum(1 for e in seeker.events if e[2] == "blocked by obstacle"),
               "events": [list(e) for e in self.events(seeker, ret)]}
        doc.update(extra or {})
        with open(self.prefix + ".json", "w", encoding="utf-8") as f:
            json.dump(doc, f, indent=1, ensure_ascii=False)

    def close(self, seeker, ret, extra=None) -> None:
        self._f.close()
        self.write_summary(seeker, ret, extra)
