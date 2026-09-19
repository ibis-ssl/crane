"""sidecar (`pass_plan_recorder.py`) が書く JSONL を pytest 側から読む。

pytest は venv で動き rclpy を持たないので、PassPlan の観測は別プロセスに任せ、
ここでは素のファイル I/O だけを行う。理由の詳細は pass_plan_recorder.py の docstring。

キック時点の計画を突き合わせるのが主目的なので、時刻で引く API を中心に置く。
recorder と pytest は同じホストの同じ壁時計（time.time()）を使う。
"""

import json
import os
import signal
import subprocess
import tempfile
import time

# recorder が起動して最初の計画を観測するまでの待ち上限 [s]。
# ここで落ちるのは「craneが起動していない」等の環境不良であり、
# シナリオのassert失敗とは区別して報告したい（scenario_test/README.md の方針）。
STARTUP_TIMEOUT = 30.0


class PassPlanLog:
    """recorder の JSONL を追いかけ、時刻で計画・割当を引く。"""

    def __init__(self, path: str):
        self._path = path
        # sidecar が書き足す間ずっと追いかけるので、with では閉じられない。
        self._fp = open(path, "r", encoding="utf-8")  # noqa: SIM115
        self._records: list[dict] = []

    # ─── 読み取り ────────────────────────────────────────────────────────────

    def poll(self) -> list[dict]:
        """新しく書かれた行を取り込み、全レコードを返す。

        `for line in fp` ではなく readline() を使う。ファイルオブジェクトの
        イテレーションは内部で先読みバッファを持ち、tell() が使えなくなるため、
        書きかけの行を読み戻せない。
        """
        while True:
            position = self._fp.tell()
            line = self._fp.readline()
            if not line.endswith("\n"):
                # 未完の行（書き込み途中）。次回の poll で読み直す。
                self._fp.seek(position)
                break
            line = line.strip()
            if line:
                self._records.append(json.loads(line))
        return self._records

    def records(self, kind: str | None = None) -> list[dict]:
        self.poll()
        if kind is None:
            return list(self._records)
        return [r for r in self._records if r.get("kind") == kind]

    # ─── 時刻で引く ──────────────────────────────────────────────────────────

    def latest_before(self, kind: str, when: float) -> dict | None:
        """`when` 以前で最後に観測された `kind` のレコード。

        recorder は値が変化したときだけ書くので、「直前の行」が
        その時刻に有効だった状態を表す。
        """
        best = None
        for record in self.records(kind):
            if record["t"] <= when:
                best = record
            else:
                break
        return best

    def plan_at(self, when: float) -> dict | None:
        return self.latest_before("plan", when)

    def plan_acted_on(self, kick_time: float, lookback: float = 1.0) -> dict | None:
        """キックの根拠になった計画を返す。無ければ None。

        `plan_at(kick_time)` では遅すぎる。キックでボールが動き出すと
        PassPlanMetric は計画を解除する（ボール移動中は writeInactivePlan）。
        一方 pytest 側のキック検出はボール速度が KICK_DETECT_SPEED を超えてからで、
        vision の速度推定窓のぶんさらに遅れる。その結果、実際には計画どおりに
        蹴っていても、検出時点では INACTIVE しか観測できない。

        そこで検出時刻から `lookback` 秒だけさかのぼり、その区間で最後に
        有効だった計画（PLANNING / BALL_IN_FLIGHT）を採用する。
        """
        usable = [
            r
            for r in self.records("plan")
            if kick_time - lookback <= r["t"] <= kick_time
            and r["state_name"] in ("PLANNING", "BALL_IN_FLIGHT")
        ]
        return usable[-1] if usable else None

    def ball_peak_speed(self, start: float, end: float) -> float:
        """区間内で観測されたボール速度の最大値 [m/s]。無ければ NaN。

        world_model の ball_info.velocity を使う。tracker がボールを見えている
        間はこれが tracker の推定値そのもの（world_model_data_provider の
        updateBallInfo）。pytest 側の vision 位置差分は 0.08 秒窓の差分なので
        外れ値が出る。実際、差分推定が 6〜8 m/s を示した試行を「シュート」と
        誤分類し、存在しない問題を追いかけた。キックの分類はこちらで行うこと。
        """
        speeds = [r["speed"] for r in self.records("ball") if start <= r["t"] <= end]
        return max(speeds) if speeds else float("nan")

    def tracker_kick_near(self, when: float, window: float = 1.5) -> dict | None:
        """自前のキック検出時刻に対応する Tracker のキック。無ければ None。

        Tracker は蹴ったロボットの ID・初速・キック時刻をそのまま持っているので、
        pytest 側のしきい値による自前検出より素性が良い。ただし tracked_frame の
        生成側が kicked_ball を入れるとは限らない（proto のコメントが optional と
        明記しており、実測でも 3 試行中 2 件で来なかった）ので、無いことを前提に
        使うこと。

        探すのは「検出時刻**以前**で最も近いキック」。自前検出はしきい値を越える
        まで待つぶん必ず遅れるため、前後で最も近いものを採ると受け手が触った瞬間
        の方を拾う。実測でそれが起き、出し手を取り違えた。
        """
        kicks = [
            r
            for r in self.records("tracker_kick")
            if when - window <= self.kick_time(r) <= when
        ]
        return max(kicks, key=self.kick_time) if kicks else None

    @staticmethod
    def kick_time(kick: dict) -> float:
        """Tracker のキック時刻 [unix 秒]。

        start_timestamp は Tracker がキックが起きたと判断した時刻そのもので、
        受信時刻（t）より正確なので、あればそちらを使う。
        """
        stamp = float(kick.get("start_timestamp") or 0.0)
        return stamp if stamp > 0.0 else float(kick["t"])

    def assign_at(self, when: float) -> dict | None:
        return self.latest_before("assign", when)

    # ─── 診断 ────────────────────────────────────────────────────────────────

    def failing_gates(self, plan: dict) -> list[str]:
        """その計画で false になっているゲート名。空なら msg 由来のゲートは全通過。"""
        return sorted(name for name, ok in plan.get("gates", {}).items() if not ok)

    def describe(self, when: float) -> str:
        """`when` 時点の計画と割当を1行で。失敗時のログ用。"""
        plan = self.plan_at(when)
        assign = self.assign_at(when)
        if plan is None:
            return "PassPlan: (未観測)"
        gates = self.failing_gates(plan)
        point = plan["receive_point"]
        return (
            f"PassPlan: {plan['state_name']} id={plan['plan_id']} "
            f"kicker={plan['kicker_id']} receiver={plan['receiver_id']} "
            f"point=({point['x']:.2f},{point['y']:.2f}) "
            f"speed={plan['kick_speed']:.2f} score={plan['score']:.2f} "
            f"NGゲート={gates or 'なし'} "
            f"割当={assign['watched'] if assign else '(未観測)'}"
        )


# Tracker のマルチキャストポート。crane_bringup/launch/crane.launch.xml の
# tracker_port と同じ値にすること（SSL 既定の 10010 ではない）。
TRACKER_PORT = 11010


class _RecorderProcess:
    """sidecar の起動と停止。"""

    def __init__(self):
        scenario_dir = os.path.dirname(os.path.abspath(__file__))
        repo_root = os.path.dirname(scenario_dir)
        workspace_root = os.path.dirname(os.path.dirname(repo_root))

        handle, self.path = tempfile.mkstemp(prefix="pass_plan_", suffix=".jsonl")
        os.close(handle)

        # venv には rclpy が無いので、システム python を ROS 環境で起動する。
        # インタプリタは絶対パスで指定すること。pytest は venv の中で動いており
        # PATH の先頭が venv なので、`python3` と書くと venv 側が選ばれる。
        # その python には rclpy も numpy も無く、rclpy.node の import が
        # ModuleNotFoundError: numpy で落ちる（rosgraph_msgs 経由）。
        # setsid 相当（start_new_session）でプロセスグループを分け、確実に止められるようにする。
        # tracker_node も一緒に上げる。crane 本体は tracked_frame を内部で parse
        # するだけで ROS トピックには出さないため、Tracker のキック検出
        # （蹴ったロボット・初速・キック時刻）を取るには受信ノードが別途要る。
        # ポートは crane.launch.xml の tracker_port と揃える（既定 10010 ではない）。
        # start_new_session でプロセスグループを分けているので、stop() の killpg で
        # 両方まとめて止まる。
        script = os.path.join(scenario_dir, "pass_plan_recorder.py")
        command = (
            "source /opt/ros/jazzy/setup.bash && "
            f"source {workspace_root}/install/local_setup.bash && "
            "ros2 run robocup_ssl_comm tracker_node --ros-args "
            f"-p multicast_port:={TRACKER_PORT} >/dev/null 2>&1 & "
            f"exec /usr/bin/python3 {script} --out {self.path}"
        )
        # 子プロセスが動いている間の出力先。stop() まで開いたままにする。
        self._log = open(self.path + ".stderr", "w", encoding="utf-8")  # noqa: SIM115
        self._proc = subprocess.Popen(
            ["bash", "-c", command],
            stdout=self._log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )

    def wait_until_observing(self, log: PassPlanLog) -> None:
        """最初の計画レコードが出るまで待つ。出なければ環境不良として落とす。"""
        deadline = time.time() + STARTUP_TIMEOUT
        while time.time() < deadline:
            if log.records("plan"):
                return
            if self._proc.poll() is not None:
                raise RuntimeError(
                    f"pass_plan_recorder が終了しました (rc={self._proc.returncode})。"
                    f"ログ: {self.path}.stderr"
                )
            time.sleep(0.05)
        raise RuntimeError(
            "pass_plan_recorder が /world_model を観測できませんでした。"
            f"crane が起動しているか確認してください。ログ: {self.path}.stderr"
        )

    def stop(self) -> None:
        if self._proc.poll() is None:
            os.killpg(os.getpgid(self._proc.pid), signal.SIGINT)
            try:
                self._proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(self._proc.pid), signal.SIGKILL)
                self._proc.wait(timeout=5)
        self._log.close()


def start_recorder() -> tuple[_RecorderProcess, PassPlanLog]:
    proc = _RecorderProcess()
    log = PassPlanLog(proc.path)
    proc.wait_until_observing(log)
    return proc, log
