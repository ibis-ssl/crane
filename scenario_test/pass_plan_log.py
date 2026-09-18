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
        script = os.path.join(scenario_dir, "pass_plan_recorder.py")
        command = (
            "source /opt/ros/jazzy/setup.bash && "
            f"source {workspace_root}/install/local_setup.bash && "
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
