// ワールドモデル・フィードバック・注目ロボットの単一の真実の源。
//
// 描画 (CanvasRenderer / RobotHud) とサイドバーの各タブは、どちらもここを読む。
// 「レンダラ用の選択」と「パネル用の選択」を別々に持つと必ず食い違うので、
// focusedRobotId は 1 つだけにして、変更は setFocus() 経由に限る。
//
// EventTarget を継承しているので、タブは 'focus' / 'world' / 'feedback' を
// 購読すれば済む。ポーリングを書かないこと。

import { MetricRing } from '../ui/Sparkline.js';
import { indexBy } from '../replay/RingBuffer.js';
import { ROBOT_HIT_RADIUS_M, BALL_HIT_RADIUS_M } from '../renderer/constants.js';

const METRIC_SAMPLES = 180;

export class ViewerState extends EventTarget {
    constructor() {
        super();
        this.robotsOurs = {};
        this.robotsTheirs = {};
        this.controlTargets = {};
        this.ballPos = { x: 0, y: 0 };
        this.isYellow = false;
        this.onPositiveHalf = false;

        this.robotFeedback = {};
        this.feedbackTimestamp = {};   // { robot_id: Date.now() } 警告バッジの stale 判定用
        this.latencyEstimation = {};   // { robot_id: { source: { latency_ms, correlation, ... } } }
        this.metrics = new Map();      // id -> { posX, posY, vel }（概要タブのスパークライン用）

        this.focusedRobotId = null;
        this.multiSelect = new Set();
        this.hoveredRobotId = null;
    }

    // ===== フォーカス =====

    setFocus(id) {
        if (this.focusedRobotId === id) return false;
        this.focusedRobotId = id;
        this.dispatchEvent(new CustomEvent('focus', { detail: { id } }));
        return true;
    }

    get focusedRobot() {
        return this.focusedRobotId === null ? null : (this.robotsOurs[this.focusedRobotId] ?? null);
    }

    get focusedCommand() {
        return this.focusedRobotId === null ? null : (this.controlTargets[this.focusedRobotId] ?? null);
    }

    // ===== 受信 =====

    ingestWorldModel(data) {
        if (data.is_yellow !== undefined) this.isYellow = data.is_yellow;
        if (data.on_positive_half !== undefined) this.onPositiveHalf = data.on_positive_half;
        if (data.ball) this.ballPos = { x: data.ball.x, y: data.ball.y };
        if (data.robots_ours) this.robotsOurs = indexBy(data.robots_ours, 'id');
        if (data.robots_theirs) this.robotsTheirs = indexBy(data.robots_theirs, 'id');

        if (data.robots_ours) {
            for (const robot of data.robots_ours) {
                if (!this.metrics.has(robot.id)) {
                    this.metrics.set(robot.id, {
                        posX: new MetricRing(METRIC_SAMPLES),
                        posY: new MetricRing(METRIC_SAMPLES),
                        vel: new MetricRing(METRIC_SAMPLES),
                    });
                }
                const m = this.metrics.get(robot.id);
                m.posX.push(robot.x ?? 0);
                m.posY.push(robot.y ?? 0);
                m.vel.push(Math.hypot(robot.vx ?? 0, robot.vy ?? 0));
            }
        }
        this.dispatchEvent(new Event('world'));
    }

    ingestControlTargets(commands) {
        this.controlTargets = indexBy(commands, 'robot_id');
    }

    // websocket_server.cpp は "robots" キーで送る（"feedback" ではない）
    ingestFeedback(robots) {
        const now = Date.now();
        for (const fb of robots) {
            this.robotFeedback[fb.robot_id] = fb;
            this.feedbackTimestamp[fb.robot_id] = now;
        }
        this.dispatchEvent(new Event('feedback'));
    }

    ingestLatency(estimations) {
        for (const est of estimations) {
            if (!this.latencyEstimation[est.robot_id]) this.latencyEstimation[est.robot_id] = {};
            this.latencyEstimation[est.robot_id][est.source] = {
                latency_ms: est.latency_ms,
                correlation: est.correlation,
                samples_used: est.samples_used,
            };
        }
    }

    // ===== 当たり判定 =====

    nearestRobot(robotMap, fieldX, fieldY, radius = ROBOT_HIT_RADIUS_M) {
        let id = null;
        let dist = radius;
        for (const [k, r] of Object.entries(robotMap)) {
            // availability フラグがない（相手チーム等）場合はフィルタしない
            const hasFlags = r.available_vision !== undefined || r.available_tracker !== undefined;
            if (hasFlags && !r.available_vision && !r.available_tracker) continue;
            const d = Math.hypot(r.x - fieldX, r.y - fieldY);
            if (d < dist) { dist = d; id = Number(k); }
        }
        return id !== null ? { id, dist } : null;
    }

    findRobotAtPosition(fieldX, fieldY) {
        return this.nearestRobot(this.robotsOurs, fieldX, fieldY)?.id ?? null;
    }

    // sim 編集の選択候補。同距離ならボール > 自チーム > 相手チームの順で拾う
    pickSimObject(fieldX, fieldY) {
        const candidates = [];
        const ourHit = this.nearestRobot(this.robotsOurs, fieldX, fieldY);
        const theirHit = this.nearestRobot(this.robotsTheirs, fieldX, fieldY);
        const ballDist = Math.hypot(this.ballPos.x - fieldX, this.ballPos.y - fieldY);

        if (ourHit) candidates.push({ kind: 'our', dist: ourHit.dist, id: ourHit.id });
        if (ballDist < BALL_HIT_RADIUS_M) candidates.push({ kind: 'ball', dist: ballDist });
        if (theirHit) candidates.push({ kind: 'their', dist: theirHit.dist, id: theirHit.id });

        candidates.sort((a, b) => {
            if (Math.abs(a.dist - b.dist) > 1e-6) return a.dist - b.dist;
            const rank = { ball: 0, our: 1, their: 2 };
            return rank[a.kind] - rank[b.kind];
        });

        const best = candidates[0] ?? null;
        if (!best) return null;
        if (best.kind === 'ball') return { type: 'ball' };
        const map = best.kind === 'our' ? this.robotsOurs : this.robotsTheirs;
        const yellow = best.kind === 'our' ? this.isYellow : !this.isYellow;
        return { type: 'robot', id: best.id, yellow, theta: map[best.id]?.theta ?? 0 };
    }
}
