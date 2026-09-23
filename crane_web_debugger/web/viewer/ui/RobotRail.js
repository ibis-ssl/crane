// 右端 56px のロボットレール。
//
// 番号の位置は動かさない。未検出でも薄く残し、異常があれば枠で示す。
// 「いつもの場所に 7 番が居る」ことが、試合中に目で探す時間をいちばん削る。
//
// 異常表示は robot_feedback が実際に届いていることが前提。

import { FEEDBACK_STALE_MS, VOLTAGE_CRIT_V, VOLTAGE_WARN_V, TEMP_CRIT_C } from '../renderer/constants.js';

const RAIL_ROBOT_COUNT = 13;
const RAIL_REFRESH_MS = 500;

export class RobotRail {
    constructor(viewer) {
        this._v = viewer;
        this._btns = new Map();   // id -> button
        this._build();
    }

    _build() {
        const rail = document.getElementById('robot-rail');
        if (!rail) return;
        for (let id = 0; id < RAIL_ROBOT_COUNT; id++) {
            const btn = document.createElement('button');
            btn.type = 'button';
            btn.className = 'cv-rail-btn';
            btn.textContent = String(id);
            btn.title = `ロボット ${id} の詳細を開く`;
            btn.setAttribute('aria-label', `ロボット ${id} の詳細を開く`);
            btn.addEventListener('click', () => this._v.toggleRobotDetail(id));
            rail.appendChild(btn);
            this._btns.set(id, btn);
        }
        this._timer = setInterval(() => this.sync(), RAIL_REFRESH_MS);
        this._v.state.addEventListener('focus', () => this.sync());
        this._v.state.addEventListener('feedback', () => this.sync());
        this.sync();
    }

    sync() {
        const state = this._v.state;
        const now = Date.now();
        for (const [id, btn] of this._btns) {
            btn.classList.toggle('selected', id === state.focusedRobotId);
            btn.classList.toggle('absent', !state.robotsOurs[id]);
            const level = this._alertLevel(id, now);
            btn.classList.toggle('danger', level === 'danger');
            btn.classList.toggle('warn', level === 'warn');
            btn.title = level === 'none'
                ? `ロボット ${id} の詳細を開く`
                : `ロボット ${id}: ${this._alertText(id)}`;
        }
    }

    _alertLevel(id, now) {
        const fb = this._v.state.robotFeedback[id];
        if (!fb) return 'none';
        // stale なフィードバックで警告を出し続けない（切断直後の残像になる）
        if (now - (this._v.state.feedbackTimestamp[id] ?? 0) > FEEDBACK_STALE_MS) return 'none';
        if ((fb.error_id ?? 0) !== 0) return 'danger';
        if (fb.voltage != null && fb.voltage <= VOLTAGE_CRIT_V) return 'danger';
        if (Math.max(...(fb.temperatures ?? [0])) >= TEMP_CRIT_C) return 'danger';
        if (fb.voltage != null && fb.voltage <= VOLTAGE_WARN_V) return 'warn';
        return 'none';
    }

    _alertText(id) {
        const fb = this._v.state.robotFeedback[id] ?? {};
        const parts = [];
        if ((fb.error_id ?? 0) !== 0) parts.push(`エラー id=${fb.error_id}`);
        if (fb.voltage != null && fb.voltage <= VOLTAGE_WARN_V) parts.push(`電圧 ${fb.voltage.toFixed(1)}V`);
        const maxT = Math.max(...(fb.temperatures ?? [0]));
        if (maxT >= TEMP_CRIT_C) parts.push(`温度 ${maxT.toFixed(0)}℃`);
        return parts.join(' / ') || '正常';
    }
}
