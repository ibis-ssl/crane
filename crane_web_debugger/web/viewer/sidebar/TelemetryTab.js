// フォーカスサイドバーの「テレメトリ」タブ。
//
// 1440px の 2×2 グリッドは 360px には入らないので、単列積みの縮約版にする。
// X/Y・Vx/Vy の系列トグル（凡例クリック）は Chart.js の標準機能として維持。
//
// 購読は WsHub 経由。タブを閉じたら必ず解除する。ここで new WebSocket すると
// タブを往復するたびに接続が増える。

import { ChartHost, makeDataset } from './ChartHost.js';

const MAX_POINTS = 300;
const CHART_H = 150;

export class TelemetryTab {
    constructor(viewer) {
        this._v = viewer;
        this._root = null;
        this._charts = {};
        this._unsubs = [];
        this._startMs = 0;
        this._id = null;
        // activate() は Chart.js の読み込みと rAF を await する。その間にタブが
        // 切り替わると、解除済みのタブが後から購読を張ってしまう（タブを 10 往復
        // させると world_model の購読が 2 → 11 に増えることを実測した）。
        // 世代番号で、古い継続が state を触るのを弾く。
        this._gen = 0;
    }

    get label() { return 'テレメトリ'; }

    mount(container) {
        this._root = document.createElement('div');
        this._root.className = 'rd-telemetry';
        this._root.innerHTML = `
            <div class="rd-section-title">Position X / Y</div>
            <div class="rd-chart" style="height:${CHART_H}px"><canvas id="tl-chart-pos"></canvas></div>
            <div class="rd-section-title">Velocity Vx / Vy</div>
            <div class="rd-chart" style="height:${CHART_H}px"><canvas id="tl-chart-vel"></canvas></div>
            <div class="rd-section-title">Angle θ</div>
            <div class="rd-chart" style="height:${CHART_H}px"><canvas id="tl-chart-theta"></canvas></div>
            <div class="rd-chart-note">実線 = 推定値 / 破線 = 目標・センサ値。凡例をクリックで系列の表示切替</div>
        `;
        container.appendChild(this._root);
    }

    async activate(id) {
        const gen = ++this._gen;
        this._id = id;
        // フォーカスが変わったら経過時間を 0 に戻す。戻さないと x 軸が飛ぶ
        this._startMs = Date.now();

        const cs = getComputedStyle(document.documentElement);
        const c = (k, fb) => cs.getPropertyValue(k).trim() || fb;
        const c1 = c('--crane-chart-1', '#5B4BE0');
        const c2 = c('--crane-chart-2', '#00959F');
        const c3 = c('--crane-chart-3', '#B02D6B');
        const c4 = c('--crane-chart-4', '#C07400');

        const charts = {};
        charts.pos = new ChartHost(
            this._root.querySelector('#tl-chart-pos'),
            [
                makeDataset('Est. X', c1), makeDataset('Est. Y', c2),
                makeDataset('Target X', c3, true), makeDataset('Target Y', c4, true),
            ], { yLabel: 'm', maxPoints: MAX_POINTS });
        charts.vel = new ChartHost(
            this._root.querySelector('#tl-chart-vel'),
            [
                makeDataset('Est. Vx', c1), makeDataset('Est. Vy', c2),
                makeDataset('Target Vx', c3, true), makeDataset('Target Vy', c4, true),
                makeDataset('Odom Vx', c3, true),
            ], { yLabel: 'm/s', maxPoints: MAX_POINTS });
        charts.theta = new ChartHost(
            this._root.querySelector('#tl-chart-theta'),
            [
                makeDataset('Est. θ', c1),
                makeDataset('Target θ', c3, true), makeDataset('Gyro Yaw', c4, true),
            ], { yLabel: 'rad', maxPoints: MAX_POINTS });

        try {
            await Promise.all(Object.values(charts).map(ch => ch.create()));
        } catch (e) {
            if (gen === this._gen && this._root) {
                this._root.innerHTML = `<div class="rd-log__empty">グラフを表示できません: ${e.message}</div>`;
            }
            return;
        }
        if (gen !== this._gen) {
            // 生成を待っている間にタブが切り替わった。作った Chart だけ畳んで抜ける
            for (const ch of Object.values(charts)) ch.destroy();
            return;
        }
        this._charts = charts;
        this._subscribe();
    }

    deactivate() {
        this._gen++;
        for (const un of this._unsubs) un();
        this._unsubs = [];
        for (const ch of Object.values(this._charts)) ch.destroy();
        this._charts = {};
        this._root = null;
    }

    // 受信のたびに再描画するので、FocusSidebar からの定期 refresh では何もしない
    refresh() {}

    _subscribe() {
        const hub = this._v.hub;
        this._unsubs.push(hub.subscribe('world_model', (d) => this._onWorldModel(d)));
        this._unsubs.push(hub.subscribe('control_targets', (d) => this._onTargets(d)));
        this._unsubs.push(hub.subscribe('robot_feedback', (d) => this._onFeedback(d)));
    }

    get _t() { return (Date.now() - this._startMs) / 1000; }

    _onWorldModel(data) {
        const robot = data.robots_ours?.find(r => r.id === this._id);
        if (!robot) return;
        const t = this._t;
        this._charts.pos?.push(0, t, robot.x);
        this._charts.pos?.push(1, t, robot.y);
        this._charts.vel?.push(0, t, robot.vx);
        this._charts.vel?.push(1, t, robot.vy);
        this._charts.theta?.push(0, t, robot.theta);
        this._flushAll();
    }

    _onTargets(data) {
        const cmd = data.commands?.find(c => c.robot_id === this._id);
        if (!cmd) return;
        const t = this._t;
        if (cmd.position_target_mode) {
            this._charts.pos?.push(2, t, cmd.position_target_mode.target_x);
            this._charts.pos?.push(3, t, cmd.position_target_mode.target_y);
        }
        if (cmd.simple_velocity_target_mode) {
            this._charts.vel?.push(2, t, cmd.simple_velocity_target_mode.target_vx);
            this._charts.vel?.push(3, t, cmd.simple_velocity_target_mode.target_vy);
        }
        this._charts.theta?.push(1, t, cmd.target_theta);
        this._flushAll();
    }

    _onFeedback(data) {
        const fb = data.robots?.find(r => r.robot_id === this._id);
        if (!fb) return;
        const t = this._t;
        if (fb.odom_speed?.length >= 1) this._charts.vel?.push(4, t, fb.odom_speed[0]);
        this._charts.theta?.push(2, t, fb.yaw_angle);
        this._flushAll();
    }

    _flushAll() {
        for (const ch of Object.values(this._charts)) ch.flush();
    }
}
