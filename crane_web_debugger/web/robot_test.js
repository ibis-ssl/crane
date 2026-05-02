'use strict';

// ---- 定数 ----
const ROBOT_HIT_RADIUS_M = 0.15;
const BALL_HIT_RADIUS_M = 0.08;
const FIELD_MARGIN_M = 1.0;
const CHART_BUF = 300;  // 10Hz × 30秒

// ---- Chart.js ヘルパー ----
function makeDataset(label, color, dashed = false) {
    return {
        label,
        data: [],
        borderColor: color,
        backgroundColor: color + '22',
        borderWidth: dashed ? 1.5 : 2,
        borderDash: dashed ? [4, 3] : [],
        pointRadius: 0,
        tension: 0.2,
    };
}

function makeSpeedChart(canvasId) {
    const canvas = document.getElementById(canvasId);
    if (!canvas) return null;
    return new Chart(canvas, {
        type: 'line',
        data: {
            labels: [],
            datasets: [
                makeDataset('Cmd Speed', '#FFA726'),
                makeDataset('Est Speed', '#A0C4FF'),
                makeDataset('Odom Speed', '#66BB6A', true),
                makeDataset('Mouse Speed', '#D0BCFF', true),
            ],
        },
        options: {
            responsive: true,
            maintainAspectRatio: false,
            animation: false,
            scales: {
                x: {
                    type: 'linear',
                    title: { display: true, text: 'Elapsed (s)', color: '#8C929A', font: { size: 10 } },
                    ticks: { color: '#8C929A', maxTicksLimit: 6, font: { size: 10 } },
                    grid: { color: '#2D3748' },
                },
                y: {
                    min: 0,
                    title: { display: true, text: 'm/s', color: '#8C929A', font: { size: 10 } },
                    ticks: { color: '#8C929A', maxTicksLimit: 5, font: { size: 10 } },
                    grid: { color: '#2D3748' },
                },
            },
            plugins: {
                legend: {
                    labels: { color: '#8C929A', font: { size: 10 }, boxWidth: 12, padding: 6 },
                },
                tooltip: {
                    mode: 'index',
                    intersect: false,
                    backgroundColor: '#213052',
                    titleColor: '#A0C4FF',
                    bodyColor: '#E2E4E8',
                    titleFont: { size: 10 },
                    bodyFont: { size: 10 },
                },
            },
        },
    });
}

// ---- Canvas Field Renderer ----
class FieldRenderer {
    constructor(canvas, controller) {
        this.canvas = canvas;
        this.ctx = canvas.getContext('2d');
        this.controller = controller;
        this.needsRedraw = true;
        this._dpr = window.devicePixelRatio || 1;
        this._rafId = null;

        this._resizeObserver = new ResizeObserver(() => this._onResize());
        this._resizeObserver.observe(canvas.parentElement);
        this._onResize();
        this._startLoop();
    }

    _onResize() {
        const container = this.canvas.parentElement;
        const cssW = container.clientWidth || 1;
        const cssH = container.clientHeight || 1;
        this._dpr = window.devicePixelRatio || 1;
        this.canvas.width = Math.round(cssW * this._dpr);
        this.canvas.height = Math.round(cssH * this._dpr);
        this.canvas.style.width = cssW + 'px';
        this.canvas.style.height = cssH + 'px';
        this.invalidate();
    }

    invalidate() { this.needsRedraw = true; }

    _startLoop() {
        const loop = () => {
            if (this.needsRedraw) {
                this.needsRedraw = false;
                this._render();
            }
            this._rafId = requestAnimationFrame(loop);
        };
        this._rafId = requestAnimationFrame(loop);
    }

    stop() {
        if (this._rafId) { cancelAnimationFrame(this._rafId); this._rafId = null; }
        this._resizeObserver.disconnect();
    }

    _getFieldDims() {
        const c = this.controller;
        const halfL = (c.fieldLength / 2 + FIELD_MARGIN_M) * 1000;
        const halfW = (c.fieldWidth / 2 + FIELD_MARGIN_M) * 1000;
        return { vbX: -halfL, vbY: -halfW, vbW: halfL * 2, vbH: halfW * 2 };
    }

    _getVP() {
        const dpr = this._dpr;
        const cssW = this.canvas.width / dpr;
        const cssH = this.canvas.height / dpr;
        const { vbX, vbY, vbW, vbH } = this._getFieldDims();
        const scaleX = cssW / vbW;
        const scaleY = cssH / vbH;
        const vs = Math.min(scaleX, scaleY);
        const ox = (cssW - vbW * vs) / 2;
        const oy = (cssH - vbH * vs) / 2;
        return { dpr, cssW, cssH, vs, ox, oy, vbX, vbY, vbW, vbH };
    }

    _render() {
        const ctx = this.ctx;
        const vp = this._getVP();
        const c = this.controller;
        const hasSel = c.selectedRobotId !== null;

        ctx.setTransform(1, 0, 0, 1, 0, 0);
        ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);

        ctx.save();
        ctx.scale(vp.dpr, vp.dpr);
        ctx.translate(vp.ox, vp.oy);
        ctx.scale(vp.vs, vp.vs);
        ctx.translate(-vp.vbX, -vp.vbY);
        ctx.scale(c.zoomLevel, c.zoomLevel);

        // フィールド背景
        ctx.fillStyle = '#1a5c1a';
        ctx.fillRect(vp.vbX, vp.vbY, vp.vbW, vp.vbH);
        this._drawGrid(ctx, vp);
        this._drawFieldLines(ctx);

        // ボール（薄いオレンジ）
        this._drawBall(ctx, c);

        // 敵ロボット（薄い灰色）
        for (const [id, robot] of Object.entries(c.robotsTheirs)) {
            if (!robot.available_vision && !robot.available_tracker) continue;
            this._drawRobot(ctx, robot, Number(id), '#888888', null, { dim: true });
        }

        // 味方ロボット（非選択ロボットは選択中がいる場合に薄表示）
        for (const [id, robot] of Object.entries(c.robotsOurs)) {
            if (!robot.available_vision && !robot.available_tracker) continue;
            const numId = Number(id);
            const isSelected = numId === c.selectedRobotId;
            const color = c.isYellow ? '#FFD700' : '#4488FF';
            const dim = hasSel && !isSelected;
            this._drawRobot(ctx, robot, numId, color, c.selectedRobotId, { dim });
        }

        // 選択ハイライト（破線ハロー）
        const selRobot = c.selectedRobotId !== null ? c.robotsOurs[c.selectedRobotId] : null;
        if (selRobot && (selRobot.available_vision || selRobot.available_tracker)) {
            const svgX = selRobot.x * 1000;
            const svgY = -selRobot.y * 1000;
            ctx.save();
            ctx.strokeStyle = '#00ffff';
            ctx.lineWidth = 20;
            ctx.setLineDash([40, 20]);
            ctx.globalAlpha = 0.9;
            ctx.beginPath();
            ctx.arc(svgX, svgY, ROBOT_HIT_RADIUS_M * 1000 * 1.4, 0, Math.PI * 2);
            ctx.stroke();
            ctx.restore();
        }

        // ターゲットマーカー
        if (c.targetPos !== null) {
            const tx = c.targetPos.x * 1000;
            const ty = -c.targetPos.y * 1000;
            const s = 120;
            ctx.save();
            ctx.strokeStyle = '#ff4444';
            ctx.lineWidth = 20;
            ctx.globalAlpha = 0.9;
            ctx.beginPath();
            ctx.moveTo(tx - s, ty); ctx.lineTo(tx + s, ty);
            ctx.moveTo(tx, ty - s); ctx.lineTo(tx, ty + s);
            ctx.stroke();
            ctx.beginPath();
            ctx.arc(tx, ty, s * 0.6, 0, Math.PI * 2);
            ctx.strokeStyle = '#ff4444';
            ctx.lineWidth = 15;
            ctx.stroke();
            ctx.restore();
        }

        ctx.restore();

        // データなし表示
        if (Object.keys(c.robotsOurs).length === 0 && Object.keys(c.robotsTheirs).length === 0) {
            ctx.save();
            ctx.fillStyle = '#666';
            ctx.font = `${16 * vp.dpr}px sans-serif`;
            ctx.textAlign = 'center';
            ctx.textBaseline = 'middle';
            ctx.fillText('世界モデル待機中...', this.canvas.width / 2, this.canvas.height / 2);
            ctx.restore();
        }
    }

    _drawBall(ctx, c) {
        if (!c.ballPos) return;
        const bx = c.ballPos.x * 1000;
        const by = -c.ballPos.y * 1000;
        const r = BALL_HIT_RADIUS_M * 1000;
        ctx.save();
        ctx.beginPath();
        ctx.arc(bx, by, r, 0, Math.PI * 2);
        ctx.fillStyle = '#FFA726';
        ctx.globalAlpha = 0.45;
        ctx.fill();
        ctx.strokeStyle = '#FFA726';
        ctx.lineWidth = 8;
        ctx.globalAlpha = 0.6;
        ctx.stroke();
        ctx.restore();
    }

    _drawRobot(ctx, robot, numId, fillColor, selectedId, opts = {}) {
        const dim = opts.dim ?? false;
        const isSelected = numId === selectedId;
        const svgX = robot.x * 1000;
        const svgY = -robot.y * 1000;
        const r = ROBOT_HIT_RADIUS_M * 1000;

        const fillAlpha = dim ? 0.30 : (isSelected ? 1.0 : 0.85);
        const strokeAlpha = dim ? 0.35 : 1.0;
        const strokeW = dim ? 4 : (isSelected ? 25 : 10);
        const strokeColor = isSelected ? '#00ffff' : '#ffffff';
        const textAlpha = dim ? 0.5 : 1.0;

        ctx.save();

        // 本体
        ctx.beginPath();
        ctx.arc(svgX, svgY, r, 0, Math.PI * 2);
        ctx.fillStyle = fillColor;
        ctx.globalAlpha = fillAlpha;
        ctx.fill();
        ctx.strokeStyle = strokeColor;
        ctx.lineWidth = strokeW;
        ctx.globalAlpha = strokeAlpha;
        ctx.stroke();

        // 方向矢印（dimの時は省略）
        if (!dim) {
            ctx.beginPath();
            ctx.moveTo(svgX, svgY);
            ctx.lineTo(
                svgX + Math.cos(robot.theta) * r * 0.9,
                svgY - Math.sin(robot.theta) * r * 0.9
            );
            ctx.strokeStyle = '#ffffff';
            ctx.lineWidth = 12;
            ctx.globalAlpha = 0.9;
            ctx.stroke();
        }

        // ID テキスト
        ctx.fillStyle = '#ffffff';
        ctx.globalAlpha = textAlpha;
        ctx.font = `bold ${r * 0.8}px sans-serif`;
        ctx.textAlign = 'center';
        ctx.textBaseline = 'middle';
        ctx.fillText(String(numId), svgX, svgY);

        ctx.restore();
    }

    _drawGrid(ctx, vp) {
        ctx.save();
        ctx.strokeStyle = '#2d8a2d';
        ctx.lineWidth = 15;
        ctx.globalAlpha = 0.3;
        ctx.beginPath();
        const xStart = Math.floor(vp.vbX / 1000) * 1000;
        const xEnd = -vp.vbX;
        const yStart = Math.floor(vp.vbY / 1000) * 1000;
        const yEnd = -vp.vbY;
        for (let x = xStart; x <= xEnd; x += 1000) {
            ctx.moveTo(x, vp.vbY); ctx.lineTo(x, -vp.vbY);
        }
        for (let y = yStart; y <= yEnd; y += 1000) {
            ctx.moveTo(vp.vbX, y); ctx.lineTo(-vp.vbX, y);
        }
        ctx.stroke();
        ctx.restore();
    }

    _drawFieldLines(ctx) {
        const c = this.controller;
        const halfL = c.fieldLength * 500;
        const halfW = c.fieldWidth * 500;
        const paDepth = c.penaltyDepth * 1000;
        const paHalfW = c.penaltyWidth * 500;
        const goalHalfW = c.goalWidth * 500;
        const goalDepth = c.goalDepth * 1000;
        const ccR = c.centerCircleRadius * 1000;

        ctx.save();
        ctx.strokeStyle = '#ffffff';
        ctx.lineWidth = 20;
        ctx.globalAlpha = 0.7;
        ctx.beginPath();
        ctx.rect(-halfL, -halfW, halfL * 2, halfW * 2);
        ctx.moveTo(0, -halfW); ctx.lineTo(0, halfW);
        ctx.moveTo(ccR, 0);
        ctx.arc(0, 0, ccR, 0, Math.PI * 2);
        ctx.rect(-halfL, -paHalfW, paDepth, paHalfW * 2);
        ctx.rect(halfL - paDepth, -paHalfW, paDepth, paHalfW * 2);
        ctx.stroke();

        ctx.strokeStyle = '#ffff88';
        ctx.lineWidth = 15;
        ctx.globalAlpha = 0.6;
        ctx.beginPath();
        ctx.rect(-halfL - goalDepth, -goalHalfW, goalDepth, goalHalfW * 2);
        ctx.rect(halfL, -goalHalfW, goalDepth, goalHalfW * 2);
        ctx.stroke();

        ctx.globalAlpha = 0.8;
        ctx.fillStyle = '#ffffff';
        ctx.beginPath();
        ctx.arc(0, 0, 40, 0, Math.PI * 2);
        ctx.fill();
        ctx.restore();
    }

    clientToFieldCoords(clientX, clientY) {
        const rect = this.canvas.getBoundingClientRect();
        const cssX = clientX - rect.left;
        const cssY = clientY - rect.top;
        const { vs, ox, oy, vbX, vbY } = this._getVP();
        const zoom = this.controller.zoomLevel;
        const svgX = (cssX - ox) / (zoom * vs) + vbX;
        const svgY = (cssY - oy) / (zoom * vs) + vbY;
        return { x: svgX / 1000, y: -svgY / 1000 };
    }
}

// ---- メインコントローラー ----
class RobotTestController {
    constructor() {
        this.websocket = null;

        // フィールド状態
        this.robotsOurs = {};
        this.robotsTheirs = {};
        this.ballPos = null;
        this.isYellow = false;
        this.selectedRobotId = null;
        this.targetPos = null;

        // フィールド寸法（Division A デフォルト）
        this.fieldLength = 12.0;
        this.fieldWidth = 9.0;
        this.penaltyDepth = 1.8;
        this.penaltyWidth = 3.6;
        this.goalDepth = 0.18;
        this.goalWidth = 1.2;
        this.centerCircleRadius = 0.5;

        // ビュー状態
        this.zoomLevel = 1.0;

        // テストモード状態
        this.testModeActive = false;
        this.cursorFollowMode = false;

        // 設定値
        this.maxVelocity = 2.0;
        this.maxAcceleration = 2.5;
        this.targetThetaDeg = 0;
        this.velocityDampingGain = 0.5;
        this._dampingSendTimer = null;

        // チャートバッファ
        this.startTime = null;
        this.chartLabels = [];
        this.cmdSpeedBuf = [];
        this.estSpeedBuf = [];
        this.odomSpeedBuf = [];
        this.mouseSpeedBuf = [];

        this.renderer = null;
        this.chart = null;

        this.init();
    }

    init() {
        const canvas = document.getElementById('field-canvas');
        if (canvas) this.renderer = new FieldRenderer(canvas, this);

        this.chart = makeSpeedChart('chart-speed');
        this.setupControls();
        this.setupCanvasEvents();
        this.setupWebSocket();
        this.setStatus(false);
        this._updateUiState();
    }

    // ---- WebSocket ----
    setupWebSocket() {
        const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
        const wsUrl = `${protocol}//${window.location.hostname}:8091`;
        this.websocket = new WebSocket(wsUrl);
        this.websocket.onopen = () => this.setStatus(true);
        this.websocket.onclose = () => {
            this.setStatus(false);
            setTimeout(() => {
                if (!this.websocket || this.websocket.readyState === WebSocket.CLOSED) {
                    this.setupWebSocket();
                }
            }, 3000);
        };
        this.websocket.onerror = () => this.setStatus(false);
        this.websocket.onmessage = (event) => {
            try { this.handleMessage(JSON.parse(event.data)); } catch (e) { console.error(e); }
        };
    }

    wsSend(obj) {
        if (this.websocket?.readyState === WebSocket.OPEN) {
            this.websocket.send(JSON.stringify(obj));
        }
    }

    handleMessage(data) {
        switch (data.type) {
            case 'world_model':    this.onWorldModel(data);     break;
            case 'robot_commands': this.onRobotCommands(data);  break;
            case 'control_targets': this.onControlTargets(data); break;
            case 'robot_feedback': this.onRobotFeedback(data);  break;
        }
    }

    onWorldModel(data) {
        if (data.is_yellow !== undefined) this.isYellow = data.is_yellow;

        if (data.robots_ours) {
            this.robotsOurs = {};
            for (const r of data.robots_ours) this.robotsOurs[r.id] = r;
        }
        if (data.robots_theirs) {
            this.robotsTheirs = {};
            for (const r of data.robots_theirs) this.robotsTheirs[r.id] = r;
        }

        // ボール位置
        if (data.ball) {
            this.ballPos = { x: data.ball.x, y: data.ball.y };
        }

        // フィールド寸法
        if (data.field_info) {
            if (data.field_info.length > 0) this.fieldLength = data.field_info.length;
            if (data.field_info.width > 0) this.fieldWidth = data.field_info.width;
        }
        if (data.penalty_area_size) {
            if (data.penalty_area_size.depth > 0) this.penaltyDepth = data.penalty_area_size.depth;
            if (data.penalty_area_size.width > 0) this.penaltyWidth = data.penalty_area_size.width;
        }
        if (data.goal_size) {
            if (data.goal_size.depth > 0) this.goalDepth = data.goal_size.depth;
            if (data.goal_size.width > 0) this.goalWidth = data.goal_size.width;
        }

        this.renderer?.invalidate();

        // Est Speed プロット
        if (this.selectedRobotId !== null && this.robotsOurs[this.selectedRobotId]) {
            const r = this.robotsOurs[this.selectedRobotId];
            const estSpeed = Math.sqrt((r.vx ?? 0) ** 2 + (r.vy ?? 0) ** 2);
            document.getElementById('readout-est-speed').textContent = estSpeed.toFixed(2) + ' m/s';
            this._pushChartPoint('est', estSpeed);
        }
    }

    onControlTargets(data) {
        if (!data.commands || this.selectedRobotId === null) return;
        const cmd = data.commands.find(c => c.robot_id === this.selectedRobotId);
        if (!cmd) return;
        const cmdSpeed = cmd.polar_velocity_target_mode?.target_velocity_r;
        if (Number.isFinite(cmdSpeed)) this._pushChartPoint('cmd', cmdSpeed);
    }

    onRobotCommands(data) {
        if (!data.commands || this.selectedRobotId === null) return;
        const cmd = data.commands.find(c => c.robot_id === this.selectedRobotId);
        if (!cmd) return;
        const cmdSpeed = cmd.polar_velocity_target_mode?.target_velocity_r;
        if (Number.isFinite(cmdSpeed)) this._pushChartPoint('cmd', cmdSpeed);
    }

    onRobotFeedback(data) {
        if (!data.robots || this.selectedRobotId === null) return;
        const fb = data.robots.find(r => r.robot_id === this.selectedRobotId);
        if (!fb) return;

        if (fb.odom_speed && fb.odom_speed.length >= 2) {
            const odomSpeed = Math.sqrt(fb.odom_speed[0] ** 2 + fb.odom_speed[1] ** 2);
            document.getElementById('readout-odom-speed').textContent = odomSpeed.toFixed(2) + ' m/s';
            this._pushChartPoint('odom', odomSpeed);
        }
        if (fb.mouse_vel && fb.mouse_vel.length >= 2) {
            const mouseSpeed = Math.sqrt(fb.mouse_vel[0] ** 2 + fb.mouse_vel[1] ** 2);
            this._pushChartPoint('mouse', mouseSpeed);
        }
    }

    // ---- チャート更新 ----
    _getElapsed() {
        const now = performance.now() / 1000;
        if (this.startTime === null) this.startTime = now;
        return Math.round((now - this.startTime) * 10) / 10;
    }

    _pushChartPoint(series, value) {
        if (!this.chart) return;
        const t = this._getElapsed();
        const datasets = this.chart.data.datasets;
        const labels = this.chart.data.labels;
        const idxMap = { cmd: 0, est: 1, odom: 2, mouse: 3 };
        const idx = idxMap[series];
        if (idx === undefined) return;

        if (series === 'est') {
            labels.push(t);
            if (labels.length > CHART_BUF) labels.shift();
        }
        datasets[idx].data.push({ x: t, y: value });
        if (datasets[idx].data.length > CHART_BUF) datasets[idx].data.shift();
        this.chart.update('none');
    }

    resetChart() {
        if (!this.chart) return;
        this.startTime = null;
        this.chart.data.labels = [];
        for (const ds of this.chart.data.datasets) ds.data = [];
        this.chart.update('none');
    }

    // ---- 操作 ----
    activateTest() {
        this.wsSend({ type: 'activate_robot_test' });
        this.testModeActive = true;
        this._updateUiState();
    }

    deactivateTest() {
        this.wsSend({ type: 'deactivate_robot_test' });
        this.testModeActive = false;
        this.cursorFollowMode = false;
        this.selectedRobotId = null;
        this.targetPos = null;
        this.renderer?.invalidate();
        this._updateUiState();
        document.getElementById('readout-est-speed').textContent = '-- m/s';
        document.getElementById('readout-odom-speed').textContent = '-- m/s';
        const canvas = document.getElementById('field-canvas');
        if (canvas) canvas.style.cursor = 'crosshair';
    }

    setCursorFollowMode(active) {
        this.cursorFollowMode = active;
        const canvas = document.getElementById('field-canvas');
        if (canvas) canvas.style.cursor = active ? 'none' : 'crosshair';
        this._updateUiState();
    }

    clearTarget() {
        this.targetPos = null;
        this.renderer?.invalidate();
        this._updateUiState();
    }

    deselectRobot() {
        this.selectRobot(null);
        this._updateUiState();
    }

    sendTarget() {
        if (this.selectedRobotId === null || this.targetPos === null) return;
        if (!this.testModeActive) return;
        this.wsSend({
            type: 'robot_test_target',
            robot_id: this.selectedRobotId,
            target_x: this.targetPos.x,
            target_y: this.targetPos.y,
            target_theta: this.targetThetaDeg * Math.PI / 180,
            max_velocity: this.maxVelocity,
            max_acceleration: this.maxAcceleration,
        });
        this._updateUiState();
    }

    sendPlannerParam() {
        this.wsSend({
            type: 'set_planner_param',
            velocity_damping_gain: this.velocityDampingGain,
        });
    }

    selectRobot(id) {
        this.selectedRobotId = id;
        document.getElementById('readout-est-speed').textContent = '-- m/s';
        document.getElementById('readout-odom-speed').textContent = '-- m/s';
        this.resetChart();
        this.renderer?.invalidate();
        this._updateUiState();
    }

    findRobotAtPosition(fieldX, fieldY) {
        let closest = null;
        let minDist = ROBOT_HIT_RADIUS_M;
        for (const [id, robot] of Object.entries(this.robotsOurs)) {
            if (!robot.available_vision && !robot.available_tracker) continue;
            const dx = robot.x - fieldX;
            const dy = robot.y - fieldY;
            const dist = Math.sqrt(dx * dx + dy * dy);
            if (dist < minDist) { minDist = dist; closest = Number(id); }
        }
        return closest;
    }

    // ---- UI 状態一括更新 ----
    _updateUiState() {
        const active = this.testModeActive;
        const follow = this.cursorFollowMode;
        const hasSel = this.selectedRobotId !== null;
        const hasTgt = this.targetPos !== null;

        // --- 状態カード ---
        const card = document.getElementById('mode-card');
        const cardIcon = document.getElementById('mode-card-icon');
        const cardLabel = document.getElementById('mode-card-label');
        const cardSub = document.getElementById('mode-card-sub');
        if (card) {
            card.className = 'mode-card' + (follow ? ' mode-card--follow' : active ? ' mode-card--active' : '');
        }
        if (cardIcon) {
            cardIcon.textContent = follow ? 'mouse' : active ? 'play_circle' : 'radio_button_unchecked';
        }
        if (cardLabel) {
            cardLabel.textContent = follow ? 'CURSOR FOLLOW' : active ? 'ROBOT TEST ACTIVE' : 'INACTIVE';
        }
        if (cardSub) {
            if (!active) {
                cardSub.textContent = 'Activate を押してテストを開始';
            } else if (follow) {
                cardSub.textContent = 'マウスで目標位置を追従中';
            } else if (hasSel) {
                cardSub.textContent = `Robot ${this.selectedRobotId} を操作中`;
            } else {
                cardSub.textContent = 'フィールドのロボットをクリックして選択';
            }
        }

        // --- Activate / Deactivate ボタン ---
        const btnActivate = document.getElementById('btn-activate');
        const btnDeactivate = document.getElementById('btn-deactivate');
        if (btnActivate) btnActivate.style.display = active ? 'none' : '';
        if (btnDeactivate) btnDeactivate.style.display = active ? '' : 'none';

        // --- Cursor Follow ボタン ---
        const btnFollow = document.getElementById('btn-cursor-follow');
        if (btnFollow) {
            btnFollow.style.display = (active && hasSel) ? '' : 'none';
            btnFollow.className = follow
                ? 'm3-btn m3-btn--filled m3-btn--sm'
                : 'm3-btn m3-btn--tonal m3-btn--sm';
        }

        // --- Selected Robot セクション ---
        const robotIdEl = document.getElementById('selected-robot-id');
        const targetPosEl = document.getElementById('target-pos-display');
        const btnDeselect = document.getElementById('btn-deselect');
        const btnClearTarget = document.getElementById('btn-clear-target');

        if (robotIdEl) robotIdEl.textContent = hasSel ? String(this.selectedRobotId) : '--';
        if (targetPosEl) {
            if (hasTgt) {
                targetPosEl.textContent = `(${this.targetPos.x.toFixed(2)}, ${this.targetPos.y.toFixed(2)})`;
                targetPosEl.style.color = 'var(--md-sys-color-tertiary)';
            } else {
                targetPosEl.textContent = '--';
                targetPosEl.style.color = '';
            }
        }
        if (btnDeselect) btnDeselect.style.display = hasSel ? '' : 'none';
        if (btnClearTarget) btnClearTarget.style.display = hasTgt ? '' : 'none';

        // --- スライダーセクションの有効/無効 ---
        const sectSpeed = document.getElementById('sect-speed-limits');
        const sectAngle = document.getElementById('sect-target-angle');
        const disableSliders = !active || !hasSel;
        for (const sect of [sectSpeed, sectAngle]) {
            if (!sect) continue;
            sect.style.opacity = disableSliders ? '0.45' : '1';
            sect.style.pointerEvents = disableSliders ? 'none' : '';
        }

        // --- フィールド枠のアウトライン ---
        const fieldContainer = document.getElementById('field-container');
        if (fieldContainer) {
            fieldContainer.classList.toggle('is-active', active && !follow);
            fieldContainer.classList.toggle('is-cursor-follow', follow);
        }

        // --- ヒント文 ---
        const hintEl = document.getElementById('hint-text');
        if (hintEl) {
            if (!active) {
                hintEl.textContent = 'Activate を押すとロボット選択・操作が可能になります';
            } else if (follow) {
                hintEl.textContent = 'マウス移動: 目標追従 ｜ クリックまたは Esc: 解除';
            } else if (!hasSel) {
                hintEl.textContent = '味方ロボットをクリックして選択 ｜ ホイール: ズーム ｜ ドラッグ: パン';
            } else {
                hintEl.textContent = 'クリック: 目標設定 ｜ Cursor Follow: マウス追従 ｜ ホイール: ズーム';
            }
        }

        // --- ナビ status pill（既存との同期） ---
        const modeStatus = document.getElementById('mode-status');
        if (modeStatus) {
            if (follow) {
                modeStatus.textContent = 'CURSOR FOLLOW';
                modeStatus.classList.add('active');
            } else if (active) {
                modeStatus.textContent = 'ROBOT TEST ACTIVE';
                modeStatus.classList.add('active');
            } else {
                modeStatus.textContent = 'INACTIVE';
                modeStatus.classList.remove('active');
            }
        }
    }

    setStatus(connected) {
        const dot = document.getElementById('status-dot');
        const label = document.getElementById('status-label');
        const banner = document.getElementById('offline-banner');
        if (dot) dot.classList.toggle('connected', connected);
        if (label) label.textContent = connected ? 'connected' : 'disconnected';
        if (banner) banner.classList.toggle('visible', !connected);
    }

    // ---- UI イベント ----
    setupControls() {
        document.getElementById('btn-activate')?.addEventListener('click', () => this.activateTest());
        document.getElementById('btn-deactivate')?.addEventListener('click', () => this.deactivateTest());
        document.getElementById('btn-cursor-follow')?.addEventListener('click', () => {
            if (this.cursorFollowMode) {
                this.setCursorFollowMode(false);
            } else {
                this.setCursorFollowMode(true);
            }
        });
        document.getElementById('btn-deselect')?.addEventListener('click', () => this.deselectRobot());
        document.getElementById('btn-clear-target')?.addEventListener('click', () => this.clearTarget());

        const velSlider = document.getElementById('slider-max-vel');
        const velDisplay = document.getElementById('display-max-vel');
        velSlider?.addEventListener('input', () => {
            this.maxVelocity = parseFloat(velSlider.value);
            if (velDisplay) velDisplay.textContent = this.maxVelocity.toFixed(1);
            this.sendTarget();
        });

        const accSlider = document.getElementById('slider-max-acc');
        const accDisplay = document.getElementById('display-max-acc');
        accSlider?.addEventListener('input', () => {
            this.maxAcceleration = parseFloat(accSlider.value);
            if (accDisplay) accDisplay.textContent = this.maxAcceleration.toFixed(1);
            this.sendTarget();
        });

        const thetaInput = document.getElementById('input-theta');
        thetaInput?.addEventListener('change', () => {
            this.targetThetaDeg = parseFloat(thetaInput.value) || 0;
            this.sendTarget();
        });

        const dampingSlider = document.getElementById('slider-damping-gain');
        const dampingDisplay = document.getElementById('display-damping-gain');
        dampingSlider?.addEventListener('input', () => {
            this.velocityDampingGain = parseFloat(dampingSlider.value);
            if (dampingDisplay) dampingDisplay.textContent = this.velocityDampingGain.toFixed(2);
            if (this._dampingSendTimer) clearTimeout(this._dampingSendTimer);
            this._dampingSendTimer = setTimeout(() => this.sendPlannerParam(), 100);
        });
    }

    setupCanvasEvents() {
        const canvas = document.getElementById('field-canvas');
        if (!canvas) return;

        canvas.addEventListener('wheel', (e) => {
            e.preventDefault();
            const factor = e.deltaY < 0 ? 1.1 : 0.9;
            this.zoomLevel = Math.min(Math.max(this.zoomLevel * factor, 0.1), 5.0);
            this.renderer?.invalidate();
        }, { passive: false });

        canvas.addEventListener('click', (e) => {
            if (this.cursorFollowMode) {
                this.setCursorFollowMode(false);
                return;
            }
            const fp = this.renderer.clientToFieldCoords(e.clientX, e.clientY);
            const hitId = this.findRobotAtPosition(fp.x, fp.y);
            if (hitId !== null) {
                this.selectRobot(hitId);
            } else if (this.selectedRobotId !== null && this.testModeActive) {
                this.targetPos = fp;
                this.sendTarget();
                this.renderer?.invalidate();
            }
        });

        canvas.addEventListener('dblclick', (e) => {
            if (!this.testModeActive || this.selectedRobotId === null) return;
            this.setCursorFollowMode(true);
            const fp = this.renderer.clientToFieldCoords(e.clientX, e.clientY);
            this.targetPos = fp;
            this.sendTarget();
            this.renderer?.invalidate();
        });

        canvas.addEventListener('mousemove', (e) => {
            if (!this.cursorFollowMode) return;
            const fp = this.renderer.clientToFieldCoords(e.clientX, e.clientY);
            this.targetPos = fp;
            this.sendTarget();
            this.renderer?.invalidate();
        });

        document.addEventListener('keydown', (e) => {
            if (e.key === 'Escape' && this.cursorFollowMode) {
                this.setCursorFollowMode(false);
            }
        });

        canvas.style.cursor = 'crosshair';
    }
}

document.addEventListener('DOMContentLoaded', () => {
    window.robotTestController = new RobotTestController();
});
