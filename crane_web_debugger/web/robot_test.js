'use strict';

// ---- 定数 ----
const ROBOT_HIT_RADIUS_M = 0.15;
const FIELD_MARGIN_M = 1.0;  // フィールド外側の余白 (m)
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

    // フィールド寸法(mm)をコントローラから取得
    _getFieldDims() {
        const c = this.controller;
        const halfL = (c.fieldLength / 2 + FIELD_MARGIN_M) * 1000;
        const halfW = (c.fieldWidth / 2 + FIELD_MARGIN_M) * 1000;
        return {
            vbX: -halfL, vbY: -halfW,
            vbW: halfL * 2, vbH: halfW * 2,
        };
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

        // 敵ロボット描画（灰色）
        for (const [id, robot] of Object.entries(c.robotsTheirs)) {
            if (!robot.available_vision && !robot.available_tracker) continue;
            this._drawRobot(ctx, robot, Number(id), '#888888', null);
        }

        // 味方ロボット描画
        for (const [id, robot] of Object.entries(c.robotsOurs)) {
            if (!robot.available_vision && !robot.available_tracker) continue;
            const color = c.isYellow ? '#FFD700' : '#4488FF';
            this._drawRobot(ctx, robot, Number(id), color, c.selectedRobotId);
        }

        // 選択ハイライト（破線円）
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

    _drawRobot(ctx, robot, numId, fillColor, selectedId) {
        const svgX = robot.x * 1000;
        const svgY = -robot.y * 1000;
        const r = ROBOT_HIT_RADIUS_M * 1000;

        ctx.save();
        ctx.beginPath();
        ctx.arc(svgX, svgY, r, 0, Math.PI * 2);
        ctx.fillStyle = fillColor;
        ctx.globalAlpha = 0.85;
        ctx.fill();
        ctx.strokeStyle = (numId === selectedId) ? '#00ffff' : '#ffffff';
        ctx.lineWidth = (numId === selectedId) ? 25 : 10;
        ctx.globalAlpha = 1.0;
        ctx.stroke();

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

        ctx.fillStyle = '#ffffff';
        ctx.globalAlpha = 1.0;
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
        const halfL = c.fieldLength * 500;   // mm
        const halfW = c.fieldWidth * 500;    // mm
        const paDepth = c.penaltyDepth * 1000;   // mm
        const paHalfW = c.penaltyWidth * 500;    // mm
        const goalHalfW = c.goalWidth * 500;     // mm
        const goalDepth = c.goalDepth * 1000;    // mm
        const ccR = c.centerCircleRadius * 1000; // mm

        ctx.save();
        ctx.strokeStyle = '#ffffff';
        ctx.lineWidth = 20;
        ctx.globalAlpha = 0.7;
        ctx.beginPath();

        // 外枠
        ctx.rect(-halfL, -halfW, halfL * 2, halfW * 2);

        // センターライン
        ctx.moveTo(0, -halfW); ctx.lineTo(0, halfW);

        // センターサークル
        ctx.moveTo(ccR, 0);
        ctx.arc(0, 0, ccR, 0, Math.PI * 2);

        // 左ペナルティエリア（x < 0 側）
        ctx.rect(-halfL, -paHalfW, paDepth, paHalfW * 2);

        // 右ペナルティエリア（x > 0 側）
        ctx.rect(halfL - paDepth, -paHalfW, paDepth, paHalfW * 2);

        ctx.stroke();

        // ゴール（塗りなし・別色）
        ctx.strokeStyle = '#ffff88';
        ctx.lineWidth = 15;
        ctx.globalAlpha = 0.6;
        ctx.beginPath();

        // 左ゴール
        ctx.rect(-halfL - goalDepth, -goalHalfW, goalDepth, goalHalfW * 2);

        // 右ゴール
        ctx.rect(halfL, -goalHalfW, goalDepth, goalHalfW * 2);

        ctx.stroke();

        // センタースポット
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
        this.updateModeDisplay();
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
            case 'world_model':
                this.onWorldModel(data);
                break;
            case 'robot_commands':
                this.onRobotCommands(data);
                break;
            case 'control_targets':
                this.onControlTargets(data);
                break;
            case 'robot_feedback':
                this.onRobotFeedback(data);
                break;
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

        // フィールド寸法を world_model から更新
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

        // backward compatibility: /control_targets が来る環境向けのフォールバック
        const cmdSpeed = cmd.polar_velocity_target_mode?.target_velocity_r;
        if (Number.isFinite(cmdSpeed)) this._pushChartPoint('cmd', cmdSpeed);
    }

    onRobotCommands(data) {
        if (!data.commands || this.selectedRobotId === null) return;
        const cmd = data.commands.find(c => c.robot_id === this.selectedRobotId);
        if (!cmd) return;

        // /robot_commands の target_velocity_r を Cmd Speed として表示
        const cmdSpeed = cmd.polar_velocity_target_mode?.target_velocity_r;
        if (Number.isFinite(cmdSpeed)) this._pushChartPoint('cmd', cmdSpeed);
    }

    onRobotFeedback(data) {
        if (!data.robots || this.selectedRobotId === null) return;
        const fb = data.robots.find(r => r.robot_id === this.selectedRobotId);
        if (!fb) return;

        // odom_speed ノルム
        if (fb.odom_speed && fb.odom_speed.length >= 2) {
            const odomSpeed = Math.sqrt(fb.odom_speed[0] ** 2 + fb.odom_speed[1] ** 2);
            document.getElementById('readout-odom-speed').textContent = odomSpeed.toFixed(2) + ' m/s';
            this._pushChartPoint('odom', odomSpeed);
        }

        // mouse_vel ノルム
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

        // 時刻ラベルは est が担当
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
        this.updateModeDisplay();
    }

    deactivateTest() {
        this.wsSend({ type: 'deactivate_robot_test' });
        this.testModeActive = false;
        this.cursorFollowMode = false;
        this.selectedRobotId = null;
        this.targetPos = null;
        this.renderer?.invalidate();
        this.updateModeDisplay();
        document.getElementById('selected-robot-id').textContent = '--';
        document.getElementById('target-pos-display').textContent = '--';
        const canvas = document.getElementById('field-canvas');
        if (canvas) canvas.style.cursor = 'crosshair';
    }

    setCursorFollowMode(active) {
        this.cursorFollowMode = active;
        const canvas = document.getElementById('field-canvas');
        if (canvas) canvas.style.cursor = active ? 'none' : 'crosshair';
        this.updateModeDisplay();
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
        document.getElementById('target-pos-display').textContent =
            `(${this.targetPos.x.toFixed(2)}, ${this.targetPos.y.toFixed(2)})`;
    }

    sendPlannerParam() {
        this.wsSend({
            type: 'set_planner_param',
            velocity_damping_gain: this.velocityDampingGain,
        });
    }

    selectRobot(id) {
        this.selectedRobotId = id;
        document.getElementById('selected-robot-id').textContent = id !== null ? String(id) : '--';
        document.getElementById('readout-est-speed').textContent = '-- m/s';
        document.getElementById('readout-odom-speed').textContent = '-- m/s';
        this.resetChart();
        this.renderer?.invalidate();
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

    updateModeDisplay() {
        const el = document.getElementById('mode-status');
        if (!el) return;
        if (this.cursorFollowMode) {
            el.textContent = 'CURSOR FOLLOW';
            el.classList.add('active');
        } else if (this.testModeActive) {
            el.textContent = 'ROBOT TEST ACTIVE';
            el.classList.add('active');
        } else {
            el.textContent = 'INACTIVE';
            el.classList.remove('active');
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

        // ホイールズーム
        canvas.addEventListener('wheel', (e) => {
            e.preventDefault();
            const factor = e.deltaY < 0 ? 1.1 : 0.9;
            this.zoomLevel = Math.min(Math.max(this.zoomLevel * factor, 0.1), 5.0);
            this.renderer?.invalidate();
        }, { passive: false });

        // シングルクリック: ロボット選択 / 目的地設定 / カーソル追従モード解除
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

        // ダブルクリック: カーソル追従モード開始
        canvas.addEventListener('dblclick', (e) => {
            if (!this.testModeActive || this.selectedRobotId === null) return;
            this.setCursorFollowMode(true);
            const fp = this.renderer.clientToFieldCoords(e.clientX, e.clientY);
            this.targetPos = fp;
            this.sendTarget();
            this.renderer?.invalidate();
        });

        // マウス移動: カーソル追従モード時にリアルタイム送信
        canvas.addEventListener('mousemove', (e) => {
            if (!this.cursorFollowMode) return;
            const fp = this.renderer.clientToFieldCoords(e.clientX, e.clientY);
            this.targetPos = fp;
            this.sendTarget();
            this.renderer?.invalidate();
        });

        // Escキーでカーソル追従モード解除
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
