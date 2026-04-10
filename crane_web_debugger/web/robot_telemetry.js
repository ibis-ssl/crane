const CHART_COLORS = {
    estX:     '#A0C4FF',  // primary
    estY:     '#5A9BD5',
    visX:     '#66BB6A',  // success green
    visY:     '#2E7D32',
    tgtX:     '#D0BCFF',  // tertiary purple
    tgtY:     '#9575CD',
    fbVx:     '#A0C4FF',
    fbVy:     '#5A9BD5',
    odomVx:   '#66BB6A',
    odomVy:   '#2E7D32',
    mouseVx:  '#D0BCFF',
    mouseVy:  '#9575CD',
    tgtVx:    '#FFA726',  // warning
    tgtVy:    '#EF6C00',
    estTheta: '#A0C4FF',
    visTheta: '#66BB6A',
    tgtTheta: '#D0BCFF',
    gyroYaw:  '#FFA726',
    velErr:   '#FFB4AB',  // error
    posErr:   '#FFA726',
};

const CONTROL_MODE_NAMES = {
    0: 'LOCAL_CAMERA',
    1: 'POSITION_TARGET',
    2: 'SIMPLE_VELOCITY',
    3: 'POLAR_VELOCITY'
};

const CORRECTION_SOURCE_COLORS = {
    'rvo2': '#FFB4AB',
    'sender_accel_limit': '#FFA726',
    'feedback_control': '#66BB6A',
};

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

function makeChart(canvasId, datasets, yLabel = '') {
    const canvas = document.getElementById(canvasId);
    if (!canvas) return null;
    return new Chart(canvas, {
        type: 'line',
        data: { labels: [], datasets },
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
                    title: { display: !!yLabel, text: yLabel, color: '#8C929A', font: { size: 10 } },
                    ticks: { color: '#8C929A', maxTicksLimit: 6, font: { size: 10 } },
                    grid: { color: '#2D3748' },
                }
            },
            plugins: {
                legend: {
                    labels: {
                        color: '#8C929A',
                        font: { size: 10 },
                        boxWidth: 12,
                        padding: 6,
                    }
                },
                tooltip: {
                    mode: 'index',
                    intersect: false,
                    backgroundColor: '#213052',
                    titleColor: '#A0C4FF',
                    bodyColor: '#E2E4E8',
                    titleFont: { size: 10 },
                    bodyFont: { size: 10 },
                }
            }
        }
    });
}

class RobotTelemetry {
    constructor() {
        this.websocket = null;
        this.robotId = this.getRobotIdFromUrl();
        this.bufferSize = 300; // 300 points at 10Hz = 30s
        this.startTime = null;
        this.knownRobotIds = new Set();

        this.charts = {};

        this.initCharts();
        this.setupControls();
        this.setupWebSocket();
        this.updateRobotIdDisplay();
    }

    getRobotIdFromUrl() {
        const params = new URLSearchParams(window.location.search);
        return parseInt(params.get('id') ?? '0', 10);
    }

    initCharts() {
        this.charts.position = makeChart('chart-position', [
            makeDataset('Est. X', CHART_COLORS.estX),
            makeDataset('Est. Y', CHART_COLORS.estY),
            makeDataset('Target X', CHART_COLORS.tgtX, true),
            makeDataset('Target Y', CHART_COLORS.tgtY, true),
            makeDataset('Vision X', CHART_COLORS.visX, true),
            makeDataset('Vision Y', CHART_COLORS.visY, true),
        ], 'm');

        this.charts.velocity = makeChart('chart-velocity', [
            makeDataset('Est. Vx', CHART_COLORS.fbVx),
            makeDataset('Est. Vy', CHART_COLORS.fbVy),
            makeDataset('Target Vx', CHART_COLORS.tgtVx, true),
            makeDataset('Target Vy', CHART_COLORS.tgtVy, true),
            makeDataset('Odom Vx', CHART_COLORS.odomVx, true),
            makeDataset('Mouse Vx', CHART_COLORS.mouseVx, true),
        ], 'm/s');

        this.charts.theta = makeChart('chart-theta', [
            makeDataset('Est. θ', CHART_COLORS.estTheta),
            makeDataset('Target θ', CHART_COLORS.tgtTheta, true),
            makeDataset('Vision θ', CHART_COLORS.visTheta, true),
            makeDataset('Gyro Yaw', CHART_COLORS.gyroYaw, true),
        ], 'rad');

        this.charts.error = makeChart('chart-error', [
            makeDataset('Vel Error', CHART_COLORS.velErr),
            makeDataset('Pos Error', CHART_COLORS.posErr),
        ], 'm / m/s');
    }

    setupControls() {
        const slider = document.getElementById('buffer-size-slider');
        const label = document.getElementById('buffer-size-label');
        if (slider) {
            slider.addEventListener('input', () => {
                this.bufferSize = parseInt(slider.value, 10);
                if (label) label.textContent = `${this.bufferSize} pts (${Math.round(this.bufferSize / 10)}s)`;
                this.trimBuffers();
            });
        }

        const selector = document.getElementById('robot-selector');
        if (selector) {
            selector.value = this.robotId;
            selector.addEventListener('change', () => {
                this.robotId = parseInt(selector.value, 10);
                this.updateUrlParam();
                this.resetCharts();
                this.updateRobotIdDisplay();
            });
        }
    }

    updateUrlParam() {
        const url = new URL(window.location.href);
        url.searchParams.set('id', this.robotId);
        window.history.replaceState({}, '', url.toString());
    }

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
            try {
                this.handleMessage(JSON.parse(event.data));
            } catch (e) {
                console.error('解析エラー:', e);
            }
        };
    }

    setStatus(connected) {
        const dot = document.getElementById('status-dot');
        const label = document.getElementById('status-label');
        const banner = document.getElementById('offline-banner');
        if (dot) dot.classList.toggle('connected', connected);
        if (label) label.textContent = connected ? 'connected' : 'disconnected';
        if (banner) banner.classList.toggle('visible', !connected);
    }

    handleMessage(data) {
        switch (data.type) {
            case 'world_model':
                this.discoverRobotIds(data);
                this.pushWorldModelData(data);
                break;
            case 'control_targets':
                this.pushControlTargetsData(data);
                break;
            case 'robot_feedback':
                this.pushRobotFeedbackData(data);
                break;
        }
    }

    discoverRobotIds(data) {
        if (!data.robots_ours) return;
        let changed = false;
        for (const robot of data.robots_ours) {
            if (!this.knownRobotIds.has(robot.id)) {
                this.knownRobotIds.add(robot.id);
                changed = true;
            }
        }
        if (changed) this.updateRobotSelector();
    }

    updateRobotSelector() {
        const selector = document.getElementById('robot-selector');
        if (!selector) return;
        const ids = [...this.knownRobotIds].sort((a, b) => a - b);
        selector.innerHTML = '';
        for (const id of ids) {
            const opt = document.createElement('option');
            opt.value = id;
            opt.textContent = `Robot ${id}`;
            if (id === this.robotId) opt.selected = true;
            selector.appendChild(opt);
        }
        if (!this.knownRobotIds.has(this.robotId) && ids.length > 0) {
            this.robotId = ids[0];
            this.updateRobotIdDisplay();
        }
    }

    updateRobotIdDisplay() {
        const el = document.getElementById('current-robot-id');
        if (el) el.textContent = this.robotId;
    }

    now() {
        if (!this.startTime) this.startTime = Date.now();
        return (Date.now() - this.startTime) / 1000;
    }

    pushToChart(chart, datasetIndex, x, y) {
        const ds = chart.data.datasets[datasetIndex];
        if (!ds) return;
        ds.data.push({ x, y });
        if (ds.data.length > this.bufferSize) ds.data.shift();
    }

    pushWorldModelData(data) {
        if (!data.robots_ours) return;
        const robot = data.robots_ours.find(r => r.id === this.robotId);
        if (!robot) return;

        const t = this.now();

        // Position chart
        this.pushToChart(this.charts.position, 0, t, robot.x ?? null);
        this.pushToChart(this.charts.position, 1, t, robot.y ?? null);
        this.pushToChart(this.charts.position, 4, t, robot.vision_x ?? null);
        this.pushToChart(this.charts.position, 5, t, robot.vision_y ?? null);
        this.charts.position.update('none');

        // Velocity chart
        this.pushToChart(this.charts.velocity, 0, t, robot.vx ?? null);
        this.pushToChart(this.charts.velocity, 1, t, robot.vy ?? null);
        this.charts.velocity.update('none');

        // Theta chart
        this.pushToChart(this.charts.theta, 0, t, robot.theta ?? null);
        this.pushToChart(this.charts.theta, 2, t, robot.vision_theta ?? null);
        this.charts.theta.update('none');

        // Update table
        const estPos = document.getElementById('tbl-est-pos');
        const visPos = document.getElementById('tbl-vis-pos');
        const estVel = document.getElementById('tbl-est-vel');
        if (estPos) estPos.textContent = `(${robot.x?.toFixed(3)}, ${robot.y?.toFixed(3)}, ${robot.theta?.toFixed(3)})`;
        if (visPos) visPos.textContent = `(${robot.vision_x?.toFixed(3) ?? '--'}, ${robot.vision_y?.toFixed(3) ?? '--'}, ${robot.vision_theta?.toFixed(3) ?? '--'})`;
        if (estVel) estVel.textContent = `(${robot.vx?.toFixed(3)}, ${robot.vy?.toFixed(3)})`;

        // Header
        document.getElementById('header-pos').textContent = `Pos: (${robot.x?.toFixed(2)}, ${robot.y?.toFixed(2)})`;
        document.getElementById('header-vel').textContent = `Vel: (${robot.vx?.toFixed(2)}, ${robot.vy?.toFixed(2)})`;
    }

    pushControlTargetsData(data) {
        if (!data.commands) return;
        const cmd = data.commands.find(c => c.robot_id === this.robotId);
        if (!cmd) return;

        const t = this.now();

        // Position target
        if (cmd.position_target_mode) {
            const pt = cmd.position_target_mode;
            this.pushToChart(this.charts.position, 2, t, pt.target_x ?? null);
            this.pushToChart(this.charts.position, 3, t, pt.target_y ?? null);
            this.charts.position.update('none');

            const tgtPos = document.getElementById('tbl-tgt-pos');
            if (tgtPos) tgtPos.textContent = `(${pt.target_x?.toFixed(3)}, ${pt.target_y?.toFixed(3)})`;
        }

        // Velocity target
        if (cmd.simple_velocity_target_mode) {
            const sv = cmd.simple_velocity_target_mode;
            this.pushToChart(this.charts.velocity, 2, t, sv.target_vx ?? null);
            this.pushToChart(this.charts.velocity, 3, t, sv.target_vy ?? null);
            this.charts.velocity.update('none');

            const tgtVel = document.getElementById('tbl-tgt-vel');
            if (tgtVel) tgtVel.textContent = `(${sv.target_vx?.toFixed(3)}, ${sv.target_vy?.toFixed(3)})`;
        }

        // Target theta
        this.pushToChart(this.charts.theta, 1, t, cmd.target_theta ?? null);
        this.charts.theta.update('none');

        // Prediction errors from velocity_plan_trace.actuals
        if (cmd.velocity_plan_trace?.actuals?.length > 0) {
            const latest = cmd.velocity_plan_trace.actuals.at(-1);
            if (latest) {
                this.pushToChart(this.charts.error, 0, t, latest.velocity_error ?? null);
                this.pushToChart(this.charts.error, 1, t, latest.position_error ?? null);
                this.charts.error.update('none');
            }
        }

        // Velocity corrections bar chart
        if (cmd.velocity_plan_trace?.corrections) {
            this.renderCorrectionBars(cmd.velocity_plan_trace.corrections);
        }

        // Header info
        const modeName = CONTROL_MODE_NAMES[cmd.control_mode] ?? `MODE_${cmd.control_mode}`;
        document.getElementById('header-planner').textContent = `Planner: ${cmd.planner_name || '--'}`;
        document.getElementById('header-mode').textContent = `Mode: ${modeName}`;
    }

    pushRobotFeedbackData(data) {
        if (!data.robots) return;
        const robot = data.robots.find(r => r.robot_id === this.robotId);
        if (!robot) return;

        const t = this.now();

        // odom speed (index 0 = vx, 1 = vy in most configurations)
        if (robot.odom_speed && robot.odom_speed.length >= 2) {
            this.pushToChart(this.charts.velocity, 4, t, robot.odom_speed[0]);
        }

        // mouse vel (index 0 = vx)
        if (robot.mouse_vel && robot.mouse_vel.length >= 1) {
            this.pushToChart(this.charts.velocity, 5, t, robot.mouse_vel[0]);
        }

        // gyro yaw
        this.pushToChart(this.charts.theta, 3, t, robot.yaw_angle ?? null);
        this.charts.theta.update('none');
        this.charts.velocity.update('none');
    }

    renderCorrectionBars(corrections) {
        const container = document.getElementById('correction-bars');
        if (!container) return;
        if (!corrections || corrections.length === 0) {
            container.innerHTML = '<div style="color:#666; font-size:0.75rem;">修正データなし</div>';
            return;
        }

        const maxDelta = Math.max(...corrections.map(c => c.velocity_delta || 0), 0.01);
        container.innerHTML = '';

        for (const corr of corrections) {
            const delta = corr.velocity_delta ?? 0;
            const color = CORRECTION_SOURCE_COLORS[corr.source] ?? '#7ec8e3';
            const pct = Math.min(100, (delta / maxDelta) * 100);

            const row = document.createElement('div');
            row.className = 'correction-row';
            row.innerHTML = `
                <span class="correction-source" title="${corr.source}">${corr.source}</span>
                <div class="correction-bar-bg">
                    <div class="correction-bar-fill" style="width:${pct}%; background:${color};"></div>
                </div>
                <span class="correction-value">${delta.toFixed(3)} m/s</span>
                <span style="font-size:0.7rem; color:#888; width:40px;">${(corr.direction_delta_deg ?? 0).toFixed(1)}°</span>
            `;
            container.appendChild(row);
        }
    }

    resetCharts() {
        this.startTime = null;
        for (const chart of Object.values(this.charts)) {
            chart?.destroy();
        }
        this.charts = {};
        this.initCharts();
        document.getElementById('correction-bars').innerHTML = '<div style="color:#666; font-size:0.75rem;">データなし</div>';
        const ids = ['tbl-est-pos', 'tbl-vis-pos', 'tbl-tgt-pos', 'tbl-est-vel', 'tbl-tgt-vel'];
        for (const id of ids) {
            const el = document.getElementById(id);
            if (el) el.textContent = '--';
        }
    }

    trimBuffers() {
        for (const chart of Object.values(this.charts)) {
            if (!chart) continue;
            for (const ds of chart.data.datasets) {
                while (ds.data.length > this.bufferSize) ds.data.shift();
            }
            chart.update('none');
        }
    }


}

document.addEventListener('DOMContentLoaded', () => {
    window.robotTelemetry = new RobotTelemetry();
});
