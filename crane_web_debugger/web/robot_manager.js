// ロボット管理画面 JavaScript
'use strict';

const NUM_ROBOTS = 13;
const WS_PORT = 8091;
const VOLTAGE_MIN = 22.0;
const VOLTAGE_MAX = 25.2;
const TEMP_WARN = 60;
const PING_WARN = 10;
const PING_CRIT = 50;

// ---- State ----------------------------------------------------------------

const robotState = {};
for (let i = 0; i < NUM_ROBOTS; i++) {
    robotState[i] = {
        feedback: null,   // latest robot_feedback entry
        ping_ms: null,
        pi_status: 'Unknown',
        last_feedback_ms: 0,
    };
}

let ws = null;
let wsConnected = false;
let pendingUpdate = false;

// ---- WebSocket ------------------------------------------------------------

function connectWebSocket() {
    const host = window.location.hostname;
    ws = new WebSocket(`ws://${host}:${WS_PORT}`);

    ws.onopen = () => {
        wsConnected = true;
        setWsStatus(true);
        // サーバー側が接続確立時に自動でpollAllPiStatus()を呼ぶため追加送信不要
    };

    ws.onclose = () => {
        wsConnected = false;
        setWsStatus(false);
        setTimeout(connectWebSocket, 3000);
    };

    ws.onerror = () => {
        setWsStatus(false);
    };

    ws.onmessage = (event) => {
        try {
            const msg = JSON.parse(event.data);
            handleMessage(msg);
        } catch (e) {
            // ignore parse errors
        }
    };
}

function setWsStatus(connected) {
    const dot = document.getElementById('ws-dot');
    const text = document.getElementById('ws-status-text');
    if (connected) {
        dot.className = 'status-dot connected';
        text.textContent = '接続済み';
    } else {
        dot.className = 'status-dot disconnected';
        text.textContent = '未接続';
    }
}

// ---- Message Handlers -----------------------------------------------------

function handleMessage(msg) {
    switch (msg.type) {
        case 'robot_feedback':
            onRobotFeedback(msg);
            break;
        case 'ping_status':
            onPingStatus(msg);
            break;
        case 'pi_status':
            onPiStatus(msg);
            break;
        case 'robot_control_result':
            onRobotControlResult(msg);
            break;
        // diagnostics は将来の拡張用
        default:
            break;
    }
}

function onRobotFeedback(msg) {
    const now = Date.now();
    for (const robot of msg.robots) {
        const id = robot.robot_id;
        if (id >= 0 && id < NUM_ROBOTS) {
            robotState[id].feedback = robot;
            robotState[id].last_feedback_ms = now;
        }
    }
    pendingUpdate = true;
}

function onPingStatus(msg) {
    for (const entry of msg.robots) {
        const id = entry.robot_id;
        if (id >= 0 && id < NUM_ROBOTS) {
            robotState[id].ping_ms = entry.ping_ms;
        }
    }
    pendingUpdate = true;
}

function onPiStatus(msg) {
    for (const entry of msg.robots) {
        const id = entry.robot_id;
        if (id >= 0 && id < NUM_ROBOTS) {
            robotState[id].pi_status = entry.status;
        }
    }
    pendingUpdate = true;
}

function onRobotControlResult(msg) {
    const id = msg.robot_id;
    const success = msg.success;
    const status = msg.status || '';
    const command = msg.command || '';

    if (id >= 0 && id < NUM_ROBOTS && success) {
        robotState[id].pi_status = status;
        pendingUpdate = true;
    }

    const icon = success ? '✅' : '❌';
    console.log(`${icon} Robot#${id} ${command}: ${status}`);
}

// ---- Error String Conversion ----------------------------------------------

function getErrorString(error_id, error_info) {
    if (!error_info) return 'OK';

    if (error_id === 100) {
        // POWERエラー
        const bits = [
            [0x0001, '低電圧'],
            [0x0002, '過電圧'],
            [0x0004, '過電流'],
            [0x0008, '短絡'],
            [0x0010, '充電タイムアウト'],
            [0x0020, '充電電力異常'],
            [0x0040, '放電異常'],
            [0x0080, 'パラメータ異常'],
            [0x0100, 'コマンド異常'],
            [0x0200, 'キャパシタ未接続'],
            [0x0400, '放電失敗'],
            [0x0800, 'GD電源異常'],
            [0x1000, 'コイル過熱'],
            [0x2000, 'FET過熱'],
        ];
        const errors = bits.filter(([bit]) => error_info & bit).map(([, name]) => name);
        return 'POWER: ' + (errors.length ? errors.join(', ') : '不明');
    }

    if (error_id <= 3) {
        // BLDCモーターエラー
        const motorNames = ['RF', 'RB', 'LB', 'LF'];
        const motorName = motorNames[error_id] ?? `M${error_id}`;
        const bits = [
            [0x0001, '低電圧'],
            [0x0002, '過電流'],
            [0x0004, 'モーター過熱'],
            [0x0008, '過負荷'],
            [0x0010, 'エンコーダエラー'],
            [0x0020, '過電圧'],
            [0x0040, 'FET過熱'],
        ];
        const errors = bits.filter(([bit]) => error_info & bit).map(([, name]) => name);
        return `BLDC[${motorName}]: ` + (errors.length ? errors.join(', ') : '不明');
    }

    return `ERR(id=${error_id}, info=0x${error_info.toString(16)})`;
}

// ---- Rendering ------------------------------------------------------------

function getOverallStatus(id) {
    const state = robotState[id];
    const fb = state.feedback;
    const pi = state.pi_status;

    if (fb && fb.error_info) return 'Error';
    if (pi === 'Running') return 'Running';
    if (pi === 'Stopped') return 'Stopped';
    if (pi === 'Offline' || pi === 'Unknown') return 'Offline';
    return pi;
}

const STATUS_CONFIG = {
    'Running': { badge: 'bg-success',          label: '稼働中' },
    'Stopped': { badge: 'bg-warning text-dark', label: '停止中' },
    'Error':   { badge: 'bg-danger',            label: 'エラー' },
    'Offline': { badge: 'bg-secondary',         label: 'オフライン' },
};

function statusBadgeClass(status) {
    return STATUS_CONFIG[status]?.badge ?? 'bg-secondary';
}

function statusLabel(status) {
    return STATUS_CONFIG[status]?.label ?? (status || '不明');
}

function voltageBarClass(v) {
    if (v < 22.0) return 'bg-danger';
    if (v < 23.0) return 'bg-warning';
    return 'bg-success';
}

function pingClass(ms) {
    if (ms === null) return 'text-secondary';
    if (ms > PING_CRIT) return 'text-danger';
    if (ms > PING_WARN) return 'text-warning';
    return 'text-success';
}

function renderRobotCard(id) {
    const state = robotState[id];
    const fb = state.feedback;
    const overallStatus = getOverallStatus(id);

    const card = document.getElementById(`robot-card-${id}`);
    if (!card) return;

    // Card border style
    card.className = 'card robot-card h-100';
    if (overallStatus === 'Error') card.classList.add('has-error');
    else if (overallStatus === 'Offline') card.classList.add('offline');

    // Status badge
    const badge = document.getElementById(`badge-${id}`);
    badge.className = `badge ${statusBadgeClass(overallStatus)}`;
    badge.textContent = statusLabel(overallStatus);

    // Voltage
    const voltageEl = document.getElementById(`voltage-${id}`);
    const voltageBarEl = document.getElementById(`voltage-bar-${id}`);
    if (fb && fb.voltage && fb.voltage.length > 0) {
        const v = fb.voltage[0];
        const pct = Math.max(0, Math.min(100, (v - VOLTAGE_MIN) / (VOLTAGE_MAX - VOLTAGE_MIN) * 100));
        voltageEl.textContent = `${v.toFixed(2)} V`;
        voltageEl.className = `info-value ${v < 22.0 ? 'text-danger' : v < 23.0 ? 'text-warning' : 'text-success'}`;
        voltageBarEl.style.width = `${pct}%`;
        voltageBarEl.className = `progress-bar ${voltageBarClass(v)}`;
    } else {
        voltageEl.textContent = '--';
        voltageEl.className = 'info-value text-secondary';
        voltageBarEl.style.width = '0%';
        voltageBarEl.className = 'progress-bar bg-secondary';
    }

    // Temperature
    const tempEl = document.getElementById(`temp-${id}`);
    if (fb && fb.temperatures && fb.temperatures.length > 0) {
        const maxTemp = Math.max(...fb.temperatures);
        tempEl.textContent = `${maxTemp} °C`;
        tempEl.className = `info-value ${maxTemp >= TEMP_WARN ? 'text-danger' : ''}`;
    } else {
        tempEl.textContent = '--';
        tempEl.className = 'info-value text-secondary';
    }

    // Ping
    const pingEl = document.getElementById(`ping-${id}`);
    if (state.ping_ms !== null) {
        pingEl.textContent = `${state.ping_ms.toFixed(1)} ms`;
        pingEl.className = `info-value ${pingClass(state.ping_ms)}`;
    } else {
        pingEl.textContent = '--';
        pingEl.className = 'info-value text-secondary';
    }

    // Packet frequency
    const freqEl = document.getElementById(`freq-${id}`);
    if (fb) {
        freqEl.textContent = `${fb.packet_frequency_hz.toFixed(1)} Hz`;
        freqEl.className = `info-value ${fb.packet_frequency_hz < 50 ? 'text-warning' : 'text-success'}`;
    } else {
        freqEl.textContent = '--';
        freqEl.className = 'info-value text-secondary';
    }

    // Error
    const errorEl = document.getElementById(`error-${id}`);
    if (fb && fb.error_info) {
        errorEl.textContent = getErrorString(fb.error_id, fb.error_info);
        errorEl.className = 'error-text text-danger';
    } else {
        errorEl.textContent = 'OK';
        errorEl.className = 'error-text text-success';
    }
}

function renderSummary() {
    let activeCount = 0;
    let errorCount = 0;
    let feedbackCount = 0;
    const staleThreshold = 2000; // 2秒以上古いフィードバックは無視
    const now = Date.now();

    for (let i = 0; i < NUM_ROBOTS; i++) {
        const state = robotState[i];
        const status = getOverallStatus(i);
        if (status === 'Running') activeCount++;
        if (status === 'Error') errorCount++;
        if (state.feedback && (now - state.last_feedback_ms) < staleThreshold) feedbackCount++;
    }

    document.getElementById('active-count').textContent = activeCount;
    document.getElementById('error-count').textContent = errorCount;
    document.getElementById('feedback-count').textContent = feedbackCount;
}

function renderAll() {
    for (let i = 0; i < NUM_ROBOTS; i++) {
        renderRobotCard(i);
    }
    renderSummary();
}

// ---- Card Generation ------------------------------------------------------

function createRobotCard(id) {
    return `
<div class="col-xl-3 col-lg-4 col-md-6">
    <div class="card robot-card h-100" id="robot-card-${id}">
        <div class="card-header d-flex justify-content-between align-items-center">
            <strong><i class="fas fa-robot me-1"></i>Robot #${id}</strong>
            <span class="badge bg-secondary" id="badge-${id}">不明</span>
        </div>
        <div class="card-body">
            <!-- 電圧 -->
            <div class="info-row">
                <span class="info-label"><i class="fas fa-battery-half me-1"></i>電圧</span>
                <span class="info-value text-secondary" id="voltage-${id}">--</span>
            </div>
            <div class="progress voltage-bar mb-2">
                <div class="progress-bar bg-secondary" id="voltage-bar-${id}" style="width:0%"></div>
            </div>
            <!-- 温度 -->
            <div class="info-row">
                <span class="info-label"><i class="fas fa-thermometer-half me-1"></i>最高温度</span>
                <span class="info-value text-secondary" id="temp-${id}">--</span>
            </div>
            <!-- Ping -->
            <div class="info-row">
                <span class="info-label"><i class="fas fa-wifi me-1"></i>Ping</span>
                <span class="info-value text-secondary" id="ping-${id}">--</span>
            </div>
            <!-- 通信頻度 -->
            <div class="info-row">
                <span class="info-label"><i class="fas fa-signal me-1"></i>通信</span>
                <span class="info-value text-secondary" id="freq-${id}">--</span>
            </div>
            <!-- エラー -->
            <div class="info-row align-items-start">
                <span class="info-label"><i class="fas fa-exclamation-circle me-1"></i>エラー</span>
                <span class="error-text text-success" id="error-${id}">OK</span>
            </div>
        </div>
        <div class="card-footer p-2">
            <div class="btn-group w-100" role="group">
                <button class="btn btn-sm btn-outline-success"
                        onclick="sendRobotControl(${id}, 'start')">
                    <i class="fas fa-play me-1"></i>Start
                </button>
                <button class="btn btn-sm btn-outline-danger"
                        onclick="sendRobotControl(${id}, 'stop')">
                    <i class="fas fa-stop me-1"></i>Stop
                </button>
            </div>
        </div>
    </div>
</div>`;
}

function initGrid() {
    const grid = document.getElementById('robot-grid');
    let html = '';
    for (let i = 0; i < NUM_ROBOTS; i++) {
        html += createRobotCard(i);
    }
    grid.innerHTML = html;
}

// ---- Control Commands -----------------------------------------------------

function sendRobotControl(robotId, command) {
    if (!ws || ws.readyState !== WebSocket.OPEN) {
        alert('WebSocketに接続されていません');
        return;
    }
    ws.send(JSON.stringify({ type: 'robot_control', robot_id: robotId, command: command }));
}

function startAll() {
    for (let i = 0; i < NUM_ROBOTS; i++) {
        sendRobotControl(i, 'start');
    }
}

function stopAll() {
    if (!confirm('全ロボットを停止しますか？')) return;
    for (let i = 0; i < NUM_ROBOTS; i++) {
        sendRobotControl(i, 'stop');
    }
}

function refreshPiStatus() {
    if (!ws || ws.readyState !== WebSocket.OPEN) {
        alert('WebSocketに接続されていません');
        return;
    }
    ws.send(JSON.stringify({ type: 'poll_pi_status' }));
}

// ---- Main -----------------------------------------------------------------

initGrid();
renderAll();
connectWebSocket();

// 500ms ごとに DOM 更新
setInterval(() => {
    if (pendingUpdate) {
        renderAll();
        pendingUpdate = false;
    }
}, 500);
