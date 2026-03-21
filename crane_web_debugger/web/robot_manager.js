// ロボット管理画面 JavaScript
'use strict';

const NUM_ROBOTS = 13;
const WS_PORT = 8091;
const VOLTAGE_MIN = 22.0;
const VOLTAGE_MAX = 25.2;
const TEMP_WARN = 60;
const PING_WARN = 10;
const PING_CRIT = 50;

const CONTROL_MODE_LONG = { 0: 'LOCAL_CAMERA', 1: 'POSITION_TARGET', 2: 'SIMPLE_VELOCITY', 3: 'POLAR_VELOCITY' };

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

// world_model / control_targets per robot
const robotPose = {};     // { x, y, theta, vx, vy, omega, vision_x, vision_y, vision_theta }
const robotCommand = {};  // control_targets commands keyed by robot_id

let detailOpenId = null;

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
        case 'robot_feedback':      onRobotFeedback(msg); break;
        case 'ping_status':         onPingStatus(msg); break;
        case 'pi_status':           onPiStatus(msg); break;
        case 'robot_control_result': onRobotControlResult(msg); break;
        case 'world_model':         onWorldModel(msg); break;
        case 'control_targets':     onControlTargets(msg); break;
        case 'robot_commands':      onRobotCommands(msg); break;
        default: break;
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

function onWorldModel(msg) {
    if (msg.robots_ours) {
        for (const r of msg.robots_ours) {
            robotPose[r.id] = r;
        }
    }
    if (detailOpenId !== null) refreshDetailPanel(detailOpenId);
}

function onControlTargets(msg) {
    if (msg.commands) {
        for (const cmd of msg.commands) {
            robotCommand[cmd.robot_id] = cmd;
        }
    }
    if (detailOpenId !== null) refreshDetailPanel(detailOpenId);
}

function onRobotCommands(msg) {
    // control_targets がない場合のフォールバック
    if (msg.commands && Object.keys(robotCommand).length === 0) {
        for (const cmd of msg.commands) {
            robotCommand[cmd.robot_id] = cmd;
        }
        if (detailOpenId !== null) refreshDetailPanel(detailOpenId);
    }
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

function renderRobotRow(id) {
    const state = robotState[id];
    const fb = state.feedback;
    const overallStatus = getOverallStatus(id);

    const row = document.getElementById(`robot-row-${id}`);
    if (!row) return;

    // Row style
    row.className = 'robot-row';
    if (overallStatus === 'Error') row.classList.add('has-error');
    else if (overallStatus === 'Offline') row.classList.add('offline');

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
        voltageEl.className = `fw-medium ${v < 22.0 ? 'text-danger' : v < 23.0 ? 'text-warning' : 'text-success'}`;
        voltageBarEl.style.width = `${pct}%`;
        voltageBarEl.className = `progress-bar ${voltageBarClass(v)}`;
    } else {
        voltageEl.textContent = '--';
        voltageEl.className = 'fw-medium text-secondary';
        voltageBarEl.style.width = '0%';
        voltageBarEl.className = 'progress-bar bg-secondary';
    }

    // Temperature
    const tempEl = document.getElementById(`temp-${id}`);
    if (fb && fb.temperatures && fb.temperatures.length > 0) {
        const maxTemp = Math.max(...fb.temperatures);
        tempEl.textContent = `${maxTemp} °C`;
        tempEl.className = `fw-medium ${maxTemp >= TEMP_WARN ? 'text-danger' : ''}`;
    } else {
        tempEl.textContent = '--';
        tempEl.className = 'fw-medium text-secondary';
    }

    // Ping
    const pingEl = document.getElementById(`ping-${id}`);
    if (state.ping_ms !== null) {
        pingEl.textContent = `${state.ping_ms.toFixed(1)} ms`;
        pingEl.className = `fw-medium ${pingClass(state.ping_ms)}`;
    } else {
        pingEl.textContent = '--';
        pingEl.className = 'fw-medium text-secondary';
    }

    // Packet frequency
    const freqEl = document.getElementById(`freq-${id}`);
    if (fb) {
        freqEl.textContent = `${fb.packet_frequency_hz.toFixed(1)} Hz`;
        freqEl.className = `fw-medium ${fb.packet_frequency_hz < 50 ? 'text-warning' : 'text-success'}`;
    } else {
        freqEl.textContent = '--';
        freqEl.className = 'fw-medium text-secondary';
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
        renderRobotRow(i);
    }
    renderSummary();
}

// ---- Table Row Generation -------------------------------------------------

function createRobotRow(id) {
    return `
<tr class="robot-row" id="robot-row-${id}">
    <td><i class="fas fa-robot me-1 text-secondary"></i><strong>#${id}</strong></td>
    <td>
        <div class="d-flex align-items-center gap-2">
            <span class="badge bg-secondary" id="badge-${id}" style="min-width:60px">不明</span>
            <div class="btn-group btn-group-sm" role="group">
                <button class="btn btn-outline-success" onclick="sendRobotControl(${id}, 'start')" title="Start">
                    <i class="fas fa-play"></i>
                </button>
                <button class="btn btn-outline-danger" onclick="sendRobotControl(${id}, 'stop')" title="Stop">
                    <i class="fas fa-stop"></i>
                </button>
            </div>
        </div>
    </td>
    <td>
        <span class="fw-medium text-secondary" id="voltage-${id}">--</span>
        <div class="progress voltage-bar">
            <div class="progress-bar bg-secondary" id="voltage-bar-${id}" style="width:0%"></div>
        </div>
    </td>
    <td><span class="fw-medium text-secondary" id="temp-${id}">--</span></td>
    <td><span class="fw-medium text-secondary" id="ping-${id}">--</span></td>
    <td><span class="fw-medium text-secondary" id="freq-${id}">--</span></td>
    <td><span class="error-text text-success" id="error-${id}">OK</span></td>
    <td>
        <button class="btn btn-sm btn-outline-info" onclick="openDetailPanel(${id})" title="詳細">
            <i class="fas fa-info-circle"></i>
        </button>
    </td>
</tr>`;
}

function initTable() {
    const tbody = document.getElementById('robot-table-body');
    let html = '';
    for (let i = 0; i < NUM_ROBOTS; i++) {
        html += createRobotRow(i);
    }
    tbody.innerHTML = html;
}

// ---- Detail Side Panel ----------------------------------------------------

function openDetailPanel(id) {
    detailOpenId = id;
    refreshDetailPanel(id);
    document.getElementById('detail-panel').classList.add('open');
    document.getElementById('panel-backdrop').style.display = 'block';
}

function closeDetailPanel() {
    detailOpenId = null;
    document.getElementById('detail-panel').classList.remove('open');
    document.getElementById('panel-backdrop').style.display = 'none';
}

function row(label, value) {
    return `<tr><td>${label}</td><td>${value}</td></tr>`;
}

function refreshDetailPanel(id) {
    const pose = robotPose[id];
    const cmd = robotCommand[id];
    const state = robotState[id];

    document.getElementById('panel-title').textContent = `Robot #${id}`;
    document.getElementById('panel-telemetry-link').href = `/robot_telemetry.html?id=${id}`;

    // --- 位置・速度 ---
    let poseHtml = '';
    if (pose) {
        const fp = (v, d) => { const n = parseFloat(v); return isNaN(n) ? '--' : n.toFixed(d); };
        const deg = (v) => { const n = parseFloat(v); return isNaN(n) ? '--' : (n * 180 / Math.PI).toFixed(1); };
        const cell = (label, val) =>
            `<td style="padding:1px 6px 1px 0;font-size:0.78rem"><span style="color:#6c757d">${label}</span> ${val}</td>`;
        poseHtml = `
<table style="width:100%;border-collapse:collapse">
  <tr>
    ${cell('X', fp(pose.x, 3))}
    ${cell('Y', fp(pose.y, 3))}
    ${cell('θ', deg(pose.theta) + '°')}
  </tr>
  <tr>
    ${cell('Vx', fp(pose.vx, 3))}
    ${cell('Vy', fp(pose.vy, 3))}
    ${cell('ω', deg(pose.omega) + '°/s')}
  </tr>`;
        if (pose.available_vision != null) {
            const flag = (ok, label) =>
                ok ? `<span class="text-success">${label}</span>` : `<span class="text-secondary">${label}</span>`;
            poseHtml += `<tr><td colspan="3" style="padding-top:4px;font-size:0.78rem">
                ${flag(pose.available_vision, 'Vision')}
                ${flag(pose.available_feedback, 'FB')}
                ${flag(pose.available_tracker, 'Tracker')}
            </td></tr>`;
        }
        poseHtml += '</table>';
    } else {
        poseHtml = '<span class="text-secondary" style="font-size:0.82rem">データなし</span>';
    }
    document.getElementById('panel-pose').innerHTML = poseHtml;

    // --- Session / スキル ---
    let sessionHtml = '';
    if (cmd) {
        const fc = (v, d) => { const n = parseFloat(v); return isNaN(n) ? '--' : n.toFixed(d); };
        const modeName = CONTROL_MODE_LONG[cmd.control_mode] ?? `MODE_${cmd.control_mode}`;
        sessionHtml += row('プランナー', cmd.planner_name || '--');
        sessionHtml += row('制御モード', modeName);
        const kick = parseFloat(cmd.kick_power);
        sessionHtml += row('キック', kick > 0 ? `<span class="text-warning">${fc(cmd.kick_power, 2)}</span>` : '0');
        const drib = parseFloat(cmd.dribble_power);
        sessionHtml += row('ドリブル', drib > 0 ? `<span class="text-info">${fc(cmd.dribble_power, 2)}</span>` : '0');
        sessionHtml += row('チップ', cmd.chip_enable ? '<span class="text-warning">ON</span>' : 'OFF');
    } else {
        sessionHtml = row('--', '<span class="text-secondary">データなし</span>');
    }
    document.getElementById('panel-session').innerHTML = sessionHtml;

    // --- 目標値 ---
    let targetHtml = '';
    if (cmd) {
        const ft = (v, d) => { const n = parseFloat(v); return isNaN(n) ? '--' : n.toFixed(d); };
        const fdeg = (v) => { const n = parseFloat(v); return isNaN(n) ? '--' : (n * 180 / Math.PI).toFixed(1) + '°'; };
        const tcell = (label, val) =>
            `<td style="padding:1px 6px 1px 0;font-size:0.78rem"><span style="color:#6c757d">${label}</span> ${val}</td>`;
        if (cmd.position_target_mode) {
            const t = cmd.position_target_mode;
            targetHtml = `<table style="width:100%;border-collapse:collapse"><tr>
                ${tcell('X', ft(t.target_x, 3))}
                ${tcell('Y', ft(t.target_y, 3))}
                ${tcell('θ', fdeg(cmd.target_theta))}
            </tr></table>`;
        } else if (cmd.simple_velocity_target_mode) {
            const t = cmd.simple_velocity_target_mode;
            targetHtml = `<table style="width:100%;border-collapse:collapse"><tr>
                ${tcell('Vx', ft(t.target_vx, 3))}
                ${tcell('Vy', ft(t.target_vy, 3))}
                ${tcell('θ', fdeg(cmd.target_theta))}
            </tr></table>`;
        } else if (cmd.polar_velocity_target_mode) {
            const t = cmd.polar_velocity_target_mode;
            targetHtml = `<table style="width:100%;border-collapse:collapse"><tr>
                ${tcell('r', ft(t.target_velocity_r, 3))}
                ${tcell('θ', fdeg(t.target_velocity_theta))}
                ${tcell('向き', fdeg(cmd.target_theta))}
            </tr></table>`;
        } else if (cmd.target_theta != null) {
            targetHtml = `<table style="width:100%;border-collapse:collapse"><tr>
                ${tcell('θ', fdeg(cmd.target_theta))}
            </tr></table>`;
        }
    }
    if (!targetHtml) targetHtml = '<span class="text-secondary" style="font-size:0.82rem">データなし</span>';
    document.getElementById('panel-target').innerHTML = targetHtml;

    // --- プランニング要素 ---
    let factorsHtml = '';
    if (cmd?.planning_factors?.length) {
        for (const f of cmd.planning_factors) {
            const stateVal = parseFloat(f.state);
            const pct = isNaN(stateVal) ? 0 : Math.max(0, Math.min(100, stateVal * 100));
            const stateStr = isNaN(stateVal) ? String(f.state) : stateVal.toFixed(3);
            factorsHtml += `
<div style="margin-bottom:0.5rem">
  <div style="display:flex; justify-content:space-between; font-size:0.8rem">
    <span>${f.name}</span>
    <span class="text-secondary">${stateStr}</span>
  </div>
  <div class="factor-bar"><div class="factor-bar-fill" style="width:${pct}%"></div></div>
</div>`;
        }
    } else {
        factorsHtml = '<span class="text-secondary" style="font-size:0.82rem">データなし</span>';
    }
    document.getElementById('panel-factors').innerHTML = factorsHtml;

    // --- ハードウェア ---
    let hwHtml = '';
    const fb = state.feedback;
    const fmt = (v, digits) => { const n = parseFloat(v); return isNaN(n) ? '--' : n.toFixed(digits); };
    if (fb) {
        if (fb.voltage?.length) hwHtml += row('電圧', `${fmt(fb.voltage[0], 2)} V`);
        if (fb.temperatures?.length) hwHtml += row('最高温度', `${Math.max(...fb.temperatures.map(parseFloat))} °C`);
        if (fb.yaw_angle != null) hwHtml += row('Yaw角', `${fmt(fb.yaw_angle, 2)} °`);
        if (fb.odom_speed != null) hwHtml += row('オドム速度', `${fmt(fb.odom_speed, 3)} m/s`);
        if (fb.ball_sensor != null) hwHtml += row('ボールセンサ', fb.ball_sensor ? '<span class="text-success">検知</span>' : '未検知');
    }
    if (state.ping_ms !== null) hwHtml += row('Ping', `${fmt(state.ping_ms, 1)} ms`);
    if (!hwHtml) hwHtml = row('--', '<span class="text-secondary">データなし</span>');
    document.getElementById('panel-hw').innerHTML = hwHtml;
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

initTable();
renderAll();
connectWebSocket();

// 500ms ごとに DOM 更新
setInterval(() => {
    if (pendingUpdate) {
        renderAll();
        pendingUpdate = false;
    }
}, 500);
