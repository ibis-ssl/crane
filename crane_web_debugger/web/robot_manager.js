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
    const banner = document.getElementById('offline-banner');
    if (connected) {
        dot.className = 'm3-connection-dot connected';
        text.textContent = 'connected';
    } else {
        dot.className = 'm3-connection-dot';
        text.textContent = 'disconnected';
    }
    if (banner) banner.classList.toggle('visible', !connected);
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
    'Running': { badge: 'm3-badge--success', label: 'Running' },
    'Stopped': { badge: 'm3-badge--warning', label: 'Stopped' },
    'Error':   { badge: 'm3-badge--error',   label: 'Error' },
    'Offline': { badge: 'm3-badge--secondary', label: 'Offline' },
};

function statusBadgeClass(status) {
    return STATUS_CONFIG[status]?.badge ?? 'm3-badge--secondary';
}

function statusLabel(status) {
    return STATUS_CONFIG[status]?.label ?? (status || '不明');
}

function voltageBarColor(v) {
    if (v < 22.0) return 'var(--md-sys-color-error)';
    if (v < 23.0) return 'var(--md-sys-color-warning)';
    return 'var(--md-sys-color-success)';
}

function pingClass(ms) {
    if (ms === null) return 'm3-text-on-surface-variant';
    if (ms > PING_CRIT) return 'm3-text-error';
    if (ms > PING_WARN) return 'm3-text-warning';
    return 'm3-text-success';
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
    badge.className = `m3-badge ${statusBadgeClass(overallStatus)}`;
    badge.textContent = statusLabel(overallStatus);

    // Voltage
    const voltageEl = document.getElementById(`voltage-${id}`);
    const voltageBarEl = document.getElementById(`voltage-bar-${id}`);
    if (fb && fb.voltage && fb.voltage.length > 0) {
        const v = fb.voltage[0];
        const pct = Math.max(0, Math.min(100, (v - VOLTAGE_MIN) / (VOLTAGE_MAX - VOLTAGE_MIN) * 100));
        voltageEl.textContent = `${v.toFixed(2)} V`;
        voltageEl.className = `m3-fw-medium ${v < 22.0 ? 'm3-text-error' : v < 23.0 ? 'm3-text-warning' : 'm3-text-success'}`;
        voltageBarEl.style.width = `${pct}%`;
        voltageBarEl.style.backgroundColor = voltageBarColor(v);
    } else {
        voltageEl.textContent = '--';
        voltageEl.className = 'm3-fw-medium m3-text-on-surface-variant';
        voltageBarEl.style.width = '0%';
        voltageBarEl.style.backgroundColor = 'var(--md-sys-color-surface-container-highest)';
    }

    // Temperature
    const tempEl = document.getElementById(`temp-${id}`);
    if (fb && fb.temperatures && fb.temperatures.length > 0) {
        const maxTemp = Math.max(...fb.temperatures);
        tempEl.textContent = `${maxTemp} °C`;
        tempEl.className = `m3-fw-medium ${maxTemp >= TEMP_WARN ? 'm3-text-error' : ''}`;
    } else {
        tempEl.textContent = '--';
        tempEl.className = 'm3-fw-medium m3-text-on-surface-variant';
    }

    // Ping
    const pingEl = document.getElementById(`ping-${id}`);
    if (state.ping_ms !== null) {
        pingEl.textContent = `${state.ping_ms.toFixed(1)} ms`;
        pingEl.className = `m3-fw-medium ${pingClass(state.ping_ms)}`;
    } else {
        pingEl.textContent = '--';
        pingEl.className = 'm3-fw-medium m3-text-on-surface-variant';
    }

    // Packet frequency
    const freqEl = document.getElementById(`freq-${id}`);
    if (fb) {
        freqEl.textContent = `${fb.packet_frequency_hz.toFixed(1)} Hz`;
        freqEl.className = `m3-fw-medium ${fb.packet_frequency_hz < 50 ? 'm3-text-warning' : 'm3-text-success'}`;
    } else {
        freqEl.textContent = '--';
        freqEl.className = 'm3-fw-medium m3-text-on-surface-variant';
    }

    // Error
    const errorEl = document.getElementById(`error-${id}`);
    if (fb && fb.error_info) {
        errorEl.textContent = getErrorString(fb.error_id, fb.error_info);
        errorEl.className = 'error-text m3-text-error';
    } else {
        errorEl.textContent = 'OK';
        errorEl.className = 'error-text m3-text-success';
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
    <td><span class="material-symbols-outlined icon-sm m3-text-on-surface-variant" style="vertical-align:middle;margin-right:4px">smart_toy</span><strong>#${id}</strong></td>
    <td>
        <div class="m3-flex m3-items-center m3-gap-sm">
            <span class="m3-badge m3-badge--secondary" id="badge-${id}" style="min-width:60px">--</span>
            <div class="m3-flex m3-gap-xs">
                <button class="m3-icon-btn m3-icon-btn--sm" onclick="sendRobotControl(${id}, 'start')" title="Start" style="color:var(--md-sys-color-success)">
                    <span class="material-symbols-outlined icon-sm">play_arrow</span>
                </button>
                <button class="m3-icon-btn m3-icon-btn--sm" onclick="sendRobotControl(${id}, 'stop')" title="Stop" style="color:var(--md-sys-color-error)">
                    <span class="material-symbols-outlined icon-sm">stop</span>
                </button>
            </div>
        </div>
    </td>
    <td>
        <span class="m3-fw-medium m3-text-on-surface-variant" id="voltage-${id}">--</span>
        <div class="m3-linear-progress voltage-bar">
            <div class="m3-linear-progress__bar" id="voltage-bar-${id}" style="width:0%"></div>
        </div>
    </td>
    <td><span class="m3-fw-medium m3-text-on-surface-variant" id="temp-${id}">--</span></td>
    <td><span class="m3-fw-medium m3-text-on-surface-variant" id="ping-${id}">--</span></td>
    <td><span class="m3-fw-medium m3-text-on-surface-variant" id="freq-${id}">--</span></td>
    <td><span class="error-text m3-text-success" id="error-${id}">OK</span></td>
    <td>
        <button class="m3-icon-btn m3-icon-btn--sm" onclick="openDetailPanel(${id})" title="Details" style="color:var(--md-sys-color-info)">
            <span class="material-symbols-outlined icon-sm">info</span>
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
            `<td style="padding:1px 6px 1px 0;font-size:0.78rem"><span style="color:var(--md-sys-color-on-surface-variant)">${label}</span> ${val}</td>`;
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
                ok ? `<span class="m3-text-success">${label}</span>` : `<span class="m3-text-on-surface-variant">${label}</span>`;
            poseHtml += `<tr><td colspan="3" style="padding-top:4px;font-size:0.78rem">
                ${flag(pose.available_vision, 'Vision')}
                ${flag(pose.available_feedback, 'FB')}
                ${flag(pose.available_tracker, 'Tracker')}
            </td></tr>`;
        }
        poseHtml += '</table>';
    } else {
        poseHtml = '<span class="m3-text-on-surface-variant m3-body-small">No data</span>';
    }
    document.getElementById('panel-pose').innerHTML = poseHtml;

    // --- Session / スキル ---
    let sessionHtml = '';
    if (cmd) {
        const fc = (v, d) => { const n = parseFloat(v); return isNaN(n) ? '--' : n.toFixed(d); };
        const modeName = CONTROL_MODE_LONG[cmd.control_mode] ?? `MODE_${cmd.control_mode}`;
        sessionHtml += row('Planner', cmd.planner_name || '--');
        sessionHtml += row('Control Mode', modeName);
        const kick = parseFloat(cmd.kick_power);
        sessionHtml += row('Kick', kick > 0 ? `<span class="m3-text-warning">${fc(cmd.kick_power, 2)}</span>` : '0');
        const drib = parseFloat(cmd.dribble_power);
        sessionHtml += row('Dribble', drib > 0 ? `<span class="m3-text-info">${fc(cmd.dribble_power, 2)}</span>` : '0');
        sessionHtml += row('Chip', cmd.chip_enable ? '<span class="m3-text-warning">ON</span>' : 'OFF');
    } else {
        sessionHtml = row('--', '<span class="m3-text-on-surface-variant">No data</span>');
    }
    document.getElementById('panel-session').innerHTML = sessionHtml;

    // --- 目標値 ---
    let targetHtml = '';
    if (cmd) {
        const ft = (v, d) => { const n = parseFloat(v); return isNaN(n) ? '--' : n.toFixed(d); };
        const fdeg = (v) => { const n = parseFloat(v); return isNaN(n) ? '--' : (n * 180 / Math.PI).toFixed(1) + '°'; };
        const tcell = (label, val) =>
            `<td style="padding:1px 6px 1px 0;font-size:0.78rem"><span style="color:var(--md-sys-color-on-surface-variant)">${label}</span> ${val}</td>`;
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
    if (!targetHtml) targetHtml = '<span class="m3-text-on-surface-variant m3-body-small">No data</span>';
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
    <span class="m3-text-on-surface-variant">${stateStr}</span>
  </div>
  <div class="factor-bar"><div class="factor-bar-fill" style="width:${pct}%"></div></div>
</div>`;
        }
    } else {
        factorsHtml = '<span class="m3-text-on-surface-variant m3-body-small">No data</span>';
    }
    document.getElementById('panel-factors').innerHTML = factorsHtml;

    // --- ハードウェア ---
    let hwHtml = '';
    const fb = state.feedback;
    const fmt = (v, digits) => { const n = parseFloat(v); return isNaN(n) ? '--' : n.toFixed(digits); };
    if (fb) {
        if (fb.voltage?.length) hwHtml += row('Voltage', `${fmt(fb.voltage[0], 2)} V`);
        if (fb.temperatures?.length) hwHtml += row('Max Temp', `${Math.max(...fb.temperatures.map(parseFloat))} °C`);
        if (fb.yaw_angle != null) hwHtml += row('Yaw', `${fmt(fb.yaw_angle, 2)} °`);
        if (fb.odom_speed != null) hwHtml += row('Odom Speed', `${fmt(fb.odom_speed, 3)} m/s`);
        if (fb.ball_sensor != null) hwHtml += row('Ball Sensor', fb.ball_sensor ? '<span class="m3-text-success">Detected</span>' : 'None');
    }
    if (state.ping_ms !== null) hwHtml += row('Ping', `${fmt(state.ping_ms, 1)} ms`);
    if (!hwHtml) hwHtml = row('--', '<span class="m3-text-on-surface-variant">No data</span>');
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
