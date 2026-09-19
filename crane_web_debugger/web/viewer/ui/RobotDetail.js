// 右サイドバーのロボット詳細。
//
// V4 で概要 / テレメトリ / テスト / ログのタブへ分割する。ここはその「概要」に
// あたる中身で、いまは単一パネルとして丸ごと描いている。

import {
    formatPlannerName, getFsmState, getControlModeLong,
    formatLatencyRich, formatVoltage, formatTemperature,
    formatKickState, formatErrorBadge, availabilityChips,
} from '../renderer/formatters.js';

const PACKET_FREQ_WARN_HZ = 80;
const LATENCY_WARN_MS = 100;

export class RobotDetail {
    constructor(state) {
        this._state = state;
    }

    get panel() { return document.getElementById('robot-detail-inline'); }

    show(id) {
        const panel = this.panel;
        if (!panel) return false;
        this.render(id);
        panel.classList.add('visible');
        return true;
    }

    hide() {
        this.panel?.classList.remove('visible');
    }

    render(id) {
        const panel = this.panel;
        if (!panel) return;
        const robot = this._state.robotsOurs[id];
        if (!robot) return;
        const cmd = this._state.controlTargets[id];

        panel.innerHTML = `
            <div class="rd-title">Robot ${id}
                <a href="/robot_telemetry.html?id=${id}" target="_blank" class="rd-telem-link" title="Telemetry を別タブで開く">
                    <span class="material-symbols-outlined icon-sm">open_in_new</span>
                </a>
            </div>
            ${this._commandHtml(cmd)}
            ${this._stateHtml(robot)}
            ${this._feedbackHtml(id)}
            ${this._latencyHtml(id)}
        `;
    }

    _commandHtml(cmd) {
        if (!cmd) return '';
        let targetHtml = '';
        if (cmd.position_target_mode) {
            targetHtml = `<tr><td>Target Pos</td><td>(${cmd.position_target_mode.target_x?.toFixed(2)}, ${cmd.position_target_mode.target_y?.toFixed(2)})</td></tr>`;
        } else if (cmd.simple_velocity_target_mode) {
            targetHtml = `<tr><td>Target Vel</td><td>(${cmd.simple_velocity_target_mode.target_vx?.toFixed(2)}, ${cmd.simple_velocity_target_mode.target_vy?.toFixed(2)})</td></tr>`;
        }
        return `
                <div class="rd-section-title">Command</div>
                <table><tbody>
                    <tr><td>Mode</td><td>${getControlModeLong(cmd)}</td></tr>
                    <tr><td>Planner</td><td>${formatPlannerName(cmd.planner_name)}</td></tr>
                    <tr><td>FSM</td><td>${getFsmState(cmd) ?? '--'}</td></tr>
                    ${targetHtml}
                    <tr><td>Target θ</td><td>${cmd.target_theta?.toFixed(3) ?? '--'}</td></tr>
                </tbody></table>`;
    }

    _stateHtml(robot) {
        const chips = availabilityChips(robot);
        const chipHtml = ['vision', 'feedback', 'tracker'].map(k =>
            `<span class="rd-chip ${chips[k] ? 'rd-chip--ok' : 'rd-chip--ng'}">${k}</span>`
        ).join('');
        return `
            <div class="rd-section-title">State</div>
            <table><tbody>
                <tr><td>Pos</td><td>(${robot.x?.toFixed(3)}, ${robot.y?.toFixed(3)}) m</td></tr>
                <tr><td>θ</td><td>${robot.theta?.toFixed(3)} rad</td></tr>
                <tr><td>Vel</td><td>(${robot.vx?.toFixed(3)}, ${robot.vy?.toFixed(3)}) m/s</td></tr>
                <tr><td>ω</td><td>${robot.omega?.toFixed(3)} rad/s</td></tr>
                ${robot.acceleration_x != null ? `<tr><td>Accel</td><td>(${robot.acceleration_x?.toFixed(2)}, ${robot.acceleration_y?.toFixed(2)}, ${robot.acceleration_theta?.toFixed(2)})</td></tr>` : ''}
                <tr><td>Avail</td><td>${chipHtml}</td></tr>
            </tbody></table>`;
    }

    _feedbackHtml(id) {
        const fb = this._state.robotFeedback[id];
        if (!fb) return '';
        const volt = formatVoltage(fb.voltage);
        const temp = formatTemperature(fb.temperatures);
        const err = formatErrorBadge(fb);
        const tempTitle = fb.temperatures ? fb.temperatures.map((t, i) => `[${i}] ${t.toFixed(0)}°C`).join(' ') : '';
        return `
                <div class="rd-section-title">Feedback</div>
                <table><tbody>
                    <tr><td>Voltage</td><td class="rd-${volt.severity}">${volt.text}</td></tr>
                    <tr><td>Temp</td><td class="rd-${temp.severity}" title="${tempTitle}">${temp.text}</td></tr>
                    <tr><td>Packet</td><td class="${(fb.packet_frequency_hz ?? 0) < PACKET_FREQ_WARN_HZ ? 'rd-warn' : ''}">${fb.packet_frequency_hz?.toFixed(0) ?? 'N/A'} Hz</td></tr>
                    <tr><td>Kick</td><td>${formatKickState(fb.kick_state)}</td></tr>
                    <tr><td>Ball</td><td>${fb.ball_sensor ? '●' : '○'}</td></tr>
                    <tr><td>Error</td><td class="${err ? 'rd-crit' : ''}">${err ? `id=${fb.error_id} ${fb.error_info ?? ''}` : 'none'}</td></tr>
                    ${fb.motor_current ? `<tr><td>Motor I</td><td>[${fb.motor_current.map(v => v.toFixed(1)).join(', ')}]</td></tr>` : ''}
                </tbody></table>`;
    }

    _latencyHtml(id) {
        const latEst = this._state.latencyEstimation[id] ?? {};
        const rows = ['world_model', 'robot_feedback'].flatMap(src => {
            const e = latEst[src];
            if (!e) return [];
            const label = src === 'world_model' ? 'WM' : 'HW';
            const high = e.latency_ms > LATENCY_WARN_MS;
            return [`<tr><td>${label}</td><td class="${high ? 'rd-warn' : ''}">${formatLatencyRich(e)}</td></tr>`];
        }).join('');
        return rows ? `
            <div class="rd-section-title">Latency</div>
            <table><tbody>${rows}</tbody></table>` : '';
    }
}
