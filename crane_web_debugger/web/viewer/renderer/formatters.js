import { CONTROL_MODE_SHORT, CONTROL_MODE_LONG } from './constants.js';

export function formatPlannerName(name, maxLen = 20) {
    if (!name) return '--';
    return name.length > maxLen ? '…' + name.slice(-(maxLen - 1)) : name;
}

export function getFsmState(cmd) {
    return cmd?.planning_factors?.[0]?.name ?? null;
}

export function getControlModeShort(cmd) {
    if (!cmd) return '--';
    return CONTROL_MODE_SHORT[cmd.control_mode] ?? `M${cmd.control_mode}`;
}

export function getControlModeLong(cmd) {
    if (!cmd) return '--';
    return CONTROL_MODE_LONG[cmd.control_mode] ?? `MODE_${cmd.control_mode}`;
}

export function formatLatencyRich(latEst) {
    if (!latEst || latEst.latency_ms == null) return 'N/A';
    const ms = latEst.latency_ms.toFixed(0);
    const corr = latEst.correlation != null ? ` (r=${(latEst.correlation).toFixed(2)})` : '';
    return `${ms}ms${corr}`;
}

export function formatLatencyMs(latEst) {
    if (!latEst || latEst.latency_ms == null) return 'N/A';
    return `${latEst.latency_ms.toFixed(0)}ms`;
}

// severity: 'ok' | 'warn' | 'crit'
export function formatVoltage(v, warnV = 22.5, critV = 21.0) {
    if (v == null) return { text: 'N/A', severity: 'ok' };
    const text = `${v.toFixed(1)} V`;
    if (v <= critV) return { text, severity: 'crit' };
    if (v <= warnV) return { text, severity: 'warn' };
    return { text, severity: 'ok' };
}

export function formatTemperature(temps) {
    if (!temps || temps.length === 0) return { max: null, text: 'N/A', severity: 'ok' };
    const max = Math.max(...temps);
    const text = `max ${max.toFixed(0)} °C`;
    if (max >= 75) return { max, text, severity: 'crit' };
    if (max >= 60) return { max, text, severity: 'warn' };
    return { max, text, severity: 'ok' };
}

export function formatKickState(state) {
    const map = { 0: 'READY', 1: 'CHARGING', 2: 'FIRING', 3: 'CHARGED' };
    return map[state] ?? `STATE_${state}`;
}

export function formatErrorBadge(fb) {
    if (!fb || fb.error_id === 0 || fb.error_id == null) return null;
    return { text: `E${fb.error_id}`, severity: 'crit' };
}

export function availabilityChips(robot) {
    return {
        vision: robot?.available_vision ?? false,
        feedback: robot?.available_feedback ?? false,
        tracker: robot?.available_tracker ?? false,
    };
}
