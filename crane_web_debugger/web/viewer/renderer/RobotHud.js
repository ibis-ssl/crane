import { formatPlannerName, getFsmState, formatLatencyMs, formatLatencyRich } from './formatters.js';
import {
    VOLTAGE_CRIT_V, VOLTAGE_WARN_V, TEMP_CRIT_C, FEEDBACK_STALE_MS, LATENCY_WARN_MS,
} from './constants.js';

const HALO_RADIUS = 120;         // mm  C-2 の二重リング内側（実線）
const HALO_RADIUS_OUTER = 165;   // mm  同 外側（細線）
// フォーカス中のロボットが居るとき、それ以外を沈めて視線を 1 台に集める（C-2）
const NON_FOCUS_ALPHA = 0.62;
const ARROW_LEN = 150;
const ARROW_HEAD_LEN = 40;
const ARROW_HEAD_HALF_ANGLE = Math.PI / 6; // 30°
const ARROW_LINE_W = 18;
const FSM_Y = 170;               // mm below center in SVG space (+Y = down)
const PLANNER_Y = 260;
const LATENCY_Y = 360;
const HW_LATENCY_Y = 460;        // mm below center
const DRIBBLER_R = 15;
const DRIBBLER_Y = 30;           // mm below center (front LED position)

// ズーム閾値
const ZOOM_MEDIUM = 1.5;
const ZOOM_HIGH = 2.0;

// 警告バッジサイズ (mm)
const BADGE_R = 32;
const BADGE_OFFSET_X = 95;
const BADGE_OFFSET_Y = -95;

export class RobotHud {
    // ctx は CanvasRenderer._render() 内の変換済みコンテキスト（SVG 座標系、1 unit = 1mm）
    draw(ctx, viewer, tokens) {
        const focus = viewer.focusedRobotId;
        const multi = viewer._multiSelect;
        const hover = viewer._hoveredRobotId;
        const zoom = viewer.zoomLevel ?? 1.0;

        for (const [idStr, robot] of Object.entries(viewer.robotsOurs)) {
            if (!robot.available_vision && !robot.available_tracker) continue;
            const id = Number(idStr);
            const cx = robot.x * 1000;
            const cy = -robot.y * 1000;
            const theta = robot.theta ?? 0;
            const cmd = viewer.controlTargets[id];
            // 各描画メソッドは自前で globalAlpha を設定するので、Canvas では
            // アルファが入れ子で乗算されない。減光率を引数で渡して掛け合わせる。
            const a = (focus !== null && id !== focus) ? NON_FOCUS_ALPHA : 1.0;
            this._drawHalo(ctx, cx, cy, id, focus, multi, hover, tokens, a);
            this._drawArrow(ctx, cx, cy, theta, cmd, tokens, a);
            this._drawDribblerLed(ctx, cx, cy, id, viewer, tokens, a);
            const latEst = viewer.latencyEstimation?.[id];
            if (cmd || latEst) this._drawLabels(ctx, cx, cy, cmd, tokens, latEst, zoom, a);
            // 警告バッジだけは減光しない。異常の通知はフォーカスの有無で弱めない。
            this._drawWarningBadges(ctx, cx, cy, id, viewer, tokens);
        }
    }

    // フォーカス = C-2 の二重リング。
    _drawHalo(ctx, cx, cy, id, focus, multi, hover, tokens, alpha) {
        const accent = tokens.select ?? '#7FE3FF';
        ctx.save();
        ctx.setLineDash([]);
        if (id === focus) {
            ctx.strokeStyle = accent;
            ctx.globalAlpha = 0.85 * alpha;
            ctx.lineWidth = 24;
            ctx.beginPath();
            ctx.arc(cx, cy, HALO_RADIUS, 0, Math.PI * 2);
            ctx.stroke();

            ctx.globalAlpha = 0.45 * alpha;
            ctx.lineWidth = 8;
            ctx.beginPath();
            ctx.arc(cx, cy, HALO_RADIUS_OUTER, 0, Math.PI * 2);
            ctx.stroke();
        } else if (multi?.has(id)) {
            ctx.strokeStyle = accent;
            ctx.globalAlpha = 0.6 * alpha;
            ctx.lineWidth = 8;
            ctx.setLineDash([20, 10]);
            ctx.beginPath();
            ctx.arc(cx, cy, HALO_RADIUS, 0, Math.PI * 2);
            ctx.stroke();
        } else if (id === hover) {
            ctx.strokeStyle = accent;
            ctx.globalAlpha = 0.35 * alpha;
            ctx.lineWidth = 6;
            ctx.setLineDash([10, 6]);
            ctx.beginPath();
            ctx.arc(cx, cy, HALO_RADIUS + 8, 0, Math.PI * 2);
            ctx.stroke();
        }
        ctx.restore();
    }

    _drawArrow(ctx, cx, cy, theta, cmd, tokens, alpha) {
        const hasDribble = (cmd?.dribble_power ?? 0) > 0;
        const color = hasDribble ? (tokens.overlayMove ?? '#D0BCFF') : (tokens.hudAccent ?? '#A0C4FF');
        // SVG 座標系: tipX = cx + cos(θ)*L, tipY = cy - sin(θ)*L (Y反転)
        const tipX = cx + Math.cos(theta) * ARROW_LEN;
        const tipY = cy - Math.sin(theta) * ARROW_LEN;

        ctx.save();
        ctx.strokeStyle = color;
        ctx.globalAlpha = 0.85 * alpha;
        ctx.lineWidth = ARROW_LINE_W;
        ctx.lineCap = 'round';
        ctx.setLineDash([]);
        ctx.beginPath();
        ctx.moveTo(cx, cy);
        ctx.lineTo(tipX, tipY);
        ctx.stroke();

        // 矢頭 (V字)
        ctx.lineWidth = ARROW_LINE_W * 0.7;
        for (const sign of [1, -1]) {
            const ha = theta + Math.PI + sign * ARROW_HEAD_HALF_ANGLE;
            ctx.beginPath();
            ctx.moveTo(tipX, tipY);
            ctx.lineTo(tipX + Math.cos(ha) * ARROW_HEAD_LEN, tipY - Math.sin(ha) * ARROW_HEAD_LEN);
            ctx.stroke();
        }
        ctx.restore();
    }

    _drawDribblerLed(ctx, cx, cy, id, viewer, tokens, alpha) {
        const hasBall = viewer.robotFeedback?.[id]?.ball_sensor ?? false;
        const color = hasBall ? (tokens.hudAccent ?? '#A0C4FF') : (tokens.inkMuted ?? '#8C929A');
        ctx.save();
        ctx.fillStyle = color;
        ctx.globalAlpha = 0.9 * alpha;
        ctx.beginPath();
        ctx.arc(cx, cy + DRIBBLER_Y, DRIBBLER_R, 0, Math.PI * 2);
        ctx.fill();
        ctx.restore();
    }

    _drawLabels(ctx, cx, cy, cmd, tokens, latEst, zoomLevel, alpha) {
        const fsm = getFsmState(cmd) ?? '';
        const planner = cmd?.planner_name ?? '';
        const wmEst = latEst?.world_model;
        const hwEst = latEst?.robot_feedback;

        ctx.save();
        ctx.textAlign = 'center';
        ctx.setLineDash([]);
        ctx.globalAlpha = 0.9 * alpha;

        if (fsm) {
            ctx.font = '80px sans-serif';
            ctx.fillStyle = tokens.ink ?? '#E2E4E8';
            ctx.textBaseline = 'top';
            ctx.fillText(fsm, cx, cy + FSM_Y);
        }
        if (planner) {
            ctx.font = '64px sans-serif';
            ctx.fillStyle = tokens.inkMuted ?? '#8C929A';
            ctx.textBaseline = 'top';
            ctx.fillText(formatPlannerName(planner), cx, cy + PLANNER_Y);
        }

        // WM latency は常時表示
        if (wmEst?.latency_ms != null) {
            const latColor = wmEst.latency_ms > LATENCY_WARN_MS
                ? (tokens.danger ?? '#B3261E')
                : (tokens.ok ?? '#F4DFF0');
            ctx.font = '56px sans-serif';
            ctx.fillStyle = latColor;
            ctx.textBaseline = 'top';

            if (zoomLevel >= ZOOM_HIGH) {
                // ズーム大: WM/HW + correlation 値
                const wmLabel = `WM: ${formatLatencyRich(wmEst)}`;
                ctx.fillText(wmLabel, cx, cy + LATENCY_Y);
                if (hwEst?.latency_ms != null) {
                    const hwColor = hwEst.latency_ms > LATENCY_WARN_MS
                        ? (tokens.danger ?? '#B3261E')
                        : (tokens.ok ?? '#F4DFF0');
                    ctx.fillStyle = hwColor;
                    ctx.fillText(`HW: ${formatLatencyRich(hwEst)}`, cx, cy + HW_LATENCY_Y);
                }
            } else if (zoomLevel >= ZOOM_MEDIUM) {
                // ズーム中: WM/HW のみ (ms)
                ctx.fillText(`WM: ${formatLatencyMs(wmEst)}`, cx, cy + LATENCY_Y);
                if (hwEst?.latency_ms != null) {
                    const hwColor = hwEst.latency_ms > LATENCY_WARN_MS
                        ? (tokens.danger ?? '#B3261E')
                        : (tokens.ok ?? '#F4DFF0');
                    ctx.fillStyle = hwColor;
                    ctx.fillText(`HW: ${formatLatencyMs(hwEst)}`, cx, cy + HW_LATENCY_Y);
                }
            } else {
                // 通常: WM のみ (ms)
                ctx.fillText(formatLatencyMs(wmEst), cx, cy + LATENCY_Y);
            }
        }
        ctx.restore();
    }

    _drawWarningBadges(ctx, cx, cy, id, viewer, tokens) {
        const fb = viewer.robotFeedback?.[id];
        const badges = [];

        // feedback stale チェック
        const fbTs = viewer._feedbackTimestamp?.[id] ?? 0;
        const stale = fb && (Date.now() - fbTs > FEEDBACK_STALE_MS);

        if (fb && !stale) {
            if (fb.error_id != null && fb.error_id !== 0) {
                badges.push({ label: '!', color: tokens.danger ?? '#B3261E', ink: tokens.onDanger ?? '#FFFFFF' });
            }
            const v = fb.voltage;
            if (v != null && v <= VOLTAGE_CRIT_V) {
                badges.push({ label: 'V', color: tokens.crit ?? '#6650A4', ink: tokens.onCrit ?? '#FFFFFF' });
            } else if (v != null && v <= VOLTAGE_WARN_V) {
                badges.push({ label: 'V', color: tokens.warn ?? '#F9A825', ink: tokens.onWarn ?? '#2A2000' });
            }
            const temps = fb.temperatures;
            if (temps && temps.length > 0) {
                const maxT = Math.max(...temps);
                if (maxT >= TEMP_CRIT_C) {
                    badges.push({ label: 'T', color: tokens.warn ?? '#F9A825', ink: tokens.onWarn ?? '#2A2000' });
                }
            }
        }

        if (badges.length === 0) return;

        ctx.save();
        ctx.setLineDash([]);
        for (let i = 0; i < badges.length; i++) {
            const bx = cx + BADGE_OFFSET_X + i * (BADGE_R * 2 + 8);
            const by = cy + BADGE_OFFSET_Y;
            ctx.fillStyle = badges[i].color;
            ctx.globalAlpha = 0.9;
            ctx.beginPath();
            ctx.arc(bx, by, BADGE_R, 0, Math.PI * 2);
            ctx.fill();
            ctx.fillStyle = badges[i].ink ?? '#FFFFFF';
            ctx.globalAlpha = 1.0;
            ctx.font = `bold ${BADGE_R * 1.2}px sans-serif`;
            ctx.textAlign = 'center';
            ctx.textBaseline = 'middle';
            ctx.fillText(badges[i].label, bx, by);
        }
        ctx.restore();
    }
}
