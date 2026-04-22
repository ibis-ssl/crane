const HALO_RADIUS = 120;         // mm
const ARROW_LEN = 150;
const ARROW_HEAD_LEN = 40;
const ARROW_HEAD_HALF_ANGLE = Math.PI / 6; // 30°
const ARROW_LINE_W = 18;
const FSM_Y = 170;               // mm below center in SVG space (+Y = down)
const PLANNER_Y = 260;
const DRIBBLER_R = 15;
const DRIBBLER_Y = 30;           // mm below center (front LED position)

export class RobotHud {
    // ctx は CanvasRenderer._render() 内の変換済みコンテキスト（SVG 座標系、1 unit = 1mm）
    draw(ctx, viewer, tokens) {
        const primary = viewer.selectedRobotId;
        const multi = viewer._multiSelect;
        const hover = viewer._hoveredRobotId;

        for (const [idStr, robot] of Object.entries(viewer.robotsOurs)) {
            const id = Number(idStr);
            const cx = robot.x * 1000;
            const cy = -robot.y * 1000;
            const theta = robot.theta ?? 0;
            const cmd = viewer.controlTargets[id];
            this._drawHalo(ctx, cx, cy, id, primary, multi, hover, tokens);
            this._drawArrow(ctx, cx, cy, theta, cmd, tokens);
            this._drawDribblerLed(ctx, cx, cy, id, viewer, tokens);
            if (cmd) this._drawLabels(ctx, cx, cy, cmd, tokens);
        }
    }

    _drawHalo(ctx, cx, cy, id, primary, multi, hover, tokens) {
        const accent = tokens.selectionHalo ?? tokens.hudAccent ?? '#D0BCFF';
        if (id === primary) {
            ctx.save();
            ctx.strokeStyle = accent;
            ctx.globalAlpha = 0.8;
            ctx.lineWidth = 24;
            ctx.setLineDash([]);
            ctx.beginPath();
            ctx.arc(cx, cy, HALO_RADIUS, 0, Math.PI * 2);
            ctx.stroke();
            ctx.restore();
        } else if (multi?.has(id)) {
            ctx.save();
            ctx.strokeStyle = accent;
            ctx.globalAlpha = 0.6;
            ctx.lineWidth = 8;
            ctx.setLineDash([20, 10]);
            ctx.beginPath();
            ctx.arc(cx, cy, HALO_RADIUS, 0, Math.PI * 2);
            ctx.stroke();
            ctx.restore();
        } else if (id === hover) {
            ctx.save();
            ctx.strokeStyle = accent;
            ctx.globalAlpha = 0.35;
            ctx.lineWidth = 6;
            ctx.setLineDash([10, 6]);
            ctx.beginPath();
            ctx.arc(cx, cy, HALO_RADIUS + 8, 0, Math.PI * 2);
            ctx.stroke();
            ctx.restore();
        }
    }

    _drawArrow(ctx, cx, cy, theta, cmd, tokens) {
        const hasDribble = (cmd?.dribble_power ?? 0) > 0;
        const color = hasDribble ? (tokens.moveOverlay ?? '#B5EAD7') : (tokens.hudAccent ?? '#D0BCFF');
        // SVG 座標系: tipX = cx + cos(θ)*L, tipY = cy - sin(θ)*L (Y反転)
        const tipX = cx + Math.cos(theta) * ARROW_LEN;
        const tipY = cy - Math.sin(theta) * ARROW_LEN;

        ctx.save();
        ctx.strokeStyle = color;
        ctx.globalAlpha = 0.85;
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

    _drawDribblerLed(ctx, cx, cy, id, viewer, tokens) {
        const hasBall = viewer.robotFeedback?.[id]?.ball_sensor ?? false;
        const color = hasBall ? (tokens.hudAccent ?? '#D0BCFF') : (tokens.noDataText ?? '#4A4458');
        ctx.save();
        ctx.fillStyle = color;
        ctx.globalAlpha = 0.9;
        ctx.beginPath();
        ctx.arc(cx, cy + DRIBBLER_Y, DRIBBLER_R, 0, Math.PI * 2);
        ctx.fill();
        ctx.restore();
    }

    _drawLabels(ctx, cx, cy, cmd, tokens) {
        const fsm = cmd.planning_factors?.[0]?.name ?? '';
        const planner = cmd.planner_name ?? '';
        if (!fsm && !planner) return;

        ctx.save();
        ctx.textAlign = 'center';
        ctx.setLineDash([]);
        ctx.globalAlpha = 0.9;

        if (fsm) {
            ctx.font = '80px sans-serif';
            ctx.fillStyle = tokens.hudText ?? '#E6E0E9';
            ctx.textBaseline = 'top';
            ctx.fillText(fsm, cx, cy + FSM_Y);
        }
        if (planner) {
            const name = planner.length > 20 ? '…' + planner.slice(-19) : planner;
            ctx.font = '64px sans-serif';
            ctx.fillStyle = tokens.noDataText ?? '#8C929A';
            ctx.textBaseline = 'top';
            ctx.fillText(name, cx, cy + PLANNER_Y);
        }
        ctx.restore();
    }
}
