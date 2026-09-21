// C-3 のテストレイヤー。
//
// canvas は 1 枚のまま。DOM レイヤーを重ねると座標変換が二重になり
// clientToFieldCoords が使えなくなるので、下地を沈める方式にする。
// 「TEST LAYER がマウスを受け取り中」であることを、破線枠と下地の減光で示す。

const DIM_ALPHA = 0.38;
const FRAME_DASH = [900, 500];
const TARGET_R = 220;        // mm
const THETA_LEN = 420;       // mm
const VEL_RING_SCALE = 260;  // mm per (m/s)

export class TestLayer {
    // 下地の上に被せる減光。SVG レイヤーと HUD を描いた直後に呼ぶ
    static drawDim(ctx, fieldLayer, tokens) {
        ctx.save();
        ctx.globalAlpha = DIM_ALPHA;
        ctx.fillStyle = tokens.fieldTurf ?? '#12291B';
        ctx.fillRect(fieldLayer.vbX, fieldLayer.vbY, fieldLayer.vbW, fieldLayer.vbH);
        ctx.restore();
    }

    // テストの枠・目標・θ・速度上限リング
    static draw(ctx, fieldLayer, tokens, session, robot) {
        const accent = tokens.select ?? '#7FE3FF';
        ctx.save();
        ctx.setLineDash(FRAME_DASH);
        ctx.strokeStyle = accent;
        ctx.globalAlpha = 0.9;
        ctx.lineWidth = 26;
        ctx.strokeRect(
            fieldLayer.vbX + 60, fieldLayer.vbY + 60,
            fieldLayer.vbW - 120, fieldLayer.vbH - 120,
        );
        ctx.restore();

        if (robot) TestLayer._drawVelocityRing(ctx, accent, session, robot);
        if (session.targetPos) TestLayer._drawTarget(ctx, accent, session, robot);
    }

    static _drawTarget(ctx, accent, session, robot) {
        const tx = session.targetPos.x * 1000;
        const ty = -session.targetPos.y * 1000;

        ctx.save();
        ctx.setLineDash([]);
        ctx.strokeStyle = accent;
        ctx.globalAlpha = 0.95;
        ctx.lineWidth = 22;
        ctx.beginPath();
        ctx.arc(tx, ty, TARGET_R, 0, Math.PI * 2);
        ctx.stroke();

        // 十字（中心が一目で分かるように）
        ctx.lineWidth = 14;
        ctx.beginPath();
        ctx.moveTo(tx - TARGET_R * 1.5, ty);
        ctx.lineTo(tx + TARGET_R * 1.5, ty);
        ctx.moveTo(tx, ty - TARGET_R * 1.5);
        ctx.lineTo(tx, ty + TARGET_R * 1.5);
        ctx.stroke();

        // 目標姿勢
        ctx.lineWidth = 20;
        ctx.beginPath();
        ctx.moveTo(tx, ty);
        ctx.lineTo(tx + Math.cos(session.targetTheta) * THETA_LEN,
            ty - Math.sin(session.targetTheta) * THETA_LEN);
        ctx.stroke();

        // 現在地から目標への線
        if (robot) {
            ctx.globalAlpha = 0.45;
            ctx.lineWidth = 12;
            ctx.setLineDash([120, 90]);
            ctx.beginPath();
            ctx.moveTo(robot.x * 1000, -robot.y * 1000);
            ctx.lineTo(tx, ty);
            ctx.stroke();
        }
        ctx.restore();
    }

    // Shift+ドラッグで変える速度上限を、ロボットを中心にした破線円で示す。
    // スライダと同じ値を見せることで「動かしただけでは何も起きない」
    // （送信時に同梱される値である）ことを目で分かるようにする。
    static _drawVelocityRing(ctx, accent, session, robot) {
        if (!session.limitsEffective) return;
        ctx.save();
        ctx.setLineDash([80, 60]);
        ctx.strokeStyle = accent;
        ctx.globalAlpha = 0.4;
        ctx.lineWidth = 12;
        ctx.beginPath();
        ctx.arc(robot.x * 1000, -robot.y * 1000, session.maxVelocity * VEL_RING_SCALE, 0, Math.PI * 2);
        ctx.stroke();
        ctx.restore();
    }
}
