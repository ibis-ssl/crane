// SVG path の d 属性コマンド列パーサー
export function parseSvgPath(d) {
    const segments = [];
    const re = /([MLHVZACSQTmlhvzacsqt])\s*([-+0-9.,eE\s]*)/g;
    let m;
    while ((m = re.exec(d)) !== null) {
        const cmd = m[1];
        const argStr = m[2].trim();
        const args = argStr.length === 0
            ? []
            : argStr.split(/[\s,]+/).filter(s => s.length > 0).map(Number);
        segments.push({ cmd, args });
    }
    return segments;
}

// SVG arc → Canvas ellipse 変換 (W3C SVG仕様 Appendix F.6)
export function svgArcToCanvas(ctx, x1, y1, rx, ry, phiDeg, fA, fS, x2, y2) {
    if (x1 === x2 && y1 === y2) return;
    rx = Math.abs(rx);
    ry = Math.abs(ry);
    if (rx === 0 || ry === 0) { ctx.lineTo(x2, y2); return; }

    const phi = (phiDeg % 360) * Math.PI / 180;
    const cosPhi = Math.cos(phi);
    const sinPhi = Math.sin(phi);
    const dx = (x1 - x2) / 2;
    const dy = (y1 - y2) / 2;
    const x1p = cosPhi * dx + sinPhi * dy;
    const y1p = -sinPhi * dx + cosPhi * dy;
    const x1p2 = x1p * x1p;
    const y1p2 = y1p * y1p;
    let rx2 = rx * rx;
    let ry2 = ry * ry;

    const lambda = Math.sqrt(x1p2 / rx2 + y1p2 / ry2);
    if (lambda > 1) {
        rx *= lambda; ry *= lambda;
        rx2 = rx * rx; ry2 = ry * ry;
    }

    const num = Math.max(0, rx2 * ry2 - rx2 * y1p2 - ry2 * x1p2);
    const den = rx2 * y1p2 + ry2 * x1p2;
    const sq = den === 0 ? 0 : Math.sqrt(num / den);
    const sign = (fA === fS) ? -1 : 1;
    const cxp = sign * sq * (rx * y1p / ry);
    const cyp = sign * sq * (-ry * x1p / rx);
    const cx = cosPhi * cxp - sinPhi * cyp + (x1 + x2) / 2;
    const cy = sinPhi * cxp + cosPhi * cyp + (y1 + y2) / 2;

    const vecAngle = (ux, uy, vx, vy) => {
        const dot = ux * vx + uy * vy;
        const len = Math.sqrt(ux * ux + uy * uy) * Math.sqrt(vx * vx + vy * vy);
        let a = Math.acos(Math.max(-1, Math.min(1, dot / len)));
        if (ux * vy - uy * vx < 0) a = -a;
        return a;
    };

    const theta1 = vecAngle(1, 0, (x1p - cxp) / rx, (y1p - cyp) / ry);
    let dtheta = vecAngle(
        (x1p - cxp) / rx, (y1p - cyp) / ry,
        (-x1p - cxp) / rx, (-y1p - cyp) / ry
    );
    if (!fS && dtheta > 0) dtheta -= 2 * Math.PI;
    else if (fS && dtheta < 0) dtheta += 2 * Math.PI;

    ctx.ellipse(cx, cy, rx, ry, phi, theta1, theta1 + dtheta, dtheta < 0);
}
