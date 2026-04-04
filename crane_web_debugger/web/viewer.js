'use strict';

// ---- 定数 ----
const CONTROL_MODE_SHORT = { 0: 'CAM', 1: 'POS', 2: 'VEL', 3: 'POL' };
const CONTROL_MODE_LONG = {
    0: 'LOCAL_CAMERA', 1: 'POSITION_TARGET', 2: 'SIMPLE_VELOCITY', 3: 'POLAR_VELOCITY'
};
const ROBOT_HIT_RADIUS_M = 0.15;
const VB_X = -6000, VB_Y = -4500, VB_W = 12000, VB_H = 9000;

// ---- SVG属性regex生成ヘルパー ----
function mkAttrRe(name) {
    return new RegExp(`${name}="([^"]*)"`);
}

// ---- SVGパスのdコマンド列パーサー ----
// returns: [{cmd: 'M', args: [x, y]}, ...]
function parseSvgPath(d) {
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

// ---- SVG arc → Canvas ellipse 変換 (W3C SVG仕様 Appendix F.6) ----
function svgArcToCanvas(ctx, x1, y1, rx, ry, phiDeg, fA, fS, x2, y2) {
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

    // 半径が小さすぎる場合はスケーリング
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

// ---- SVGプリミティブパーサー ----
// 正規表現をプリコンパイル（crane_visualizer_wrapperの固定フォーマットに合わせて最適化）
const RE_CX = mkAttrRe('cx');
const RE_CY = mkAttrRe('cy');
const RE_R = /\br="([^"]*)"/;
const RE_X1 = mkAttrRe('x1');
const RE_Y1 = mkAttrRe('y1');
const RE_X2 = mkAttrRe('x2');
const RE_Y2 = mkAttrRe('y2');
const RE_X = /(?:^|\s)x="([^"]*)"/;
const RE_Y = /(?:^|\s)y="([^"]*)"/;
const RE_W = mkAttrRe('width');
const RE_H = mkAttrRe('height');
const RE_POINTS = mkAttrRe('points');
const RE_D = mkAttrRe('d');
const RE_FILL = mkAttrRe('fill');
const RE_FILL_OP = mkAttrRe('fill-opacity');
const RE_STROKE = mkAttrRe('stroke');
const RE_STROKE_OP = mkAttrRe('stroke-opacity');
const RE_STROKE_W = mkAttrRe('stroke-width');
const RE_FONT_SIZE = mkAttrRe('font-size');
const RE_ANCHOR = mkAttrRe('text-anchor');
const RE_TEXT_CONTENT = /<text[^>]*>([^<]*)<\/text>/;

function getF(s, re, def = 0) { const m = re.exec(s); return m ? parseFloat(m[1]) : def; }
function getS(s, re, def = '') { const m = re.exec(s); return m ? m[1] : def; }

function parsePoints(s) {
    const nums = s.trim().split(/[\s,]+/).map(Number);
    const pts = [];
    for (let i = 0; i + 1 < nums.length; i += 2) pts.push({ x: nums[i], y: nums[i + 1] });
    return pts;
}

function getStyle(s) {
    return {
        fill: getS(s, RE_FILL, 'none'),
        fillOpacity: getF(s, RE_FILL_OP, 1),
        stroke: getS(s, RE_STROKE, 'none'),
        strokeOpacity: getF(s, RE_STROKE_OP, 1),
        strokeWidth: getF(s, RE_STROKE_W, 1),
    };
}

class SvgPrimitiveParser {
    constructor() {
        this._cache = new Map();
    }

    parse(svgStr) {
        let cmd = this._cache.get(svgStr);
        if (cmd !== undefined) return cmd;
        cmd = this._doParse(svgStr);
        // LRUライクなキャッシュ管理
        if (this._cache.size >= 20000) {
            const iter = this._cache.keys();
            for (let i = 0; i < 5000; i++) this._cache.delete(iter.next().value);
        }
        this._cache.set(svgStr, cmd);
        return cmd;
    }

    _doParse(s) {
        const t = s.trimStart();
        if (t.startsWith('<circle')) {
            return {
                type: 'circle',
                cx: getF(s, RE_CX), cy: getF(s, RE_CY), r: getF(s, RE_R),
                ...getStyle(s),
            };
        }
        if (t.startsWith('<line')) {
            return {
                type: 'line',
                x1: getF(s, RE_X1), y1: getF(s, RE_Y1),
                x2: getF(s, RE_X2), y2: getF(s, RE_Y2),
                ...getStyle(s),
            };
        }
        if (t.startsWith('<rect')) {
            return {
                type: 'rect',
                x: getF(s, RE_X), y: getF(s, RE_Y),
                w: getF(s, RE_W), h: getF(s, RE_H),
                ...getStyle(s),
            };
        }
        if (t.startsWith('<text')) {
            const xStr = getS(s, RE_X);
            const yStr = getS(s, RE_Y);
            const isPercent = xStr.includes('%') || yStr.includes('%');
            const xVal = parseFloat(xStr) || 0;
            const yVal = parseFloat(yStr) || 0;
            const textM = RE_TEXT_CONTENT.exec(s);
            return {
                type: 'text',
                x: isPercent ? xVal / 100 * VB_W + VB_X : xVal,
                y: isPercent ? yVal / 100 * VB_H + VB_Y : yVal,
                fill: getS(s, RE_FILL, 'white'),
                fillOpacity: getF(s, RE_FILL_OP, 1),
                fontSize: getF(s, RE_FONT_SIZE, 100),
                textAnchor: getS(s, RE_ANCHOR, 'start'),
                content: textM ? textM[1] : '',
            };
        }
        if (t.startsWith('<polyline')) {
            const style = getStyle(s);
            return {
                type: 'polyline',
                points: parsePoints(getS(s, RE_POINTS)),
                ...style,
                fill: 'none',
            };
        }
        if (t.startsWith('<polygon')) {
            return {
                type: 'polygon',
                points: parsePoints(getS(s, RE_POINTS)),
                ...getStyle(s),
            };
        }
        if (t.startsWith('<path')) {
            return {
                type: 'path',
                segments: parseSvgPath(getS(s, RE_D)),
                ...getStyle(s),
            };
        }
        return null;
    }
}

// ---- Canvas Renderer ----
class CanvasRenderer {
    constructor(canvas, viewer) {
        this.canvas = canvas;
        this.ctx = canvas.getContext('2d');
        this.viewer = viewer;
        this.needsRedraw = true;
        this._rafId = null;
        this._dpr = window.devicePixelRatio || 1;

        this._resizeObserver = new ResizeObserver(() => this._onResize());
        this._resizeObserver.observe(canvas.parentElement);
        this._onResize();
        this._startLoop();
    }

    _onResize() {
        const container = this.canvas.parentElement;
        const cssW = container.clientWidth || 1;
        const cssH = container.clientHeight || 1;
        this._dpr = window.devicePixelRatio || 1;
        this.canvas.width = Math.round(cssW * this._dpr);
        this.canvas.height = Math.round(cssH * this._dpr);
        this.canvas.style.width = cssW + 'px';
        this.canvas.style.height = cssH + 'px';
        this.invalidate();
    }

    invalidate() { this.needsRedraw = true; }

    _startLoop() {
        const loop = () => {
            if (this.needsRedraw) {
                this.needsRedraw = false;
                this._render();
            }
            this._rafId = requestAnimationFrame(loop);
        };
        this._rafId = requestAnimationFrame(loop);
    }

    stop() {
        if (this._rafId) { cancelAnimationFrame(this._rafId); this._rafId = null; }
        this._resizeObserver.disconnect();
    }

    // viewBox座標系→CSS座標へのビューポート変換パラメータを計算
    _getVP() {
        const dpr = this._dpr;
        const cssW = this.canvas.width / dpr;
        const cssH = this.canvas.height / dpr;
        const scaleX = cssW / VB_W;
        const scaleY = cssH / VB_H;
        const vs = Math.min(scaleX, scaleY); // xMidYMid meet
        const ox = (cssW - VB_W * vs) / 2;
        const oy = (cssH - VB_H * vs) / 2;
        return { dpr, cssW, cssH, vs, ox, oy };
    }

    _render() {
        const ctx = this.ctx;
        const vp = this._getVP();
        const v = this.viewer;

        ctx.setTransform(1, 0, 0, 1, 0, 0);
        ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);

        ctx.save();
        ctx.scale(vp.dpr, vp.dpr);           // HiDPIスケーリング
        ctx.translate(vp.ox, vp.oy);          // viewBoxのオフセット（letterbox）
        ctx.scale(vp.vs, vp.vs);              // viewBoxスケール
        ctx.translate(-VB_X, -VB_Y);          // viewBox原点シフト
        ctx.translate(v.panOffset.x, v.panOffset.y); // ユーザーパン
        ctx.scale(v.zoomLevel, v.zoomLevel);  // ユーザーズーム

        // フィールド背景
        ctx.fillStyle = '#1a5c1a';
        ctx.fillRect(VB_X, VB_Y, VB_W, VB_H);
        this._drawGrid(ctx);

        // SVGレイヤー描画
        const parser = v.parser;
        for (const [name, layer] of v.layerStore) {
            if (!v.visibleLayers.has(name)) continue;
            if (layer.dirty) {
                layer.commands = layer.primitives.map(p => parser.parse(p)).filter(Boolean);
                layer.dirty = false;
            }
            for (const cmd of layer.commands) {
                this._drawCmd(ctx, cmd);
            }
        }

        // ロボット移動モードのオーバーレイ
        if (v.moveMode && v.selectedRobotId !== null) {
            const robot = v.robotsOurs[v.selectedRobotId];
            if (robot) {
                ctx.save();
                ctx.strokeStyle = '#00ffff';
                ctx.lineWidth = 20;
                ctx.setLineDash([40, 20]);
                ctx.globalAlpha = 0.9;
                ctx.beginPath();
                // SVG座標系: x*1000, -y*1000（crane_visualizer_wrapperのtoSvgX/toSvgY変換に合わせる）
                ctx.arc(robot.x * 1000, -robot.y * 1000, ROBOT_HIT_RADIUS_M * 1000, 0, Math.PI * 2);
                ctx.stroke();
                ctx.restore();
            }
        }

        ctx.restore();

        // データなし表示
        if (v.layerStore.size === 0) {
            ctx.save();
            ctx.fillStyle = '#444';
            ctx.font = `${40 * vp.dpr}px sans-serif`;
            ctx.textAlign = 'center';
            ctx.textBaseline = 'middle';
            ctx.fillText('SVGデータなし - ROS topicを待機中...', this.canvas.width / 2, this.canvas.height / 2);
            ctx.restore();
        }
    }

    _drawGrid(ctx) {
        ctx.save();
        ctx.strokeStyle = '#2d8a2d';
        ctx.lineWidth = 15;
        ctx.globalAlpha = 0.3;
        ctx.beginPath();
        for (let x = VB_X; x <= -VB_X; x += 1000) {
            ctx.moveTo(x, VB_Y); ctx.lineTo(x, -VB_Y);
        }
        for (let y = VB_Y; y <= -VB_Y; y += 1000) {
            ctx.moveTo(VB_X, y); ctx.lineTo(-VB_X, y);
        }
        ctx.stroke();
        ctx.restore();
    }

    _fill(ctx, cmd) {
        if (cmd.fill && cmd.fill !== 'none') {
            ctx.globalAlpha = cmd.fillOpacity ?? 1;
            ctx.fillStyle = cmd.fill;
            ctx.fill();
        }
    }

    _stroke(ctx, cmd) {
        if (cmd.stroke && cmd.stroke !== 'none') {
            ctx.globalAlpha = cmd.strokeOpacity ?? 1;
            ctx.strokeStyle = cmd.stroke;
            ctx.lineWidth = cmd.strokeWidth ?? 1;
            ctx.stroke();
        }
    }

    _drawCmd(ctx, cmd) {
        ctx.save();
        ctx.globalAlpha = 1;
        switch (cmd.type) {
            case 'circle':
                ctx.beginPath();
                ctx.arc(cmd.cx, cmd.cy, Math.abs(cmd.r), 0, Math.PI * 2);
                this._fill(ctx, cmd);
                this._stroke(ctx, cmd);
                break;
            case 'line':
                ctx.beginPath();
                ctx.moveTo(cmd.x1, cmd.y1);
                ctx.lineTo(cmd.x2, cmd.y2);
                this._stroke(ctx, cmd);
                break;
            case 'rect':
                ctx.beginPath();
                ctx.rect(cmd.x, cmd.y, cmd.w, cmd.h);
                this._fill(ctx, cmd);
                this._stroke(ctx, cmd);
                break;
            case 'text': {
                const ALIGN = { start: 'left', middle: 'center', end: 'right' };
                ctx.fillStyle = cmd.fill || 'white';
                ctx.globalAlpha = cmd.fillOpacity ?? 1;
                ctx.font = `${cmd.fontSize}px sans-serif`;
                ctx.textAlign = ALIGN[cmd.textAnchor] || 'left';
                ctx.textBaseline = 'alphabetic';
                ctx.fillText(cmd.content, cmd.x, cmd.y);
                break;
            }
            case 'polyline':
                if (cmd.points.length < 2) break;
                ctx.beginPath();
                ctx.moveTo(cmd.points[0].x, cmd.points[0].y);
                for (let i = 1; i < cmd.points.length; i++) ctx.lineTo(cmd.points[i].x, cmd.points[i].y);
                this._stroke(ctx, cmd);
                break;
            case 'polygon':
                if (cmd.points.length < 2) break;
                ctx.beginPath();
                ctx.moveTo(cmd.points[0].x, cmd.points[0].y);
                for (let i = 1; i < cmd.points.length; i++) ctx.lineTo(cmd.points[i].x, cmd.points[i].y);
                ctx.closePath();
                this._fill(ctx, cmd);
                this._stroke(ctx, cmd);
                break;
            case 'path':
                this._drawPath(ctx, cmd);
                break;
        }
        ctx.restore();
    }

    _buildPath(ctx, segments) {
        let cx = 0, cy = 0;
        let lastC = null, lastQ = null;
        for (const { cmd: c, args: a } of segments) {
            switch (c) {
                case 'M': cx = a[0]; cy = a[1]; ctx.moveTo(cx, cy); lastC = lastQ = null; break;
                case 'L': cx = a[0]; cy = a[1]; ctx.lineTo(cx, cy); lastC = lastQ = null; break;
                case 'H': cx = a[0]; ctx.lineTo(cx, cy); lastC = lastQ = null; break;
                case 'V': cy = a[0]; ctx.lineTo(cx, cy); lastC = lastQ = null; break;
                case 'Z': ctx.closePath(); lastC = lastQ = null; break;
                case 'C':
                    ctx.bezierCurveTo(a[0], a[1], a[2], a[3], a[4], a[5]);
                    lastC = { x: a[2], y: a[3] }; cx = a[4]; cy = a[5]; lastQ = null; break;
                case 'S': {
                    const sx = lastC ? 2 * cx - lastC.x : cx;
                    const sy = lastC ? 2 * cy - lastC.y : cy;
                    ctx.bezierCurveTo(sx, sy, a[0], a[1], a[2], a[3]);
                    lastC = { x: a[0], y: a[1] }; cx = a[2]; cy = a[3]; lastQ = null; break;
                }
                case 'Q':
                    ctx.quadraticCurveTo(a[0], a[1], a[2], a[3]);
                    lastQ = { x: a[0], y: a[1] }; cx = a[2]; cy = a[3]; lastC = null; break;
                case 'T': {
                    const qx = lastQ ? 2 * cx - lastQ.x : cx;
                    const qy = lastQ ? 2 * cy - lastQ.y : cy;
                    ctx.quadraticCurveTo(qx, qy, a[0], a[1]);
                    lastQ = { x: qx, y: qy }; cx = a[0]; cy = a[1]; lastC = null; break;
                }
                case 'A':
                    // A rx ry x-rotation large-arc-flag sweep-flag x y
                    svgArcToCanvas(ctx, cx, cy, a[0], a[1], a[2], a[3], a[4], a[5], a[6]);
                    cx = a[5]; cy = a[6]; lastC = lastQ = null; break;
            }
        }
    }

    _drawPath(ctx, cmd) {
        ctx.beginPath();
        this._buildPath(ctx, cmd.segments);
        this._fill(ctx, cmd);
        this._stroke(ctx, cmd);
    }

    // クライアント座標(px) → フィールド座標(m)
    clientToFieldCoords(clientX, clientY) {
        const rect = this.canvas.getBoundingClientRect();
        const cssX = clientX - rect.left;
        const cssY = clientY - rect.top;
        const { vs, ox, oy } = this._getVP();
        const v = this.viewer;
        // 逆ビューポート変換
        const svgX = (cssX - ox) / vs + VB_X;
        const svgY = (cssY - oy) / vs + VB_Y;
        // 逆ユーザーパン/ズーム
        const mmX = (svgX - v.panOffset.x) / v.zoomLevel;
        const mmY = (svgY - v.panOffset.y) / v.zoomLevel;
        return { x: mmX / 1000, y: mmY / 1000 };
    }

    // ピクセルデルタ → SVG座標デルタ（パン操作用）
    pixelDeltaToSvgDelta(dxPx, dyPx) {
        const { vs } = this._getVP();
        return {
            dx: dxPx / vs / this.viewer.zoomLevel,
            dy: dyPx / vs / this.viewer.zoomLevel,
        };
    }
}

// ---- CraneViewer（メインクラス） ----
class CraneViewer {
    constructor() {
        this.websocket = null;
        // レイヤーストア: Map<layerName, {primitives, commands, dirty}>
        this.layerStore = new Map();
        this.visibleLayers = new Set();
        this.seenLayers = new Set();
        this.zoomLevel = 1.0;
        this.panOffset = { x: 0, y: 0 };
        this.isPanning = false;
        this.lastPanPoint = { x: 0, y: 0 };
        this.pendingUpdateBatch = [];
        this.flushTimer = null;
        this.flushIntervalMs = 50;

        this.robotsOurs = {};
        this.controlTargets = {};

        this.moveMode = false;
        this.selectedRobotId = null;
        this.isYellow = false;
        this.onPositiveHalf = false;

        this.robotUpdateTimer = null;
        this.parser = new SvgPrimitiveParser();
        this.renderer = null;

        this.init();
    }

    init() {
        const canvas = document.getElementById('field-canvas');
        if (canvas) this.renderer = new CanvasRenderer(canvas, this);
        this.setupWebSocket();
        this.setupEventListeners();
        this.updateConnectionStatus(false);
    }

    setupWebSocket() {
        const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
        const wsUrl = `${protocol}//${window.location.hostname}:8091`;
        this.websocket = new WebSocket(wsUrl);
        this.websocket.onopen = () => this.updateConnectionStatus(true);
        this.websocket.onmessage = (event) => {
            try { this.handleMessage(JSON.parse(event.data)); } catch (e) { console.error('メッセージ解析エラー:', e); }
        };
        this.websocket.onclose = () => {
            this.updateConnectionStatus(false);
            setTimeout(() => {
                if (!this.websocket || this.websocket.readyState === WebSocket.CLOSED) this.setupWebSocket();
            }, 3000);
        };
        this.websocket.onerror = () => this.updateConnectionStatus(false);
    }

    handleMessage(data) {
        switch (data.type) {
            case 'svg_data':       this.handleSvgData(data); break;
            case 'svg_update':     this.handleSvgUpdate(data); break;
            case 'world_model':    this.handleWorldModel(data); break;
            case 'control_targets': this.handleControlTargets(data); break;
            case 'robot_commands': this.handleRobotCommands(data); break;
            case 'game_info':      this.handleGameInfo(data); break;
        }
    }

    _registerLayer(name) {
        if (!this.seenLayers.has(name)) {
            this.seenLayers.add(name);
            this.visibleLayers.add(name);
        }
    }

    handleSvgData(data) {
        this.layerStore.clear();
        if (data.layers) {
            for (const layer of data.layers) {
                this._registerLayer(layer.layer);
                this.layerStore.set(layer.layer, {
                    primitives: [...layer.svg_primitives],
                    commands: [],
                    dirty: true,
                });
            }
        }
        this.renderer?.invalidate();
        this.updateLayerList();
        this.updateStats();
    }

    handleSvgUpdate(data) {
        if (Array.isArray(data.updates) && data.updates.length > 0) {
            this.pendingUpdateBatch.push(...data.updates);
        }
        this.scheduleFlushUpdates();
    }

    scheduleFlushUpdates() {
        if (this.flushTimer) return;
        this.flushTimer = setTimeout(() => {
            const batch = this.pendingUpdateBatch.splice(0);
            this.flushTimer = null;
            if (batch.length === 0) return;
            this.applySvgUpdates(this.coalesceLayerUpdates(batch));
            this.renderer?.invalidate();
            this.updateLayerList();
            this.updateStats();
        }, this.flushIntervalMs);
    }

    coalesceLayerUpdates(updates) {
        const byLayer = new Map();
        for (const upd of updates) {
            const layer = upd.layer;
            const op = (upd.operation || '').toLowerCase();
            const prim = Array.isArray(upd.svg_primitives) ? upd.svg_primitives : [];
            if (!layer || !op) continue;
            if (!byLayer.has(layer)) byLayer.set(layer, { operation: null, svg_primitives: [] });
            const entry = byLayer.get(layer);
            if (op === 'replace') {
                entry.operation = 'replace';
                entry.svg_primitives = [...prim];
            } else if (op === 'clear') {
                entry.operation = 'clear';
                entry.svg_primitives = [];
            } else if (op === 'append') {
                if (entry.operation === 'clear') {
                    entry.operation = 'replace';
                    entry.svg_primitives = [...prim];
                } else {
                    if (!entry.operation) entry.operation = 'append';
                    entry.svg_primitives.push(...prim);
                }
            }
        }
        return Array.from(byLayer.entries()).map(([layer, v]) => ({ layer, ...v }));
    }

    applySvgUpdates(updates) {
        if (!Array.isArray(updates) || updates.length === 0) return;
        for (const upd of updates) {
            const op = (upd.operation || '').toLowerCase();
            const prims = Array.isArray(upd.svg_primitives) ? upd.svg_primitives : [];
            const existing = this.layerStore.get(upd.layer);
            if (op === 'replace') {
                this._registerLayer(upd.layer);
                this.layerStore.set(upd.layer, { primitives: [...prims], commands: [], dirty: true });
            } else if (op === 'append') {
                if (existing) {
                    existing.primitives.push(...prims);
                    existing.dirty = true;
                } else {
                    this._registerLayer(upd.layer);
                    this.layerStore.set(upd.layer, { primitives: [...prims], commands: [], dirty: true });
                }
            } else if (op === 'clear') {
                if (existing) {
                    existing.primitives = []; existing.commands = []; existing.dirty = false;
                } else {
                    this.layerStore.set(upd.layer, { primitives: [], commands: [], dirty: false });
                }
            }
        }
    }

    handleWorldModel(data) {
        if (data.is_yellow !== undefined) this.isYellow = data.is_yellow;
        if (data.on_positive_half !== undefined) this.onPositiveHalf = data.on_positive_half;
        if (data.robots_ours) {
            this.robotsOurs = {};
            for (const robot of data.robots_ours) this.robotsOurs[robot.id] = robot;
        }
        this.scheduleRobotUpdate();
        if (this.moveMode && this.selectedRobotId !== null) this.renderer?.invalidate();
    }

    handleControlTargets(data) {
        if (data.commands) {
            this.controlTargets = {};
            for (const cmd of data.commands) this.controlTargets[cmd.robot_id] = cmd;
        }
        this.scheduleRobotUpdate();
    }

    handleRobotCommands(data) {
        if (data.commands && Object.keys(this.controlTargets).length === 0) {
            for (const cmd of data.commands) this.controlTargets[cmd.robot_id] = cmd;
            this.scheduleRobotUpdate();
        }
    }

    handleGameInfo(data) {
        const set = (id, val) => { const el = document.getElementById(id); if (el) el.textContent = val; };
        set('score-our', data.our_score ?? 0);
        set('score-their', data.their_score ?? 0);
        set('play-situation', data.play_situation || '--');
        set('game-stage', data.game_stage || '--');
    }

    scheduleRobotUpdate() {
        if (this.robotUpdateTimer) return;
        this.robotUpdateTimer = setTimeout(() => {
            this.robotUpdateTimer = null;
            this.renderRobotCards();
        }, 500);
    }

    updateConnectionStatus(connected) {
        const dot = document.getElementById('connection-dot');
        const label = document.getElementById('connection-label');
        if (dot) dot.classList.toggle('connected', connected);
        if (label) label.textContent = connected ? 'connected' : 'disconnected';
    }

    renderRobotCards() {
        const container = document.getElementById('robot-cards-grid');
        if (!container) return;
        const ids = Object.keys(this.robotsOurs).map(Number).sort((a, b) => a - b);
        if (ids.length === 0) {
            container.innerHTML = '<div style="font-size:0.7rem;color:var(--md-sys-color-on-surface-variant);text-align:center;grid-column:span 4;padding:8px;">No robot data</div>';
            return;
        }
        container.innerHTML = '';
        for (const id of ids) {
            const cmd = this.controlTargets[id];
            const modeName = cmd ? (CONTROL_MODE_SHORT[cmd.control_mode] ?? `M${cmd.control_mode}`) : '--';
            const plannerName = cmd?.planner_name || '--';
            const card = document.createElement('div');
            card.className = 'robot-card';
            card.innerHTML = `<div class="robot-id">${id}</div><div class="robot-mode">${modeName}</div><div class="robot-planner" title="${plannerName}">${plannerName.substring(0, 6)}</div>`;
            card.addEventListener('click', () => this.showRobotDetail(id));
            container.appendChild(card);
        }
    }

    showRobotDetail(id) {
        const robot = this.robotsOurs[id];
        const cmd = this.controlTargets[id];
        if (!robot) return;
        const modal = document.getElementById('robot-detail-modal');
        const scrim = document.getElementById('robot-detail-scrim');
        if (!modal) return;
        const modeName = cmd ? (CONTROL_MODE_LONG[cmd.control_mode] ?? `MODE_${cmd.control_mode}`) : '--';
        document.getElementById('robot-detail-title').textContent = `Robot ${id}`;
        let targetHtml = '';
        if (cmd?.position_target_mode) {
            targetHtml = `<tr><td>Target Pos</td><td>(${cmd.position_target_mode.target_x?.toFixed(3)}, ${cmd.position_target_mode.target_y?.toFixed(3)})</td></tr>`;
        } else if (cmd?.simple_velocity_target_mode) {
            targetHtml = `<tr><td>Target Vel</td><td>(${cmd.simple_velocity_target_mode.target_vx?.toFixed(3)}, ${cmd.simple_velocity_target_mode.target_vy?.toFixed(3)})</td></tr>`;
        }
        document.getElementById('robot-detail-body').innerHTML = `
            <table class="m3-data-table m3-data-table--borderless m3-mb-sm"><tbody>
                <tr><td style="width:6em">Control Mode</td><td>${modeName}</td></tr>
                <tr><td>Planner</td><td>${cmd?.planner_name || '--'}</td></tr>
                <tr><td>Est. X</td><td>${robot.x?.toFixed(3)} m</td></tr>
                <tr><td>Est. Y</td><td>${robot.y?.toFixed(3)} m</td></tr>
                <tr><td>Est. θ</td><td>${robot.theta?.toFixed(3)} rad</td></tr>
                <tr><td>Vel Vx</td><td>${robot.vx?.toFixed(3)} m/s</td></tr>
                <tr><td>Vel Vy</td><td>${robot.vy?.toFixed(3)} m/s</td></tr>
                <tr><td>ω</td><td>${robot.omega?.toFixed(3)} rad/s</td></tr>
                <tr><td>Vision X</td><td>${robot.vision_x?.toFixed(3) ?? '--'}</td></tr>
                <tr><td>Vision Y</td><td>${robot.vision_y?.toFixed(3) ?? '--'}</td></tr>
                ${targetHtml}
                <tr><td>Target θ</td><td>${cmd?.target_theta?.toFixed(3) ?? '--'}</td></tr>
            </tbody></table>
            <a href="/robot_telemetry.html?id=${id}" class="m3-btn m3-btn--filled m3-btn--sm m3-w-full">
                <span class="material-symbols-outlined icon-sm">show_chart</span> Open Telemetry
            </a>
        `;
        modal.classList.add('open');
        if (scrim) scrim.classList.add('open');
        window.__closeRobotDialog = () => {
            modal.classList.remove('open');
            if (scrim) scrim.classList.remove('open');
        };
        const onKey = (e) => {
            if (e.key === 'Escape') {
                window.__closeRobotDialog();
                document.removeEventListener('keydown', onKey);
            }
        };
        document.addEventListener('keydown', onKey);
    }

    updateLayerList() {
        const container = document.getElementById('layer-list');
        if (!container) return;
        if (this.layerStore.size === 0) {
            container.innerHTML = '<div style="font-size:0.7rem;color:var(--md-sys-color-on-surface-variant);text-align:center;padding:6px;">No layers</div>';
            return;
        }
        container.innerHTML = '';
        for (const [name, layer] of this.layerStore) {
            const item = document.createElement('div');
            item.className = 'layer-item';
            item.innerHTML = `
                <input type="checkbox" class="m3-checkbox layer-cb" id="layer-${name}"
                       data-layer="${name}" ${this.visibleLayers.has(name) ? 'checked' : ''}>
                <label for="layer-${name}">${name}</label>
                <span class="m3-ms-auto m3-text-on-surface-variant" style="font-size:0.65rem">${layer.primitives.length}</span>
            `;
            container.appendChild(item);
        }
        container.querySelectorAll('.layer-cb').forEach(cb => {
            cb.addEventListener('change', (e) => {
                const name = e.target.dataset.layer;
                if (e.target.checked) this.visibleLayers.add(name);
                else this.visibleLayers.delete(name);
                this.renderer?.invalidate();
            });
        });
    }

    updateStats() {
        const layerEl = document.getElementById('layer-count');
        const primEl = document.getElementById('prim-count');
        if (layerEl) layerEl.textContent = this.layerStore.size;
        if (primEl) {
            let total = 0;
            for (const layer of this.layerStore.values()) total += layer.primitives.length;
            primEl.textContent = total;
        }
    }

    clientToFieldCoords(clientX, clientY) {
        return this.renderer ? this.renderer.clientToFieldCoords(clientX, clientY) : { x: 0, y: 0 };
    }

    findRobotAtPosition(fieldX, fieldY) {
        let closest = null;
        let minDist = ROBOT_HIT_RADIUS_M;
        for (const [id, robot] of Object.entries(this.robotsOurs)) {
            const dx = robot.x - fieldX;
            const dy = robot.y - fieldY;
            const dist = Math.sqrt(dx * dx + dy * dy);
            if (dist < minDist) { minDist = dist; closest = Number(id); }
        }
        return closest;
    }

    activateMoveMode() {
        this.moveMode = true;
        document.getElementById('btn-move-mode')?.classList.add('active');
        if (this.websocket?.readyState === WebSocket.OPEN) {
            this.websocket.send(JSON.stringify({ type: 'activate_move_mode' }));
        }
    }

    deactivateMoveMode() {
        this.moveMode = false;
        this.selectedRobotId = null;
        document.getElementById('btn-move-mode')?.classList.remove('active');
        const label = document.getElementById('selected-robot-label');
        if (label) label.textContent = '--';
        this.renderer?.invalidate();
    }

    sendMoveCommand(robotId, targetX, targetY) {
        const robot = this.robotsOurs[robotId];
        if (!robot || this.websocket?.readyState !== WebSocket.OPEN) return;
        this.websocket.send(JSON.stringify({
            type: 'move_robot', robot_id: robotId,
            target_x: targetX, target_y: targetY, target_theta: robot.theta ?? 0,
        }));
    }

    setupEventListeners() {
        document.getElementById('btn-select-all')?.addEventListener('click', () => {
            for (const name of this.layerStore.keys()) this.visibleLayers.add(name);
            this.updateLayerList();
            this.renderer?.invalidate();
        });
        document.getElementById('btn-deselect-all')?.addEventListener('click', () => {
            this.visibleLayers.clear();
            this.updateLayerList();
            this.renderer?.invalidate();
        });
        document.getElementById('btn-move-mode')?.addEventListener('click', () => {
            this.moveMode ? this.deactivateMoveMode() : this.activateMoveMode();
        });
        document.getElementById('btn-zoom-in')?.addEventListener('click', () => {
            this.zoomLevel = Math.min(this.zoomLevel * 1.2, 5.0);
            this.renderer?.invalidate();
        });
        document.getElementById('btn-zoom-out')?.addEventListener('click', () => {
            this.zoomLevel = Math.max(this.zoomLevel / 1.2, 0.1);
            this.renderer?.invalidate();
        });
        document.getElementById('btn-zoom-reset')?.addEventListener('click', () => {
            this.zoomLevel = 1.0;
            this.panOffset = { x: 0, y: 0 };
            this.renderer?.invalidate();
        });

        const canvas = document.getElementById('field-canvas');
        if (!canvas) return;

        canvas.addEventListener('wheel', (e) => {
            e.preventDefault();
            const factor = e.deltaY < 0 ? 1.1 : 0.9;
            this.zoomLevel = Math.min(Math.max(this.zoomLevel * factor, 0.1), 5.0);
            this.renderer?.invalidate();
        }, { passive: false });

        canvas.addEventListener('mousedown', (e) => {
            if (e.button !== 0) return;
            if (e.shiftKey && this.moveMode) {
                const fp = this.clientToFieldCoords(e.clientX, e.clientY);
                const hitId = this.findRobotAtPosition(fp.x, fp.y);
                if (hitId !== null) {
                    this.selectedRobotId = hitId;
                    const label = document.getElementById('selected-robot-label');
                    if (label) label.textContent = hitId;
                    this.renderer?.invalidate();
                } else if (this.selectedRobotId !== null) {
                    this.sendMoveCommand(this.selectedRobotId, fp.x, fp.y);
                }
            } else {
                this.isPanning = true;
                this.lastPanPoint = { x: e.clientX, y: e.clientY };
                canvas.style.cursor = 'grabbing';
            }
        });

        canvas.addEventListener('mousemove', (e) => {
            if (!this.isPanning) return;
            if (this.renderer) {
                const d = this.renderer.pixelDeltaToSvgDelta(
                    e.clientX - this.lastPanPoint.x,
                    e.clientY - this.lastPanPoint.y
                );
                this.panOffset.x += d.dx;
                this.panOffset.y += d.dy;
            }
            this.lastPanPoint = { x: e.clientX, y: e.clientY };
            this.renderer?.invalidate();
        });

        canvas.addEventListener('mouseup', () => {
            this.isPanning = false;
            canvas.style.cursor = 'grab';
        });
        canvas.addEventListener('mouseleave', () => {
            this.isPanning = false;
            canvas.style.cursor = 'default';
        });
        canvas.style.cursor = 'grab';
    }
}

document.addEventListener('DOMContentLoaded', () => {
    window.craneViewer = new CraneViewer();
});
