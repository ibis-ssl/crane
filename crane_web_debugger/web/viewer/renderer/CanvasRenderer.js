import { ROBOT_HIT_RADIUS_M, BALL_HIT_RADIUS_M } from './constants.js';
import { svgArcToCanvas } from './SvgPathUtils.js';

export class CanvasRenderer {
    constructor(canvas, viewer, fieldLayer, themeTokens) {
        this.canvas = canvas;
        this.ctx = canvas.getContext('2d');
        this.viewer = viewer;
        this.fieldLayer = fieldLayer;
        this.themeTokens = themeTokens;
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

    // viewBox座標系 → CSS座標へのビューポート変換パラメータを計算
    _getVP() {
        const fl = this.fieldLayer;
        const dpr = this._dpr;
        const cssW = this.canvas.width / dpr;
        const cssH = this.canvas.height / dpr;
        const scaleX = cssW / fl.vbW;
        const scaleY = cssH / fl.vbH;
        const vs = Math.min(scaleX, scaleY);
        const ox = (cssW - fl.vbW * vs) / 2;
        const oy = (cssH - fl.vbH * vs) / 2;
        return { dpr, cssW, cssH, vs, ox, oy };
    }

    _render() {
        const ctx = this.ctx;
        const vp = this._getVP();
        const v = this.viewer;
        const fl = this.fieldLayer;
        const tokens = this.themeTokens.get() ?? {};

        ctx.setTransform(1, 0, 0, 1, 0, 0);
        ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);

        ctx.save();
        ctx.scale(vp.dpr, vp.dpr);
        ctx.translate(vp.ox, vp.oy);
        ctx.scale(vp.vs, vp.vs);
        ctx.translate(-fl.vbX, -fl.vbY);
        ctx.translate(v.panOffset.x, v.panOffset.y);
        ctx.scale(v.zoomLevel, v.zoomLevel);

        // フィールド背景 + グリッド (M3トークン使用)
        fl.drawBackground(ctx, tokens);
        fl.drawGrid(ctx, tokens);

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

        // ロボット移動モードのオーバーレイ (M3トークン使用)
        if (v.moveMode && v.selectedRobotId !== null) {
            const robot = v.robotsOurs[v.selectedRobotId];
            if (robot) {
                ctx.save();
                ctx.strokeStyle = tokens.moveOverlay ?? '#D0BCFF';
                ctx.lineWidth = 20;
                ctx.setLineDash([40, 20]);
                ctx.globalAlpha = 0.9;
                ctx.beginPath();
                ctx.arc(robot.x * 1000, -robot.y * 1000, ROBOT_HIT_RADIUS_M * 1000, 0, Math.PI * 2);
                ctx.stroke();
                ctx.restore();
            }
        }

        // Sim edit mode オーバーレイ (M3トークン使用)
        if (v.simEditMode && v.simSelectedObj) {
            const obj = v.simSelectedObj;
            ctx.save();
            ctx.globalAlpha = 0.85;
            ctx.lineWidth = 22;
            ctx.setLineDash([45, 25]);
            if (obj.type === 'ball') {
                ctx.strokeStyle = tokens.simOverlayBall ?? '#FFA726';
                ctx.beginPath();
                ctx.arc(v.ballPos.x * 1000, -v.ballPos.y * 1000, BALL_HIT_RADIUS_M * 900, 0, Math.PI * 2);
                ctx.stroke();
            } else {
                const source = (obj.yellow === v.isYellow) ? v.robotsOurs : v.robotsTheirs;
                const robot = source[obj.id];
                if (robot) {
                    ctx.strokeStyle = tokens.simOverlayRobot ?? '#D0BCFF';
                    ctx.beginPath();
                    ctx.arc(robot.x * 1000, -robot.y * 1000, ROBOT_HIT_RADIUS_M * 1000, 0, Math.PI * 2);
                    ctx.stroke();
                }
            }
            ctx.restore();
        }

        ctx.restore();

        // データなし表示 (M3トークン使用)
        if (v.layerStore.size === 0) {
            ctx.save();
            ctx.fillStyle = tokens.noDataText ?? '#8C929A';
            ctx.font = `${40 * vp.dpr}px sans-serif`;
            ctx.textAlign = 'center';
            ctx.textBaseline = 'middle';
            ctx.fillText('SVGデータなし - ROS topicを待機中...', this.canvas.width / 2, this.canvas.height / 2);
            ctx.restore();
        }
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
        const fl = this.fieldLayer;
        const v = this.viewer;
        const svgX = (cssX - ox) / vs + fl.vbX;
        const svgY = (cssY - oy) / vs + fl.vbY;
        const mmX = (svgX - v.panOffset.x) / v.zoomLevel;
        const mmY = (svgY - v.panOffset.y) / v.zoomLevel;
        return { x: mmX / 1000, y: -mmY / 1000 };
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
