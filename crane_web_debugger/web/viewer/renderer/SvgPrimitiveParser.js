import { parseSvgPath } from './SvgPathUtils.js';

// ---- SVG属性regex (crane_visualizer_wrapper の固定フォーマット向けに最適化) ----
function mkAttrRe(name) { return new RegExp(`${name}="([^"]*)"`); }

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

export class SvgPrimitiveParser {
    constructor() {
        this._cache = new Map();
        this._vbX = -6000;
        this._vbY = -4500;
        this._vbW = 12000;
        this._vbH = 9000;
    }

    // FieldLayer の viewBox が変化した時に呼ぶ。% テキストのキャッシュを無効化するため全クリア。
    setViewBox(vbX, vbY, vbW, vbH) {
        if (this._vbX !== vbX || this._vbY !== vbY || this._vbW !== vbW || this._vbH !== vbH) {
            this._cache.clear();
            this._vbX = vbX; this._vbY = vbY; this._vbW = vbW; this._vbH = vbH;
        }
    }

    parse(svgStr) {
        let cmd = this._cache.get(svgStr);
        if (cmd !== undefined) return cmd;
        cmd = this._doParse(svgStr);
        if (this._cache.size >= 40000) {
            const iter = this._cache.keys();
            for (let i = 0; i < 10000; i++) this._cache.delete(iter.next().value);
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
                x: isPercent ? xVal / 100 * this._vbW + this._vbX : xVal,
                y: isPercent ? yVal / 100 * this._vbH + this._vbY : yVal,
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
