// 60×20px 純 Canvas2D sparkline。最大 5 系列。DPR 対応。
export class Sparkline {
    constructor(container, options = {}) {
        this._w = options.width ?? 80;
        this._h = options.height ?? 24;
        this._series = [];

        this._canvas = document.createElement('canvas');
        this._canvas.style.width = this._w + 'px';
        this._canvas.style.height = this._h + 'px';
        this._canvas.style.display = 'block';
        const dpr = window.devicePixelRatio || 1;
        this._canvas.width = Math.round(this._w * dpr);
        this._canvas.height = Math.round(this._h * dpr);
        this._ctx = this._canvas.getContext('2d');
        this._ctx.scale(dpr, dpr);
        this._dpr = dpr;

        container.appendChild(this._canvas);
    }

    // series: [{ color: '#hex', data: Float32Array }]
    setSeries(series) {
        this._series = series.slice(0, 5);
    }

    render(tokens) {
        const ctx = this._ctx;
        const W = this._w, H = this._h;
        ctx.clearRect(0, 0, W, H);

        // 背景
        ctx.fillStyle = tokens?.bgDim ?? 'rgba(0,0,0,0.3)';
        ctx.fillRect(0, 0, W, H);

        if (this._series.length === 0) return;

        // 全系列の min/max
        let minVal = Infinity, maxVal = -Infinity;
        for (const s of this._series) {
            for (const v of s.data) {
                if (v < minVal) minVal = v;
                if (v > maxVal) maxVal = v;
            }
        }
        if (minVal === maxVal) { maxVal = minVal + 1; }

        const range = maxVal - minVal;
        const pad = 2;
        const drawH = H - pad * 2;
        const toY = (v) => pad + drawH * (1 - (v - minVal) / range);

        // ゼロ軸 (正負跨ぎ時のみ)
        if (minVal < 0 && maxVal > 0) {
            ctx.save();
            ctx.strokeStyle = tokens?.hudTextMuted ?? 'rgba(200,200,200,0.2)';
            ctx.lineWidth = 0.5;
            const zy = toY(0);
            ctx.beginPath();
            ctx.moveTo(0, zy);
            ctx.lineTo(W, zy);
            ctx.stroke();
            ctx.restore();
        }

        // 系列描画
        for (const s of this._series) {
            const data = s.data;
            if (!data || data.length < 2) continue;
            const n = data.length;
            const dx = W / (n - 1);

            ctx.save();
            ctx.strokeStyle = s.color;
            ctx.lineWidth = 1.5;
            ctx.lineJoin = 'round';
            ctx.beginPath();
            ctx.moveTo(0, toY(data[0]));
            for (let i = 1; i < n; i++) {
                ctx.lineTo(i * dx, toY(data[i]));
            }
            ctx.stroke();

            // 末端マーカ
            ctx.fillStyle = s.color;
            ctx.beginPath();
            ctx.arc((n - 1) * dx, toY(data[n - 1]), 2.5, 0, Math.PI * 2);
            ctx.fill();
            ctx.restore();
        }
    }
}

// 10Hz × 60s = 600 サンプルのリングバッファ
export class MetricRing {
    constructor(capacity = 600) {
        this._cap = capacity;
        this._data = new Float32Array(capacity);
        this._head = 0;
        this._len = 0;
    }

    push(value) {
        this._data[this._head] = value;
        this._head = (this._head + 1) % this._cap;
        if (this._len < this._cap) this._len++;
    }

    // 最新 n サンプルを Float32Array で返す
    latest(n) {
        const count = Math.min(n, this._len);
        const out = new Float32Array(count);
        for (let i = 0; i < count; i++) {
            const idx = (this._head - count + i + this._cap) % this._cap;
            out[i] = this._data[idx];
        }
        return out;
    }
}
