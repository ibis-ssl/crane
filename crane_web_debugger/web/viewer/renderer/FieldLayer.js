import { FIELD_BOUNDARY_DEFAULT_M, DEFAULT_FIELD_LENGTH_M, DEFAULT_FIELD_WIDTH_M } from './constants.js';

// フィールドの viewBox 管理と turf/grid 描画を担う。
// world_model.field_info を受け取り、div-A/div-B に動的対応する。
export class FieldLayer {
    constructor() {
        this._lengthM = DEFAULT_FIELD_LENGTH_M;
        this._widthM = DEFAULT_FIELD_WIDTH_M;
        this._boundaryM = FIELD_BOUNDARY_DEFAULT_M;
    }

    // field_info が変化した場合 true を返す
    updateFromFieldInfo(info) {
        if (!info) return false;
        const newL = (info.length > 0) ? info.length : this._lengthM;
        const newW = (info.width > 0) ? info.width : this._widthM;
        const changed = (newL !== this._lengthM || newW !== this._widthM);
        this._lengthM = newL;
        this._widthM = newW;
        return changed;
    }

    get vbX() { return -(this._lengthM / 2 + this._boundaryM) * 1000; }
    get vbY() { return -(this._widthM / 2 + this._boundaryM) * 1000; }
    get vbW() { return (this._lengthM + 2 * this._boundaryM) * 1000; }
    get vbH() { return (this._widthM + 2 * this._boundaryM) * 1000; }

    drawBackground(ctx, tokens) {
        ctx.fillStyle = tokens.fieldTurf;
        ctx.fillRect(this.vbX, this.vbY, this.vbW, this.vbH);
    }

    drawGrid(ctx, tokens) {
        ctx.save();
        ctx.strokeStyle = tokens.fieldGrid;
        ctx.lineWidth = 15;
        ctx.globalAlpha = 0.3;
        ctx.beginPath();
        const x0 = Math.ceil(this.vbX / 1000) * 1000;
        const xEnd = this.vbX + this.vbW;
        for (let x = x0; x <= xEnd + 1; x += 1000) {
            ctx.moveTo(x, this.vbY); ctx.lineTo(x, this.vbY + this.vbH);
        }
        const y0 = Math.ceil(this.vbY / 1000) * 1000;
        const yEnd = this.vbY + this.vbH;
        for (let y = y0; y <= yEnd + 1; y += 1000) {
            ctx.moveTo(this.vbX, y); ctx.lineTo(xEnd, y);
        }
        ctx.stroke();
        ctx.restore();
    }
}
