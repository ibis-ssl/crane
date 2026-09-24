// SVG レイヤーの保持と、レイヤードロワーの一覧・件数表示。
//
// svg_update は 50ms ぶんまとめてから適用する。1 メッセージごとに再描画すると
// レイヤーの多い試合で描画が追いつかない。まとめる際は同一レイヤーへの
// replace / clear / append を畳んでから渡す（coalesce）。

import { applyLayerUpdate } from '../replay/RingBuffer.js';

const FLUSH_INTERVAL_MS = 50;

export class LayerStore {
    constructor(onChange) {
        this.layers = new Map();       // name -> { primitives, commands, dirty }
        this.visible = new Set();
        this.seen = new Set();
        this._pending = [];
        this._flushTimer = null;
        this._onChange = onChange;     // 再描画と表示更新のトリガ
    }

    get size() { return this.layers.size; }

    get primitiveCount() {
        let total = 0;
        for (const layer of this.layers.values()) total += layer.primitives.length;
        return total;
    }

    _register(name) {
        if (this.seen.has(name)) return;
        this.seen.add(name);
        this.visible.add(name);
    }

    // svg_data: キーフレーム。全レイヤーを置き換える
    replaceAll(layers) {
        this.layers.clear();
        for (const layer of layers ?? []) {
            this._register(layer.layer);
            this.layers.set(layer.layer, {
                primitives: [...layer.svg_primitives],
                commands: [],
                dirty: true,
            });
        }
    }

    // svg_update: 差分。まとめてから適用する
    queueUpdates(updates) {
        if (Array.isArray(updates) && updates.length > 0) this._pending.push(...updates);
        if (this._flushTimer) return;
        this._flushTimer = setTimeout(() => {
            const batch = this._pending.splice(0);
            this._flushTimer = null;
            if (batch.length === 0) return;
            this._apply(this._coalesce(batch));
            this._onChange?.();
        }, FLUSH_INTERVAL_MS);
    }

    markAllDirty() {
        for (const layer of this.layers.values()) layer.dirty = true;
    }

    selectAll() { for (const name of this.layers.keys()) this.visible.add(name); }

    deselectAll() { this.visible.clear(); }

    _coalesce(updates) {
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
                entry.svg_primitives = prim;
            } else if (op === 'clear') {
                entry.operation = 'clear';
                entry.svg_primitives = [];
            } else if (op === 'append') {
                if (entry.operation === 'clear') {
                    // clear の直後の append は「この内容で置き換える」と等価
                    entry.operation = 'replace';
                    entry.svg_primitives = prim;
                } else {
                    if (!entry.operation) entry.operation = 'append';
                    entry.svg_primitives.push(...prim);
                }
            }
        }
        return Array.from(byLayer.entries()).map(([layer, v]) => ({ layer, ...v }));
    }

    _apply(updates) {
        if (!Array.isArray(updates) || updates.length === 0) return;
        for (const upd of updates) {
            const op = (upd.operation || '').toLowerCase();
            if (op === 'replace' || (op === 'append' && !this.layers.has(upd.layer))) {
                this._register(upd.layer);
            }
            applyLayerUpdate(this.layers, upd);
        }
    }

    // ===== 表示 =====

    renderList(onToggle) {
        const container = document.getElementById('layer-list');
        if (!container) return;
        if (this.layers.size === 0) {
            container.innerHTML = '<div style="font-size:0.7rem;color:var(--md-sys-color-on-surface-variant);text-align:center;padding:6px;">No layers</div>';
            return;
        }
        container.innerHTML = '';
        for (const [name, layer] of this.layers) {
            const item = document.createElement('div');
            item.className = 'layer-item';
            item.innerHTML = `
                <input type="checkbox" class="m3-checkbox layer-cb" id="layer-${name}"
                       data-layer="${name}" ${this.visible.has(name) ? 'checked' : ''}>
                <label for="layer-${name}">${name}</label>
                <span class="m3-ms-auto m3-text-on-surface-variant" style="font-size:0.65rem">${layer.primitives.length}</span>
            `;
            container.appendChild(item);
        }
        container.querySelectorAll('.layer-cb').forEach(cb => {
            cb.addEventListener('change', (e) => {
                const name = e.target.dataset.layer;
                if (e.target.checked) this.visible.add(name);
                else this.visible.delete(name);
                onToggle?.();
            });
        });
    }

    renderStats() {
        const layerEl = document.getElementById('layer-count');
        const primEl = document.getElementById('prim-count');
        if (layerEl) layerEl.textContent = this.layers.size;
        if (primEl) primEl.textContent = this.primitiveCount;
    }
}
