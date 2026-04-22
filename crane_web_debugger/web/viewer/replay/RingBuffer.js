// Two-tier ring buffer: keyframe(1Hz) + delta events
// 60 秒ウィンドウ。seek(tsMs) で任意フレームを復元。

const DEFAULT_WINDOW_MS = 60_000;

export function indexBy(arr, keyName) {
    const map = {};
    for (const item of arr) map[item[keyName]] = item;
    return map;
}

export function applyLayerUpdate(layerStore, upd) {
    const op = (upd.operation || '').toLowerCase();
    const prims = Array.isArray(upd.svg_primitives) ? upd.svg_primitives : [];
    const existing = layerStore.get(upd.layer);
    if (op === 'replace') {
        layerStore.set(upd.layer, { primitives: [...prims], commands: [], dirty: true });
    } else if (op === 'append') {
        if (existing) { existing.primitives.push(...prims); existing.dirty = true; }
        else layerStore.set(upd.layer, { primitives: [...prims], commands: [], dirty: true });
    } else if (op === 'clear') {
        if (existing) { existing.primitives = []; existing.commands = []; existing.dirty = false; }
        else layerStore.set(upd.layer, { primitives: [], commands: [], dirty: false });
    }
}

export class RingBuffer {
    constructor(windowMs = DEFAULT_WINDOW_MS) {
        this.windowMs = windowMs;
        this._keyframes = []; // { tsMs, state: {layerStore, robots, ball, controlTargets} }
        this._deltas = [];    // { tsMs, type, data }
        this._keyframeInterval = 1000;
        this._lastKeyframeTs = -Infinity;
    }

    // svg_data または world_model を受信したときに呼ぶ
    // state: { layerStore: Map, robotsOurs: {}, robotsTheirs: {}, ball: {}, controlTargets: {} }
    addKeyframe(tsMs, state) {
        if (tsMs - this._lastKeyframeTs < this._keyframeInterval) return;
        this._lastKeyframeTs = tsMs;
        this._keyframes.push({ tsMs, state: this._cloneState(state) });
        this._prune(tsMs);
    }

    addDelta(tsMs, type, data) {
        this._deltas.push({ tsMs, type, data });
        // デルタは prune をキーフレームタイミングに依存させないため独立に刈る
        this._pruneDeltasOlderThan(tsMs - this.windowMs);
    }

    seek(tsMs) {
        // 直前のキーフレームを探す
        let kf = null;
        for (let i = this._keyframes.length - 1; i >= 0; i--) {
            if (this._keyframes[i].tsMs <= tsMs) { kf = this._keyframes[i]; break; }
        }
        if (!kf) return null;

        const state = this._cloneState(kf.state);

        // キーフレーム以降 tsMs 以前のデルタを適用
        for (const d of this._deltas) {
            if (d.tsMs <= kf.tsMs) continue;
            if (d.tsMs > tsMs) break;
            this._applyDelta(state, d);
        }
        return state;
    }

    // バッファ内の最古・最新のタイムスタンプ
    get oldestMs() { return this._keyframes[0]?.tsMs ?? null; }
    get newestMs() { return this._keyframes[this._keyframes.length - 1]?.tsMs ?? null; }

    _applyDelta(state, d) {
        switch (d.type) {
            case 'world_model':
                if (d.data.robots_ours) state.robotsOurs = indexBy(d.data.robots_ours, 'id');
                if (d.data.robots_theirs) state.robotsTheirs = indexBy(d.data.robots_theirs, 'id');
                if (d.data.ball) state.ball = d.data.ball;
                break;
            case 'control_targets':
                if (d.data.commands) state.controlTargets = indexBy(d.data.commands, 'robot_id');
                break;
            case 'svg_update':
                if (!Array.isArray(d.data.updates)) break;
                for (const upd of d.data.updates) applyLayerUpdate(state.layerStore, upd);
                break;
        }
    }

    _cloneState(state) {
        const layerStore = new Map();
        if (state.layerStore) {
            for (const [k, v] of state.layerStore) {
                layerStore.set(k, { primitives: [...v.primitives], commands: [], dirty: true });
            }
        }
        return {
            layerStore,
            robotsOurs: { ...state.robotsOurs },
            robotsTheirs: { ...state.robotsTheirs },
            ball: { ...state.ball },
            controlTargets: { ...state.controlTargets },
        };
    }

    _prune(nowMs) {
        const cutoff = nowMs - this.windowMs;
        while (this._keyframes.length > 1 && this._keyframes[0].tsMs < cutoff) {
            this._keyframes.shift();
        }
    }

    _pruneDeltasOlderThan(cutoff) {
        while (this._deltas.length > 0 && this._deltas[0].tsMs < cutoff) {
            this._deltas.shift();
        }
    }
}
