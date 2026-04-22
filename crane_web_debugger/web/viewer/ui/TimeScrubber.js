// 下ドック用タイムスクラバー。
// viewer から RingBuffer を受け取り、playback 状態を管理。
export class TimeScrubber {
    constructor(container, ringBuffer, viewer) {
        this._ring = ringBuffer;
        this._viewer = viewer;
        this._playing = false;
        this._speed = 1;
        this._indexMs = null; // null = live
        this._rafId = null;
        this._lastRafTs = null;

        this._build(container);
        this._startLoop();
    }

    get isLive() { return this._indexMs === null; }

    _build(container) {
        container.innerHTML = `
            <div style="display:flex;align-items:center;gap:6px;padding:0 8px;height:100%;font-size:0.72rem;">
                <button class="m3-icon-btn m3-icon-btn--sm" id="ts-to-start" title="先頭へ">
                    <span class="material-symbols-outlined icon-sm">skip_previous</span>
                </button>
                <button class="m3-icon-btn m3-icon-btn--sm" id="ts-play" title="再生/一時停止">
                    <span class="material-symbols-outlined icon-sm" id="ts-play-icon">play_arrow</span>
                </button>
                <button class="m3-icon-btn m3-icon-btn--sm" id="ts-to-end" title="最新へ">
                    <span class="material-symbols-outlined icon-sm">skip_next</span>
                </button>
                <select class="m3-text-input" id="ts-speed" style="padding:2px 4px;font-size:0.68rem;">
                    <option value="0.5">0.5×</option>
                    <option value="1" selected>1×</option>
                    <option value="2">2×</option>
                    <option value="4">4×</option>
                </select>
                <input type="range" id="ts-slider" min="0" max="1000" value="1000"
                    style="flex:1;accent-color:var(--md-sys-color-primary);">
                <span id="ts-time-label" style="min-width:7em;text-align:right;color:var(--md-sys-color-on-surface-variant);">LIVE</span>
                <button class="m3-btn m3-btn--sm m3-btn--tonal" id="ts-live"
                    style="padding:2px 10px;font-size:0.68rem;">LIVE</button>
            </div>
        `;
        this._slider = container.querySelector('#ts-slider');
        this._playIcon = container.querySelector('#ts-play-icon');
        this._timeLabel = container.querySelector('#ts-time-label');

        container.querySelector('#ts-to-start').addEventListener('click', () => this._goToStart());
        container.querySelector('#ts-play').addEventListener('click', () => this._togglePlay());
        container.querySelector('#ts-to-end').addEventListener('click', () => this._goLive());
        container.querySelector('#ts-live').addEventListener('click', () => this._goLive());
        container.querySelector('#ts-speed').addEventListener('change', (e) => {
            this._speed = parseFloat(e.target.value);
        });

        let dragging = false;
        this._slider.addEventListener('pointerdown', () => {
            dragging = true;
            this._playing = false;
            this._playIcon.textContent = 'play_arrow';
        });
        this._slider.addEventListener('input', () => {
            if (!dragging) return;
            this._seekBySlider(parseInt(this._slider.value, 10));
        });
        this._slider.addEventListener('pointerup', () => { dragging = false; });

        document.addEventListener('keydown', (e) => {
            const tag = document.activeElement?.tagName;
            if (tag === 'INPUT' || tag === 'TEXTAREA' || tag === 'SELECT') return;
            if (e.key === '[') { e.preventDefault(); this._seekRelative(-500); }
            if (e.key === ']') { e.preventDefault(); this._seekRelative(500); }
            if (e.key === ' ' && !this.isLive) { e.preventDefault(); this._togglePlay(); }
            if (e.key === 'End') { e.preventDefault(); this._goLive(); }
        });
    }

    _startLoop() {
        const loop = (ts) => {
            this._rafId = requestAnimationFrame(loop);
            if (this._lastRafTs === null) { this._lastRafTs = ts; return; }
            const dt = ts - this._lastRafTs;
            this._lastRafTs = ts;

            if (!this.isLive && this._playing) {
                this._indexMs += dt * this._speed;
                const newest = this._ring.newestMs;
                if (newest !== null && this._indexMs >= newest) {
                    this._goLive();
                    return;
                }
                this._applyFrame(this._indexMs);
            }
            this._updateSlider();
        };
        this._rafId = requestAnimationFrame(loop);
    }

    _goToStart() {
        const oldest = this._ring.oldestMs;
        if (oldest === null) return;
        this._indexMs = oldest;
        this._playing = false;
        this._playIcon.textContent = 'play_arrow';
        this._applyFrame(this._indexMs);
    }

    _goLive() {
        this._playing = false;
        this._indexMs = null;
        this._playIcon.textContent = 'play_arrow';
        this._slider.value = 1000;
        this._timeLabel.textContent = 'LIVE';
        // LIVE 復帰: viewer の状態は WebSocket からリアルタイムで来るので特に何もしない
        this._viewer._replayMode = false;
    }

    _togglePlay() {
        if (this.isLive) return;
        this._playing = !this._playing;
        this._playIcon.textContent = this._playing ? 'pause' : 'play_arrow';
    }

    _seekBySlider(sliderVal) {
        const oldest = this._ring.oldestMs;
        const newest = this._ring.newestMs;
        if (oldest === null || newest === null) return;
        const range = newest - oldest;
        if (range <= 0) return;
        this._indexMs = oldest + range * (sliderVal / 1000);
        this._applyFrame(this._indexMs);
    }

    _seekRelative(deltaMs) {
        if (this.isLive) return;
        this._indexMs = (this._indexMs ?? this._ring.newestMs ?? 0) + deltaMs;
        const oldest = this._ring.oldestMs ?? 0;
        const newest = this._ring.newestMs ?? 0;
        this._indexMs = Math.max(oldest, Math.min(newest, this._indexMs));
        this._applyFrame(this._indexMs);
    }

    _applyFrame(tsMs) {
        const state = this._ring.seek(tsMs);
        if (!state) return;
        this._viewer._replayMode = true;
        this._viewer.layerStore = state.layerStore;
        this._viewer.robotsOurs = state.robotsOurs;
        this._viewer.robotsTheirs = state.robotsTheirs;
        this._viewer.ballPos = state.ball;
        this._viewer.controlTargets = state.controlTargets;
        this._viewer.renderer?.invalidate();
    }

    _updateSlider() {
        const oldest = this._ring.oldestMs;
        const newest = this._ring.newestMs;
        if (oldest === null || newest === null) return;
        const range = newest - oldest;

        if (this.isLive) {
            this._slider.value = 1000;
            this._timeLabel.textContent = 'LIVE';
        } else {
            const pos = range > 0 ? (this._indexMs - oldest) / range * 1000 : 1000;
            this._slider.value = Math.round(Math.max(0, Math.min(1000, pos)));
            const relSec = ((newest - this._indexMs) / 1000).toFixed(1);
            this._timeLabel.textContent = `-${relSec}s`;
        }
    }

    destroy() {
        if (this._rafId) cancelAnimationFrame(this._rafId);
    }
}
