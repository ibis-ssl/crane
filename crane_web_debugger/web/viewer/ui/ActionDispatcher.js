// data-action の意味論を 1 箇所に置く。
//
// DOM の click 委譲（ボタン）とコマンドパレット（⌘K）の両方がここを呼ぶ。
// 実装を 2 つ作ると、片方だけ直した／片方にしか無い操作がある、という形で
// 必ずずれる。パレットに出す操作は PALETTE_ITEMS に並べる。

export class ActionDispatcher {
    constructor(viewer) {
        this._v = viewer;
        this._last = null;
        this._lastSubs = new Set();
        this._handlers = this._build();
        document.addEventListener('click', (e) => {
            const btn = e.target.closest('[data-action]');
            if (!btn) return;
            this.dispatch(btn.dataset.action, btn.dataset);
        });
    }

    // 直前の操作。C-1 のステータスストリップに出す
    get lastAction() { return this._last; }

    onLastAction(fn) {
        this._lastSubs.add(fn);
        return () => this._lastSubs.delete(fn);
    }

    dispatch(action, data = {}) {
        const fn = this._handlers[action];
        if (!fn) return false;
        fn(data);
        return true;
    }

    _note(label) {
        this._last = { label, at: Date.now() };
        for (const fn of this._lastSubs) fn(this._last);
    }

    _build() {
        const v = this._v;
        const log = (tag, msg) => {
            v.logPanel?.appendLog('action', tag, msg);
            this._note(msg);
        };

        return {
            'gc-command': (d) => {
                v.gcClient.newCommand(d.type, d.team || 'UNKNOWN');
                log('GC', `command ${d.type}${d.team ? ' for ' + d.team : ''}`);
            },
            'gc-goals': (d) => {
                const delta = parseInt(d.delta ?? '0', 10);
                v.gcClient.updateGoals(d.team, delta);
                log('GC', `goals ${d.team} ${delta >= 0 ? '+' : ''}${delta}`);
            },
            'gc-card-yellow': (d) => {
                v.gcClient.addYellowCard(d.team);
                log('GC', `yellow card for ${d.team}`);
            },
            'gc-card-red': (d) => {
                v.gcClient.addRedCard(d.team);
                log('GC', `red card for ${d.team}`);
            },
            'gc-next-stage': () => {
                v.gcClient.nextStage();
                log('GC', 'next stage');
            },
            'ball-place': (d) => {
                v.modes.enterBallPlacement(d.team);
                log('GC', `ball placement mode: ${d.team}`);
            },
            'sim-edit': () => {
                v.modes.toggleSimEdit();
                v.renderer?.invalidate();
            },
            'sim-reset-ball': () => {
                v.hub.send({ type: 'sim_teleport_ball', x: 0, y: 0, vx: 0, vy: 0 });
                log('Sim', 'ball reset to center');
            },
            'sim-set-endpoint': () => {
                const host = document.getElementById('sim-host-input')?.value || '127.0.0.1';
                const port = parseInt(document.getElementById('sim-port-input')?.value || '10300', 10);
                v.hub.send({ type: 'sim_set_endpoint', host, port });
                log('Sim', 'endpoint updated');
            },
            'session-inject-select': () => {
                const name = document.getElementById('session-select')?.value;
                if (!name) return;
                v.hub.send({ type: 'session_inject', name });
                log('Session', `inject ${name}`);
            },
            'session-inject-custom': () => {
                const name = (document.getElementById('session-custom-input')?.value || '').trim();
                if (!name) return;
                v.hub.send({ type: 'session_inject', name });
                log('Session', `inject(custom) ${name}`);
            },
            'session-clear': () => {
                v.hub.send({ type: 'session_clear' });
                log('Session', 'clear → HALT');
            },
        };
    }
}
