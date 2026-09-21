// data-action の意味論を 1 箇所に置く。
//
// DOM の click 委譲（ボタン）とコマンドパレット（⌘K）の両方がここを呼ぶ。
// 実装を 2 つ作ると、片方だけ直した／片方にしか無い操作がある、という形で
// 必ずずれる。パレットに出す操作は PALETTE_ITEMS に並べる。

export const PALETTE_ITEMS = [
    { action: 'gc-command', data: { type: 'HALT' }, label: 'HALT — 全ロボット停止', keywords: 'halt stop 停止 緊急' },
    { action: 'gc-command', data: { type: 'STOP' }, label: 'STOP', keywords: 'stop 停止' },
    { action: 'gc-command', data: { type: 'FORCE_START' }, label: 'FORCE START', keywords: 'force start 強制 開始' },
    { action: 'gc-command', data: { type: 'NORMAL_START' }, label: 'NORMAL START', keywords: 'normal start 開始' },

    // ここから下の 9 つは C 案のコマンドバーに居場所が無く、パレットが唯一の導線になる
    { action: 'gc-goals', data: { team: 'YELLOW', delta: '1' }, label: 'スコア +1 — YELLOW', keywords: 'score goal yellow 得点 スコア' },
    { action: 'gc-goals', data: { team: 'YELLOW', delta: '-1' }, label: 'スコア −1 — YELLOW', keywords: 'score goal yellow 得点 スコア' },
    { action: 'gc-goals', data: { team: 'BLUE', delta: '1' }, label: 'スコア +1 — BLUE', keywords: 'score goal blue 得点 スコア' },
    { action: 'gc-goals', data: { team: 'BLUE', delta: '-1' }, label: 'スコア −1 — BLUE', keywords: 'score goal blue 得点 スコア' },
    { action: 'gc-card-yellow', data: { team: 'YELLOW' }, label: '警告カード — YELLOW', keywords: 'card yellow warning カード 警告' },
    { action: 'gc-card-yellow', data: { team: 'BLUE' }, label: '警告カード — BLUE', keywords: 'card yellow warning カード 警告' },
    { action: 'gc-card-red', data: { team: 'YELLOW' }, label: '退場カード — YELLOW', keywords: 'card red カード 退場' },
    { action: 'gc-card-red', data: { team: 'BLUE' }, label: '退場カード — BLUE', keywords: 'card red カード 退場' },
    { action: 'gc-next-stage', data: {}, label: '次のステージへ', keywords: 'next stage ステージ 前半 後半' },

    { action: 'gc-command', data: { type: 'DIRECT', team: 'YELLOW' }, label: 'フリーキック — YELLOW', keywords: 'free kick fk direct' },
    { action: 'gc-command', data: { type: 'DIRECT', team: 'BLUE' }, label: 'フリーキック — BLUE', keywords: 'free kick fk direct' },
    { action: 'gc-command', data: { type: 'KICKOFF', team: 'YELLOW' }, label: 'キックオフ — YELLOW', keywords: 'kickoff ko' },
    { action: 'gc-command', data: { type: 'KICKOFF', team: 'BLUE' }, label: 'キックオフ — BLUE', keywords: 'kickoff ko' },
    { action: 'ball-place', data: { team: 'YELLOW' }, label: 'ボール配置 — YELLOW', keywords: 'ball placement 配置' },
    { action: 'ball-place', data: { team: 'BLUE' }, label: 'ボール配置 — BLUE', keywords: 'ball placement 配置' },

    { action: 'sim-edit', data: {}, label: 'Sim 編集モードの切り替え', keywords: 'sim edit teleport 編集' },
    { action: 'sim-reset-ball', data: {}, label: 'ボールを中央へ戻す', keywords: 'ball reset center リセット' },
    { action: 'session-clear', data: {}, label: 'セッションをクリア（HALT）', keywords: 'session clear halt セッション' },

    { action: 'drawer', data: { name: 'position-control' }, label: '位置制御ゲインを開く', keywords: 'gain kp tuning ゲイン 調整 位置制御' },
    { action: 'drawer', data: { name: 'match' }, label: '試合管理を開く', keywords: 'match game 試合' },
    { action: 'drawer', data: { name: 'log' }, label: 'ログを開く', keywords: 'log ログ' },
    { action: 'drawer', data: { name: 'layers' }, label: 'レイヤーを開く', keywords: 'layer レイヤー' },
    { action: 'drawer', data: { name: 'session' }, label: 'Sim / Session を開く', keywords: 'sim session 注入' },
    { action: 'drawer', data: { name: 'replay' }, label: 'リプレイを開く', keywords: 'replay リプレイ' },
];

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
            // パレット専用（DOM 側に対応するボタンは無い）
            'drawer': (d) => v.shell?.openDrawer(d.name),
        };
    }
}
