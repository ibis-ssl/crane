// 48px ステータスストリップ。試合中に見るものだけを固定位置に出す。
//
// スコアの書き手をここ 1 つに決める。Game Controller (8081) の
// teamState.YELLOW/BLUE.goals を正とし、8091 の game_info は GC 未接続時の
// フォールバックとして扱う。C-1 のスコアは両脇にチーム色ドットを置く
// YELLOW/BLUE 軸なので、自/敵軸の game_info をそのまま出すと左右が入れ替わる。

const LAST_ACTION_HOLD_MS = 8000;

export class StatusStrip {
    constructor(actions) {
        this._score = document.getElementById('gc-score');
        this._stage = document.getElementById('gc-stage');
        this._command = document.getElementById('gc-command');
        this._session = document.getElementById('strip-session');
        this._last = document.getElementById('strip-last-action');
        this._hasGc = false;
        this._lastTimer = null;
        actions?.onLastAction((entry) => this.showLastAction(entry));
    }

    // Game Controller (8081) — こちらが正
    updateFromGc(state) {
        if (!state) return;
        this._hasGc = true;
        const ts = state.teamState ?? {};
        if (this._score) this._score.textContent = `${ts.YELLOW?.goals ?? 0} : ${ts.BLUE?.goals ?? 0}`;
        if (this._stage) this._stage.textContent = (state.stage ?? '--').replace('NORMAL_', '').replace('_', ' ');
        if (this._command) this._command.textContent = state.command?.type ?? '--';
    }

    // websocket_server (8091) の game_info — GC が居ないときだけ使う
    updateFromGameInfo(data) {
        const set = (id, val) => { const el = document.getElementById(id); if (el) el.textContent = val; };
        set('score-our', data.our_score ?? 0);
        set('score-their', data.their_score ?? 0);
        set('play-situation', data.play_situation || '--');
        set('game-stage', data.game_stage || '--');
        if (this._hasGc) return;
        if (this._stage) this._stage.textContent = data.game_stage || '--';
        if (this._command) this._command.textContent = data.play_situation || '--';
    }

    // セッション注入は「Sim / Session」ドロワーとテストモードで同じ publisher を
    // 共有する。現在値の表示はここ 1 箇所に集約して二重管理にしない。
    updateSession(name) {
        if (!this._session) return;
        this._session.textContent = name || 'HALT';
        this._session.parentElement?.classList.toggle('cv-chip--warn', Boolean(name) && name !== 'HALT');
    }

    showLastAction(entry) {
        if (!this._last || !entry) return;
        this._last.textContent = entry.label;
        this._last.parentElement?.classList.add('visible');
        clearTimeout(this._lastTimer);
        this._lastTimer = setTimeout(() => {
            this._last.parentElement?.classList.remove('visible');
        }, LAST_ACTION_HOLD_MS);
    }
}
