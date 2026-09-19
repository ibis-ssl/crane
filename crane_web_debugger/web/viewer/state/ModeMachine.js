// フィールドの操作モードと、その DOM 側の副作用（ボタンの active、カーソル）を
// 1 箇所に集める。
//
// 【排他の規則】現行の振る舞いをそのまま明文化したもので、変更ではない。
//   - test は他のすべてと共存しない（C-3 の「他レイヤーのクリックは透過しません」）
//   - simEdit は move / ballPlacement のどちらとも共存しない
//   - move と ballPlacement は共存する（移動モードのまま配置位置を指す運用がある）
//
// 【Escape ラダー】exitTop() が上から 1 段だけ降りる。順序はここが正本で、
// main.js 側に if の列を書き直さないこと。test / move の解除は HALT セッションを
// 投げるため、ballPlacement より先に置く（Esc 連打で確認なし HALT にしない）。

const LADDER = ['test', 'move', 'ballPlacement', 'simEdit'];

export class ModeMachine extends EventTarget {
    constructor() {
        super();
        this.test = false;
        this.move = false;
        this.simEdit = false;
        this.ballPlacement = null;   // null | 'YELLOW' | 'BLUE'
        this.simSelectedObj = null;
        this._hooks = {};
    }

    // 副作用（WS 送信・再描画）は所有者が注入する。DOM 操作はここで完結させる。
    setHooks(hooks) { this._hooks = hooks; }

    is(name) {
        if (name === 'ballPlacement') return this.ballPlacement !== null;
        return Boolean(this[name]);
    }

    // ===== テストモード =====
    // ロボット単位ではなく crane 全体のモード。解除は HALT を注入する。

    enterTest() {
        if (this.test) return;
        this.exitMove();
        this.exitBallPlacement();
        if (this.simEdit) this.toggleSimEdit();
        this.test = true;
        document.body.classList.add('test-mode');
        this._syncRefereeButtons();
        this._hooks.onEnterTest?.();
        this._changed();
    }

    exitTest() {
        if (!this.test) return;
        this.test = false;
        document.body.classList.remove('test-mode');
        this._syncRefereeButtons();
        this._hooks.onExitTest?.();
        this._changed();
    }

    // テスト中に生かすのは HALT / STOP / NORMAL_START の 3 つだけ。
    // FK / KO / PLACE を押せると、テスト中の機体にレフェリー指令が割り込む。
    _syncRefereeButtons() {
        for (const btn of document.querySelectorAll('#command-bar [data-action]')) {
            const keep = btn.dataset.action === 'gc-command'
                && ['HALT', 'STOP', 'NORMAL_START'].includes(btn.dataset.type);
            btn.disabled = this.test && !keep;
        }
    }

    // ===== 移動モード =====

    enterMove() {
        if (this.move || this.test) return;
        this.move = true;
        document.getElementById('btn-move-mode')?.classList.add('active');
        this._hooks.onEnterMove?.();
        this._changed();
    }

    exitMove() {
        if (!this.move) return;
        this.move = false;
        document.getElementById('btn-move-mode')?.classList.remove('active');
        // フォーカスは維持する（サイドバーの表示対象を兼ねるため）
        this._changed();
    }

    toggleMove() { this.move ? this.exitMove() : this.enterMove(); }

    // ===== sim 編集 =====

    toggleSimEdit() {
        if (this.test) return;   // テスト中は他のモードに入らせない
        this.simEdit = !this.simEdit;
        const btn = document.getElementById('btn-sim-edit');
        if (this.simEdit) {
            this.exitMove();
            this.exitBallPlacement();
            btn?.classList.add('active');
        } else {
            btn?.classList.remove('active');
        }
        this.simSelectedObj = null;
        this._syncCursor();
        this._changed();
    }

    // ===== ボール配置 =====

    enterBallPlacement(team) {
        if (this.test) return;
        if (this.simEdit) this.toggleSimEdit();
        this.exitBallPlacement();
        this.ballPlacement = team;
        document.getElementById(`btn-place-ball-${team.toLowerCase()}`)?.classList.add('active');
        this._syncCursor();
        this._changed();
    }

    exitBallPlacement() {
        if (this.ballPlacement === null) return;
        const team = this.ballPlacement;
        this.ballPlacement = null;
        document.getElementById(`btn-place-ball-${team.toLowerCase()}`)?.classList.remove('active');
        this._syncCursor();
        this._changed();
    }

    // ===== Escape =====

    // 有効なモードのうち最上位を 1 段だけ降りる。降りたモード名を返す（無ければ null）
    exitTop() {
        for (const name of LADDER) {
            if (!this.is(name)) continue;
            if (name === 'test') this.exitTest();
            else if (name === 'move') this.exitMove();
            else if (name === 'ballPlacement') this.exitBallPlacement();
            else if (name === 'simEdit') this.toggleSimEdit();
            return name;
        }
        return null;
    }

    get anyActive() { return LADDER.some(n => this.is(n)); }

    // ===== 表示 =====

    _syncCursor() {
        const canvas = document.getElementById('field-canvas');
        if (!canvas) return;
        canvas.style.cursor = (this.simEdit || this.ballPlacement) ? 'crosshair' : 'grab';
    }

    syncSimLabel() {
        const el = document.getElementById('sim-selected-label');
        if (!el) return;
        if (!this.simEdit) { el.textContent = 'mode off'; return; }
        if (!this.simSelectedObj) { el.textContent = 'click to select'; return; }
        const obj = this.simSelectedObj;
        el.textContent = obj.type === 'ball' ? 'Ball' : `${obj.yellow ? 'Yellow' : 'Blue'} #${obj.id}`;
    }

    _changed() {
        this.syncSimLabel();
        this.dispatchEvent(new Event('change'));
        this._hooks.onChange?.();
    }
}
