// フォーカスサイドバーの「ログ」タブ。
//
// ドロワーのログは全体向けなので、ここでは注目中のロボットに触れた行だけを出す。
// LogPanel は 1 バッファを共有しているため、そこから読むだけで二重に貯めない。

const MAX_ROWS = 200;

export class LogTab {
    constructor(logPanel) {
        this._logPanel = logPanel;
        this._root = null;
        this._id = null;
    }

    get label() { return 'ログ'; }

    mount(container) {
        this._root = document.createElement('div');
        this._root.className = 'rd-log';
        container.appendChild(this._root);
    }

    activate(id) {
        this._id = id;
        this._unsub = this._logPanel?.onAppend?.(() => this.refresh(this._id));
        this.refresh(id);
    }

    deactivate() {
        this._unsub?.();
        this._unsub = null;
        this._root = null;
    }

    refresh(id) {
        if (!this._root) return;
        const needle = `${id}`;
        const rows = (this._logPanel?.entries ?? [])
            .filter(e => e.msg.includes(needle) || e.tag === `R${id}`)
            .slice(-MAX_ROWS);
        if (rows.length === 0) {
            this._root.innerHTML = '<div class="rd-log__empty">このロボットに関するログはまだありません</div>';
            return;
        }
        this._root.innerHTML = rows.map(e =>
            `<div class="rd-log__row rd-log__row--${e.level}">`
            + `<span class="rd-log__time">${e.time}</span>`
            + `<span class="rd-log__tag">${e.tag}</span>`
            + `<span class="rd-log__msg">${e.msg}</span></div>`
        ).join('');
        this._root.scrollTop = this._root.scrollHeight;
    }
}
