// 生きたログパネル。level フィルタ + テキスト検索付き。
export class LogPanel {
    constructor(bodyEl) {
        this._body = bodyEl;
        this._count = 0;
        this._maxEntries = 1000;
        this._autoScroll = true;
        this._levelFilter = new Set(['info', 'warn', 'error', 'action', 'metric']);
        this._textFilter = '';
        // DOM とは別に素の記録も持つ。サイドバーのログタブが、同じ行を
        // もう一度貯めることなくロボット単位で絞り込めるようにするため。
        this.entries = [];
        this._appendSubs = new Set();

        this._setupScrollTracking();
    }

    // level: 'info'|'warn'|'error'|'action'|'metric'
    appendLog(level, tag, msg) {
        const entry = document.createElement('div');
        entry.className = 'log-entry';
        entry.dataset.level = level;
        const ts = new Date().toLocaleTimeString('ja', { hour12: false, hour: '2-digit', minute: '2-digit', second: '2-digit' });
        const text = `[${ts}][${tag}] ${msg}`;
        entry.textContent = text;
        this.entries.push({ time: ts, level, tag, msg });
        if (this.entries.length > this._maxEntries) this.entries.shift();

        const levelOk = this._levelFilter.has(level);
        const textOk = !this._textFilter || text.includes(this._textFilter);
        if (!levelOk || !textOk) entry.style.display = 'none';

        this._body.appendChild(entry);
        this._count++;

        if (this._count > this._maxEntries) {
            this._body.firstChild?.remove();
            this._count--;
        }
        if (this._autoScroll) {
            this._body.scrollTop = this._body.scrollHeight;
        }
        for (const fn of this._appendSubs) fn(this.entries[this.entries.length - 1]);
    }

    onAppend(fn) {
        this._appendSubs.add(fn);
        return () => this._appendSubs.delete(fn);
    }

    setLevelFilter(levels) {
        this._levelFilter = new Set(levels);
        this._refilter();
    }

    setTextFilter(text) {
        this._textFilter = text;
        this._refilter();
    }

    clear() {
        this.entries = [];
        this._body.innerHTML = '';
        this._count = 0;
    }

    _refilter() {
        for (const entry of this._body.querySelectorAll('.log-entry')) {
            const level = entry.dataset.level;
            const text = entry.textContent;
            const levelOk = this._levelFilter.has(level);
            const textOk = !this._textFilter || text.includes(this._textFilter);
            entry.style.display = (levelOk && textOk) ? '' : 'none';
        }
    }

    _setupScrollTracking() {
        this._body.addEventListener('scroll', () => {
            const isAtBottom = this._body.scrollHeight - this._body.scrollTop - this._body.clientHeight < 20;
            this._autoScroll = isAtBottom;
        });
    }
}
