// ⌘K / Ctrl+K のコマンドパレット。
//
// C 案のコマンドバーには HALT / STOP / FORCE / START と各チームの
// FK / KO / PLACE しか居場所がない。旧・左ドックにあったスコア±4・カード4・
// 次のステージ 1 の計 9 操作は、ここが唯一の導線になる。
//
// 実行は必ず ActionDispatcher を通す。パレット用に操作をもう一度実装しない。

import { PALETTE_ITEMS } from './ActionDispatcher.js';

export class CommandPalette {
    constructor(dispatcher) {
        this._dispatch = dispatcher;
        this._items = PALETTE_ITEMS;
        this._filtered = [];
        this._cursor = 0;
        this._root = document.getElementById('command-palette');
        this._input = document.getElementById('command-palette-input');
        this._list = document.getElementById('command-palette-list');
        if (!this._root) return;

        this._input.addEventListener('input', () => this._refilter());
        this._input.addEventListener('keydown', (e) => this._onKey(e));
        this._root.addEventListener('mousedown', (e) => {
            // 背景クリックで閉じる（中身のクリックは拾わない）
            if (e.target === this._root) this.close();
        });
        document.getElementById('btn-command-palette')
            ?.addEventListener('click', () => this.toggle());
    }

    get isOpen() { return this._root?.classList.contains('open') ?? false; }

    toggle() { this.isOpen ? this.close() : this.open(); }

    open() {
        if (!this._root) return;
        this._root.classList.add('open');
        this._input.value = '';
        this._refilter();
        this._input.focus();
    }

    close() {
        this._root?.classList.remove('open');
        this._input?.blur();
    }

    _refilter() {
        const q = (this._input.value ?? '').trim().toLowerCase();
        this._filtered = q
            ? this._items.filter(it => `${it.label} ${it.keywords}`.toLowerCase().includes(q))
            : this._items;
        this._cursor = 0;
        this._render();
    }

    _render() {
        this._list.innerHTML = '';
        if (this._filtered.length === 0) {
            const empty = document.createElement('div');
            empty.className = 'cp-empty';
            empty.textContent = '一致する操作がありません';
            this._list.appendChild(empty);
            return;
        }
        this._filtered.forEach((item, i) => {
            const row = document.createElement('button');
            row.type = 'button';
            row.className = 'cp-item';
            row.textContent = item.label;
            if (i === this._cursor) row.classList.add('active');
            row.addEventListener('click', () => this._run(item));
            this._list.appendChild(row);
        });
        this._list.children[this._cursor]?.scrollIntoView({ block: 'nearest' });
    }

    _onKey(e) {
        if (e.key === 'ArrowDown') {
            e.preventDefault();
            this._cursor = Math.min(this._cursor + 1, this._filtered.length - 1);
            this._render();
        } else if (e.key === 'ArrowUp') {
            e.preventDefault();
            this._cursor = Math.max(this._cursor - 1, 0);
            this._render();
        } else if (e.key === 'Enter') {
            e.preventDefault();
            const item = this._filtered[this._cursor];
            if (item) this._run(item);
        }
        // Escape は main.js の Escape ラダー 1 段目で閉じる（ここでは拾わない）
    }

    _run(item) {
        this.close();
        this._dispatch.dispatch(item.action, item.data);
    }
}
