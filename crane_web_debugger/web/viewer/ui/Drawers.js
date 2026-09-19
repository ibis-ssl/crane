// フィールド左上のハンドルで開くオーバーレイドロワー。
//
// ドロワーはフィールドを押しのけず上に重なる。フィールドの幅が変わらないので、
// 開閉してもカメラ (viewBox) が揺れない。開くのは常に 1 枚だけ。
//
// Escape は capture 段階で握って伝播を止める（ラダーの 2 段目）。ここで止めないと
// 同じ Escape が main.js の 3 段目以降まで通り、ドロワーを閉じるつもりが
// 指令モードまで解除されてしまう。

export class Drawers {
    constructor() {
        this._drawers = new Map();   // name -> element
        this._open = null;
        for (const el of document.querySelectorAll('.cv-drawer')) {
            const name = el.dataset.drawer;
            if (!name) continue;
            this._drawers.set(name, el);
            el.querySelector('[data-drawer-close]')
                ?.addEventListener('click', () => this.close());
        }
        for (const btn of document.querySelectorAll('[data-drawer-toggle]')) {
            btn.addEventListener('click', () => this.toggle(btn.dataset.drawerToggle));
        }
        document.addEventListener('keydown', (e) => {
            if (e.key !== 'Escape' || this._open === null) return;
            this.close();
            e.stopImmediatePropagation();
        }, true);
    }

    get openName() { return this._open; }

    toggle(name) { this._open === name ? this.close() : this.open(name); }

    open(name) {
        const el = this._drawers.get(name);
        if (!el) return;
        this.close();
        el.classList.add('open');
        this._open = name;
        this._syncHandles();
    }

    close() {
        if (this._open === null) return;
        this._drawers.get(this._open)?.classList.remove('open');
        this._open = null;
        this._syncHandles();
    }

    _syncHandles() {
        for (const btn of document.querySelectorAll('[data-drawer-toggle]')) {
            const on = btn.dataset.drawerToggle === this._open;
            btn.classList.toggle('active', on);
            btn.setAttribute('aria-expanded', String(on));
        }
    }
}
