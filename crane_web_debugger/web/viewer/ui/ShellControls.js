// C-1 シェルの操作系。フィールド左上のハンドルで開くオーバーレイドロワーと、
// 右端のロボットレール(56px) ⇄ 詳細サイドバーの開閉を受け持つ。
//
// ドロワーはフィールドを押しのけず上に重なる。フィールドの幅が変わらないので
// 開閉してもカメラ(viewBox)が揺れない。
//
// V3 で Drawers / RobotRail / StatusStrip に分割する予定の暫定モジュール。

const RAIL_ROBOT_COUNT = 13;
const RAIL_REFRESH_MS = 500;

export class ShellControls {
    constructor(viewer) {
        this._viewer = viewer;
        this._drawers = new Map();   // name -> { el, handle }
        this._open = null;
        this._railBtns = new Map();  // id -> button
        this._setupDrawers();
        this._setupRail();
        this._setupEscape();
    }

    // ===== ドロワー =====
    _setupDrawers() {
        for (const el of document.querySelectorAll('.cv-drawer')) {
            const name = el.dataset.drawer;
            if (!name) continue;
            this._drawers.set(name, el);
            el.querySelector('[data-drawer-close]')
                ?.addEventListener('click', () => this.closeDrawer());
        }
        for (const btn of document.querySelectorAll('[data-drawer-toggle]')) {
            btn.addEventListener('click', () => this.toggleDrawer(btn.dataset.drawerToggle));
        }
    }

    toggleDrawer(name) {
        if (this._open === name) this.closeDrawer();
        else this.openDrawer(name);
    }

    openDrawer(name) {
        const el = this._drawers.get(name);
        if (!el) return;
        this.closeDrawer();
        el.classList.add('open');
        this._open = name;
        this._syncHandles();
    }

    closeDrawer() {
        if (this._open === null) return;
        this._drawers.get(this._open)?.classList.remove('open');
        this._open = null;
        this._syncHandles();
    }

    get openDrawerName() { return this._open; }

    _syncHandles() {
        for (const btn of document.querySelectorAll('[data-drawer-toggle]')) {
            const on = btn.dataset.drawerToggle === this._open;
            btn.classList.toggle('active', on);
            btn.setAttribute('aria-expanded', String(on));
        }
    }

    // ===== ロボットレール =====
    _setupRail() {
        const rail = document.getElementById('robot-rail');
        const side = document.getElementById('robot-side');
        if (!rail || !side) return;

        for (let id = 0; id < RAIL_ROBOT_COUNT; id++) {
            const btn = document.createElement('button');
            btn.type = 'button';
            btn.className = 'cv-rail-btn';
            btn.textContent = String(id);
            btn.title = `ロボット ${id} の詳細を開く`;
            btn.setAttribute('aria-label', `ロボット ${id} の詳細を開く`);
            btn.addEventListener('click', () => this._viewer.toggleRobotDetail(id));
            rail.appendChild(btn);
            this._railBtns.set(id, btn);
        }

        // 詳細パネルの表示状態をサイドバーの幅に反映する。
        // main.js の showRobotDetail/closeRobotDetail は #robot-detail-inline の
        // .visible を切り替えるだけなので、そこを監視して結合を最小に保つ。
        const panel = document.getElementById('robot-detail-inline');
        if (panel) {
            const sync = () => {
                side.dataset.expanded = String(panel.classList.contains('visible'));
                this._syncRailSelection();
            };
            new MutationObserver(sync).observe(panel, {
                attributes: true, attributeFilter: ['class'],
            });
            sync();
        }

        this._railTimer = setInterval(() => this._syncRailPresence(), RAIL_REFRESH_MS);
        this._syncRailPresence();
    }

    _syncRailSelection() {
        const sel = this._viewer._detailRobotId;
        for (const [id, btn] of this._railBtns) {
            btn.classList.toggle('selected', id === sel);
        }
    }

    // 未検出のロボットは薄く出す（番号の位置が動かないよう非表示にはしない）
    _syncRailPresence() {
        const ours = this._viewer.robotsOurs ?? {};
        for (const [id, btn] of this._railBtns) {
            btn.classList.toggle('absent', !ours[id]);
        }
        this._syncRailSelection();
    }

    // ===== Escape =====
    // ドロワーが開いていれば閉じるだけで、main.js 側の Escape 処理
    // (ボール配置の取消・HALT 確認など) には渡さない。
    _setupEscape() {
        document.addEventListener('keydown', (e) => {
            if (e.key !== 'Escape' || this._open === null) return;
            this.closeDrawer();
            e.stopImmediatePropagation();
        }, true);
    }
}
