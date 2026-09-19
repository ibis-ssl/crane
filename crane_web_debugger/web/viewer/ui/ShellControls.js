// 右端のロボットレール (56px) と詳細サイドバー (360px) の開閉。
//
// ドロワーは Drawers.js が持つ。V4 で RobotRail として独立させる予定。

import { Drawers } from './Drawers.js';

const RAIL_ROBOT_COUNT = 13;
const RAIL_REFRESH_MS = 500;

export class ShellControls {
    constructor(viewer) {
        this._viewer = viewer;
        this._railBtns = new Map();  // id -> button
        this.drawers = new Drawers();
        this._setupRail();
    }

    // ActionDispatcher の 'drawer' アクションと CommandPalette から使う
    openDrawer(name) { this.drawers.open(name); }
    closeDrawer() { this.drawers.close(); }
    get openDrawerName() { return this.drawers.openName; }

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
        const sel = this._viewer.focusedRobotId;
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
}
