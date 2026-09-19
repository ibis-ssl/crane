// C-1 シェルの操作系をまとめる薄い入口。
// 実体は Drawers / RobotRail / FocusSidebar にあり、ここは組み立てと
// 既存の呼び出し口（openDrawer 等）の維持だけを受け持つ。

import { Drawers } from './Drawers.js';
import { RobotRail } from './RobotRail.js';

export class ShellControls {
    constructor(viewer) {
        this.drawers = new Drawers();
        this.rail = new RobotRail(viewer);
    }

    // ActionDispatcher の 'drawer' アクションと CommandPalette から使う
    openDrawer(name) { this.drawers.open(name); }
    closeDrawer() { this.drawers.close(); }
    get openDrawerName() { return this.drawers.openName; }
}
