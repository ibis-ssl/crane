// 右サイドバー (56px ⇄ 360px)。ロボット 1 台の情報をタブで完結させる。
//
// 別ページへ飛ばさないのが C-2 の要点。タブは「同時に生きるのは 1 枚だけ」を
// 不変条件にして、切替時に必ず前のタブの deactivate() を呼ぶ。これを怠ると
// Chart.js のインスタンスとリスナが残ってゾンビ更新になる。
//
// URL 契約: ?robot=<id>&tab=overview|telemetry|test|log
// 廃止した robot_telemetry.html / robot_test.html からの移送先でもある。

const DEFAULT_TAB = 'overview';

export class FocusSidebar {
    constructor(viewer) {
        this._v = viewer;
        this._tabs = new Map();      // name -> tab インスタンス
        this._activeName = null;
        this._robotId = null;
        this._body = document.getElementById('robot-detail-inline');
        this._tabBar = document.getElementById('focus-tabs');
        this._title = document.getElementById('focus-robot-id');
        // 56px ⇄ 360px の切り替えはこの属性が握る。以前は #robot-detail-inline の
        // .visible を MutationObserver で監視していたが、持ち主がここに来たので直接書く。
        this._side = document.getElementById('robot-side');
    }

    // tab は { label, mount(container), activate(robotId), deactivate(), refresh() } を持つ
    register(name, tab) {
        this._tabs.set(name, tab);
        this._renderTabBar();
    }

    _renderTabBar() {
        if (!this._tabBar) return;
        this._tabBar.innerHTML = '';
        for (const [name, tab] of this._tabs) {
            const btn = document.createElement('button');
            btn.type = 'button';
            btn.className = 'cv-tab';
            btn.dataset.tab = name;
            btn.textContent = tab.label;
            btn.setAttribute('role', 'tab');
            btn.classList.toggle('active', name === this._activeName);
            btn.addEventListener('click', () => this.selectTab(name));
            this._tabBar.appendChild(btn);
        }
    }

    get isOpen() { return this._robotId !== null; }
    get robotId() { return this._robotId; }
    get activeTabName() { return this._activeName; }

    open(id, tabName = null) {
        if (!this._body) return false;
        this._robotId = id;
        if (this._title) this._title.textContent = id;
        this._body.classList.add('visible');
        if (this._side) this._side.dataset.expanded = 'true';
        this.selectTab(tabName ?? this._activeName ?? DEFAULT_TAB);
        this._syncUrl();
        return true;
    }

    close() {
        if (this._activeName) this._tabs.get(this._activeName)?.deactivate?.();
        this._activeName = null;
        this._robotId = null;
        if (this._title) this._title.textContent = '--';
        this._body?.classList.remove('visible');
        if (this._side) this._side.dataset.expanded = 'false';
        if (this._body) this._body.innerHTML = '';
        this._renderTabBar();
        this._syncUrl();
    }

    selectTab(name) {
        if (!this._tabs.has(name)) name = DEFAULT_TAB;
        if (!this._tabs.has(name)) return;
        if (this._activeName === name) { this.refresh(); return; }

        // 同時に生きるタブは 1 枚だけ
        if (this._activeName) this._tabs.get(this._activeName)?.deactivate?.();
        this._activeName = name;
        this._body.innerHTML = '';
        const tab = this._tabs.get(name);
        tab.mount(this._body);
        tab.activate(this._robotId);
        this._renderTabBar();
        this._syncUrl();
    }

    refresh() {
        if (this._robotId === null || !this._activeName) return;
        this._tabs.get(this._activeName)?.refresh?.(this._robotId);
    }

    // 起動時に ?robot=&tab= を反映する（旧ページからのリダイレクト先）
    applyUrlParams() {
        const q = new URLSearchParams(location.search);
        const robot = q.get('robot');
        if (robot === null) return;
        const id = Number(robot);
        if (!Number.isInteger(id)) return;
        this._v.showRobotDetail(id, q.get('tab'));
    }

    // 履歴を汚さずアドレスバーだけ追従させる
    _syncUrl() {
        const q = new URLSearchParams(location.search);
        if (this._robotId === null) {
            q.delete('robot');
            q.delete('tab');
        } else {
            q.set('robot', String(this._robotId));
            q.set('tab', this._activeName ?? DEFAULT_TAB);
        }
        const qs = q.toString();
        history.replaceState(null, '', qs ? `${location.pathname}?${qs}` : location.pathname);
    }
}
