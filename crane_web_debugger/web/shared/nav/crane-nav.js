/**
 * <crane-nav> — Crane 全 Web UI 共通ナビゲーションバー。
 *
 * 使い方:
 *   <script type="module" src="/shared/nav/crane-nav.js"></script>
 *   <crane-nav active="viewer"></crane-nav>
 *
 * active 属性: viewer | packet-forge | annotation | robot-manager |
 *              game-controller | vision-client | status-board
 *
 * Telemetry と Robot Test は Viewer のフォーカスサイドバー
 * (/?robot=<id>&tab=telemetry|test) へ統合したので項目から外してある。
 * 旧 URL は robot_telemetry.html / robot_test.html のスタブが転送する。
 *
 * ステータス表示:
 *   接続インジケータなど追加コンテンツは
 *   this.querySelector('#crane-nav-status') に書き込む（ページ側 JS で操作）。
 */

const NAV_ITEMS = [
  { id: 'viewer',           label: 'Viewer',           icon: 'stadium',       port: 8090, path: '/' },
  { id: 'packet-forge',     label: 'Packet Forge',     icon: 'construction',  port: 8094, path: '/' },
  { id: 'annotation',       label: 'Annotation',       icon: 'edit_note',     port: 8090, path: '/annotation/' },
  { id: 'robot-manager',    label: 'Robot Manager',    icon: 'tune',          port: 8090, path: '/robot_manager/' },
  { id: 'game-controller',  label: 'Game Controller',  icon: 'sports',        port: 8081, path: '/' },
  { id: 'vision-client',    label: 'Vision Client',    icon: 'visibility',    port: 8082, path: '/' },
  { id: 'status-board',     label: 'Status Board',     icon: 'dashboard',     port: 8083, path: '/' },
];

class CraneNav extends HTMLElement {
  connectedCallback() {
    const active = this.getAttribute('active') || '';
    const host = window.location.hostname;

    const linkItems = NAV_ITEMS.map(({ id, label, icon, port, path }) => {
      const url = `http://${host}:${port}${path}`;
      const isCurrent = id === active;
      const ariaCurrent = isCurrent ? ' aria-current="page"' : '';
      const cls = isCurrent ? ' class="active"' : '';
      return `<li><a href="${url}"${cls}${ariaCurrent}><span class="material-symbols-outlined icon-sm">${icon}</span><span class="crane-nav__label">${label}</span></a></li>`;
    }).join('');

    this.innerHTML = `
<nav class="m3-top-app-bar crane-nav" aria-label="Crane navigation">
  <div class="m3-top-app-bar__title">
    <span class="material-symbols-outlined">smart_toy</span>
    Crane
  </div>
  <div class="m3-top-app-bar__actions">
    <div id="crane-nav-status"></div>
    <ul class="m3-top-app-bar__nav-links crane-nav__links" role="list">
      ${linkItems}
    </ul>
  </div>
</nav>`;
  }
}

customElements.define('crane-nav', CraneNav);
