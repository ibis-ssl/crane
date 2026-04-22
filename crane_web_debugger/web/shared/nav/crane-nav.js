/**
 * <crane-nav> — Crane 全 Web UI 共通ナビゲーションバー。
 *
 * 使い方:
 *   <script type="module" src="/shared/nav/crane-nav.js"></script>
 *   <crane-nav active="viewer"></crane-nav>
 *
 * active 属性: viewer | telemetry | robot-test | annotation |
 *              ball-calibration | robot-manager | game-controller |
 *              vision-client | status-board
 *
 * ステータス表示:
 *   接続インジケータなど追加コンテンツは
 *   this.querySelector('#crane-nav-status') に書き込む（ページ側 JS で操作）。
 */

const NAV_ITEMS = [
  { id: 'viewer',           label: 'Viewer',           icon: 'stadium',       port: 8090, path: '/' },
  { id: 'telemetry',        label: 'Telemetry',        icon: 'monitor_heart', port: 8090, path: '/robot_telemetry.html' },
  { id: 'robot-test',       label: 'Robot Test',       icon: 'bolt',          port: 8090, path: '/robot_test.html' },
  { id: 'annotation',       label: 'Annotation',       icon: 'edit_note',     port: 8090, path: '/annotation/' },
  { id: 'ball-calibration', label: 'Ball Calibration', icon: 'sports_soccer', port: 8093, path: '/' },
  { id: 'robot-manager',    label: 'Robot Manager',    icon: 'tune',          port: 8092, path: '/' },
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
