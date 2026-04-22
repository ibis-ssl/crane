/**
 * Crane Web UI 共通リンク定義。
 * hostname をベースにポートごとの URL を動的生成する。
 */

export const NAV_ITEMS = [
  { id: 'viewer',           label: 'Viewer',            icon: 'stadium',        port: 8090, path: '/' },
  { id: 'telemetry',        label: 'Telemetry',         icon: 'monitor_heart',  port: 8090, path: '/robot_telemetry.html' },
  { id: 'robot-test',       label: 'Robot Test',        icon: 'bolt',           port: 8090, path: '/robot_test.html' },
  { id: 'annotation',       label: 'Annotation',        icon: 'edit_note',      port: 8090, path: '/annotation/' },
  { id: 'ball-calibration', label: 'Ball Calibration',  icon: 'sports_soccer',  port: 8093, path: '/' },
  { id: 'robot-manager',    label: 'Robot Manager',     icon: 'tune',           port: 8092, path: '/' },
  { id: 'game-controller',  label: 'Game Controller',   icon: 'sports',         port: 8081, path: '/' },
  { id: 'vision-client',    label: 'Vision Client',     icon: 'visibility',     port: 8082, path: '/' },
  { id: 'status-board',     label: 'Status Board',      icon: 'dashboard',      port: 8083, path: '/' },
];

/**
 * @param {string} id  NAV_ITEMS の id
 * @returns {string}   http://{hostname}:{port}{path}
 */
export function navUrl(id) {
  const item = NAV_ITEMS.find(n => n.id === id);
  if (!item) return '#';
  return `http://${window.location.hostname}:${item.port}${item.path}`;
}
