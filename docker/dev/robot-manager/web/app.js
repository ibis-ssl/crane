'use strict';

// ---- 定数 ------------------------------------------------------------------

const NUM_ROBOTS = 13;

// 温度センサーのコンポーネント名（インデックス順）
const TEMP_NAMES = ['RF', 'RB', 'LB', 'LF', 'FET', 'コイル1', 'コイル2'];
const TEMP_WARN = 45;
const TEMP_CRIT = 60;

// 電圧しきい値
const VOLTAGE_WARN = 23.0;
const VOLTAGE_CRIT = 22.0;

// ---- ステータス設定 ---------------------------------------------------------

const STATUS_CONFIG = {
  running: { badge: 'm3-badge--success', label: '稼働中' },
  stopped: { badge: 'm3-badge--warning', label: '停止中' },
  error:   { badge: 'm3-badge--error',   label: 'エラー' },
  offline: { badge: 'm3-badge--secondary', label: 'オフライン' },
};

// ---- 状態 ------------------------------------------------------------------

let robots = [];
let pendingControlIds = new Set();

// ---- DOM参照 ---------------------------------------------------------------

const tableBody = document.getElementById('table-body');
const connDot = document.getElementById('conn-dot');
const connText = document.getElementById('conn-text');
const runningCount = document.getElementById('running-count');
const stoppedCount = document.getElementById('stopped-count');
const offlineCount = document.getElementById('offline-count');
const lastUpdatedEl = document.getElementById('last-updated');
const refreshBtn = document.getElementById('refresh-btn');
const startAllBtn = document.getElementById('start-all-btn');
const stopAllBtn = document.getElementById('stop-all-btn');

// ---- 接続状態 --------------------------------------------------------------

function setConnected(ok) {
  connDot.className = ok ? 'm3-connection-dot connected' : 'm3-connection-dot';
  connText.textContent = ok ? '接続済み' : '切断';
}

// ---- エラーデコード ---------------------------------------------------------

function getErrorString(error_id, error_info) {
  if (!error_info) return null;

  if (error_id === 100) {
    // POWERボードエラー
    const bits = [
      [0x0001, '低電圧'],
      [0x0002, '過電圧'],
      [0x0004, '過電流'],
      [0x0008, '短絡'],
      [0x0010, '充電タイムアウト'],
      [0x0020, '充電電力異常'],
      [0x0040, '放電異常'],
      [0x0080, 'パラメータ異常'],
      [0x0100, 'コマンド異常'],
      [0x0200, 'キャパシタ未接続'],
      [0x0400, '放電失敗'],
      [0x0800, 'GD電源異常'],
      [0x1000, 'コイル過熱'],
      [0x2000, 'FET過熱'],
    ];
    const errors = bits.filter(([bit]) => error_info & bit).map(([, name]) => name);
    return 'POWER: ' + (errors.length ? errors.join(', ') : '不明');
  }

  if (error_id <= 3) {
    // BLDCモーターエラー（モーター位置付き）
    const motorNames = ['RF', 'RB', 'LB', 'LF'];
    const motorName = motorNames[error_id] ?? `M${error_id}`;
    const bits = [
      [0x0001, '低電圧'],
      [0x0002, '過電流'],
      [0x0004, 'モーター過熱'],
      [0x0008, '過負荷'],
      [0x0010, 'エンコーダエラー'],
      [0x0020, '過電圧'],
      [0x0040, 'FET過熱'],
    ];
    const errors = bits.filter(([bit]) => error_info & bit).map(([, name]) => name);
    return `BLDC[${motorName}]: ` + (errors.length ? errors.join(', ') : '不明');
  }

  return `ERR(id=${error_id}, info=0x${error_info.toString(16)})`;
}

// ---- ハードウェア詳細レンダリング -------------------------------------------

function renderHwDetail(robot) {
  const parts = [];

  // 電圧（配列の最初の値を使用）
  if (Array.isArray(robot.voltage) && robot.voltage.length > 0) {
    const v = robot.voltage[0];
    const cls = v < VOLTAGE_CRIT ? 'm3-text-error' : v < VOLTAGE_WARN ? 'm3-text-warning' : 'm3-text-success';
    parts.push(`<span class="${cls}">⚡${v.toFixed(1)}V</span>`);
  }

  // 温度（最大温度とそのコンポーネント名）
  if (Array.isArray(robot.temperatures) && robot.temperatures.length > 0) {
    const temps = robot.temperatures;
    const maxTemp = Math.max(...temps);
    const maxIdx = temps.indexOf(maxTemp);
    const label = TEMP_NAMES[maxIdx] ?? `T${maxIdx}`;
    const cls = maxTemp >= TEMP_CRIT ? 'm3-text-error' : maxTemp >= TEMP_WARN ? 'm3-text-warning' : '';
    parts.push(`<span class="${cls}">🌡${maxTemp}°C (${label})</span>`);
  }

  // エラー（デコード済み）
  if (robot.error_id !== undefined && robot.error_info) {
    const errStr = getErrorString(robot.error_id, robot.error_info);
    if (errStr) {
      parts.push(`<span class="m3-text-error">${errStr}</span>`);
    }
  }

  if (parts.length === 0) {
    return '<span class="m3-text-on-surface-variant">--</span>';
  }

  return parts.map((p, i) =>
    i === 0 ? p : `<span class="hw-divider">|</span>${p}`
  ).join('');
}

// ---- ステータス分類 ---------------------------------------------------------

function classifyStatus(status) {
  const s = (status || '').toLowerCase();
  if (s.includes('run')) return 'running';
  if (s.includes('stop')) return 'stopped';
  if (s.includes('error')) return 'error';
  return 'offline';
}

// ---- テーブル描画 -----------------------------------------------------------

function rowHtml(robot) {
  const cls = classifyStatus(robot.status);
  const cfg = STATUS_CONFIG[cls] ?? STATUS_CONFIG.offline;
  const isBusy = pendingControlIds.has(robot.robot_id);
  const disabledAttr = isBusy ? 'disabled' : '';

  return `
    <tr class="robot-row ${cls === 'offline' ? 'offline' : ''}" id="robot-row-${robot.robot_id}">
      <td>
        <span class="material-symbols-outlined icon-sm m3-text-on-surface-variant"
              style="vertical-align:middle;margin-right:4px">smart_toy</span>
        <strong>#${robot.robot_id}</strong>
      </td>
      <td>
        <div class="m3-flex m3-items-center m3-gap-sm">
          <span class="m3-badge ${cfg.badge}" style="min-width:72px">${cfg.label}</span>
          <div class="m3-flex m3-gap-xs">
            <button class="m3-icon-btn m3-icon-btn--sm"
                    onclick="controlRobot(${robot.robot_id}, 'start')"
                    title="起動" ${disabledAttr}
                    style="color:var(--md-sys-color-success)">
              <span class="material-symbols-outlined icon-sm">play_arrow</span>
            </button>
            <button class="m3-icon-btn m3-icon-btn--sm"
                    onclick="controlRobot(${robot.robot_id}, 'stop')"
                    title="停止" ${disabledAttr}
                    style="color:var(--md-sys-color-error)">
              <span class="material-symbols-outlined icon-sm">stop</span>
            </button>
          </div>
        </div>
      </td>
      <td class="col-ip">
        <span class="m3-text-on-surface-variant m3-body-small m3-tabular-nums">
          ${robot.ip || '--'}
        </span>
      </td>
      <td>
        <div class="hw-detail">${renderHwDetail(robot)}</div>
      </td>
    </tr>`;
}

function renderTable() {
  if (robots.length === 0) {
    tableBody.innerHTML = `
      <tr><td colspan="4" style="text-align:center;padding:24px">
        <span class="m3-text-on-surface-variant">データなし</span>
      </td></tr>`;
    return;
  }
  tableBody.innerHTML = robots.map(rowHtml).join('');
  renderSummary();
}

function renderSummary() {
  let run = 0, stop = 0, off = 0;
  for (const r of robots) {
    const cls = classifyStatus(r.status);
    if (cls === 'running') run++;
    else if (cls === 'stopped') stop++;
    else off++;
  }
  runningCount.textContent = String(run);
  stoppedCount.textContent = String(stop);
  offlineCount.textContent = String(off);
}

function updateLastUpdated() {
  const now = new Date();
  const hh = String(now.getHours()).padStart(2, '0');
  const mm = String(now.getMinutes()).padStart(2, '0');
  const ss = String(now.getSeconds()).padStart(2, '0');
  lastUpdatedEl.textContent = `最終更新: ${hh}:${mm}:${ss}`;
}

// ---- オフラインフォールバック ------------------------------------------------

function offlineRobots() {
  return Array.from({ length: NUM_ROBOTS }, (_, id) => ({
    robot_id: id,
    ip: `192.168.20.${100 + id}`,
    success: false,
    status: 'Offline',
  }));
}

// ---- API通信 ---------------------------------------------------------------

async function fetchRobots() {
  try {
    const res = await fetch('/api/robots', { cache: 'no-store' });
    if (!res.ok) throw new Error(`HTTP ${res.status}`);
    const data = await res.json();
    robots = data.robots || [];
    renderTable();
    setConnected(true);
    updateLastUpdated();
  } catch {
    setConnected(false);
    robots = offlineRobots();
    renderTable();
  }
}

async function controlRobot(robotId, command) {
  if (pendingControlIds.has(robotId)) return;
  pendingControlIds.add(robotId);
  renderTable();
  try {
    const res = await fetch(`/api/robots/${robotId}/${command}`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: '{}',
    });
    if (!res.ok) throw new Error(`HTTP ${res.status}`);
    const data = await res.json();
    const label = command === 'start' ? '起動' : '停止';
    const ok = data.success !== false;
    showToast(`ロボット #${robotId}: ${label}${ok ? '成功' : '失敗'}`, ok ? 'success' : 'error');
  } catch {
    showToast(`ロボット #${robotId}: 通信エラー`, 'error');
    setConnected(false);
  } finally {
    pendingControlIds.delete(robotId);
    await fetchRobots();
  }
}

window.controlRobot = controlRobot;

async function startAll() {
  try {
    startAllBtn.disabled = true;
    await Promise.all(
      Array.from({ length: NUM_ROBOTS }, (_, id) =>
        fetch(`/api/robots/${id}/start`, { method: 'POST' })
      )
    );
    await fetchRobots();
    showToast('全台起動コマンドを送信しました', 'success');
  } catch {
    showToast('一部のロボットへの通信に失敗しました', 'error');
    setConnected(false);
    robots = offlineRobots();
    renderTable();
  } finally {
    startAllBtn.disabled = false;
  }
}

async function stopAll() {
  const confirmed = await showConfirm(
    '全台停止',
    `全${NUM_ROBOTS}台のロボットを停止します。よろしいですか？`
  );
  if (!confirmed) return;
  try {
    stopAllBtn.disabled = true;
    await Promise.all(
      Array.from({ length: NUM_ROBOTS }, (_, id) =>
        fetch(`/api/robots/${id}/stop`, { method: 'POST' })
      )
    );
    await fetchRobots();
    showToast('全台停止コマンドを送信しました', 'info');
  } catch {
    showToast('一部のロボットへの通信に失敗しました', 'error');
    setConnected(false);
    robots = offlineRobots();
    renderTable();
  } finally {
    stopAllBtn.disabled = false;
  }
}

// ---- トースト通知 -----------------------------------------------------------

function showToast(message, type = 'info') {
  const iconMap = { success: 'check_circle', error: 'error', info: 'info' };
  const container = document.getElementById('toast-container');
  const toast = document.createElement('div');
  toast.className = `m3-toast m3-toast--${type}`;
  toast.innerHTML = `
    <span class="material-symbols-outlined icon-sm">${iconMap[type] ?? 'info'}</span>
    <span>${message}</span>`;
  container.appendChild(toast);
  setTimeout(() => {
    toast.classList.add('exit');
    setTimeout(() => toast.remove(), 300);
  }, 3000);
}

// ---- 確認ダイアログ ---------------------------------------------------------

function showConfirm(title, body) {
  return new Promise((resolve) => {
    const scrim = document.getElementById('confirm-scrim');
    const dialog = document.getElementById('confirm-dialog');
    document.getElementById('confirm-title').textContent = title;
    document.getElementById('confirm-body').textContent = body;

    scrim.style.display = 'block';
    dialog.style.display = 'block';
    // アニメーション起動
    requestAnimationFrame(() => {
      scrim.classList.add('open');
      dialog.classList.add('open');
    });

    function close(result) {
      scrim.classList.remove('open');
      dialog.classList.remove('open');
      setTimeout(() => {
        scrim.style.display = 'none';
        dialog.style.display = 'none';
      }, 200);
      okBtn.removeEventListener('click', onOk);
      cancelBtn.removeEventListener('click', onCancel);
      resolve(result);
    }

    const okBtn = document.getElementById('confirm-ok');
    const cancelBtn = document.getElementById('confirm-cancel');
    function onOk() { close(true); }
    function onCancel() { close(false); }

    okBtn.addEventListener('click', onOk);
    cancelBtn.addEventListener('click', onCancel);
    scrim.addEventListener('click', () => close(false), { once: true });
  });
}

// ---- イベントリスナー -------------------------------------------------------

refreshBtn.addEventListener('click', fetchRobots);
startAllBtn.addEventListener('click', startAll);
stopAllBtn.addEventListener('click', stopAll);

// ---- 初期化 ----------------------------------------------------------------

robots = offlineRobots();
renderTable();
fetchRobots();
setInterval(fetchRobots, 4000);
