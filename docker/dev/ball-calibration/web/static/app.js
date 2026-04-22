/**
 * Ball Calibration UI - フロントエンドロジック
 */

'use strict';

// ===== グローバル状態 =====
const state = {
  trajectories: [],
  selectedIds: new Set(),
  previewId: null,
  timeRanges: {},       // { [eventId]: [start, end] }
  optimizationResult: null,
  predictedData: [],
};

const PLOT_LAYOUT_BASE = {
  paper_bgcolor: '#141C2E',
  plot_bgcolor: '#0D1117',
  font: { color: '#E2E4E8', size: 11 },
  margin: { l: 45, r: 15, t: 30, b: 40 },
  xaxis: { gridcolor: '#2D3748', zerolinecolor: '#4A5568' },
  yaxis: { gridcolor: '#2D3748', zerolinecolor: '#4A5568' },
};

const timelineState = {
  range: null,         // [start_s, end_s]
  totalDuration: 0,
};

// ===== API ヘルパー =====
async function apiPost(url, body) {
  const res = await fetch(url, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(body),
  });
  if (!res.ok) {
    const err = await res.json().catch(() => ({ detail: res.statusText }));
    throw new Error(err.detail || res.statusText);
  }
  return res.json();
}

async function apiGet(url) {
  const res = await fetch(url);
  if (!res.ok) {
    const err = await res.json().catch(() => ({ detail: res.statusText }));
    throw new Error(err.detail || res.statusText);
  }
  return res.json();
}

// ===== ファイル読み込み =====

document.addEventListener('DOMContentLoaded', () => {
  document.getElementById('pathInput').addEventListener('keydown', e => {
    if (e.key === 'Enter') loadFromPath();
  });
});

async function openSystemFileDialog() {
  const currentPath = document.getElementById('pathInput').value.trim() || '/home';
  showStatus('loadStatus', 'ファイル選択ダイアログを開いています...', 'info');
  try {
    const data = await apiGet(`/api/file_dialog?start_path=${encodeURIComponent(currentPath)}`);
    document.getElementById('pathInput').value = data.path;
    showStatus('loadStatus', `選択: ${data.path}`, 'secondary');
    await loadFromPath();
  } catch (e) {
    if (e.message.includes('キャンセル')) {
      showStatus('loadStatus', '', '');
    } else {
      showStatus('loadStatus', `エラー: ${e.message}`, 'danger');
    }
  }
}

async function loadFromPath() {
  const path = document.getElementById('pathInput').value.trim();
  if (!path) {
    showStatus('loadStatus', 'ファイルパスを入力してください', 'warning');
    return;
  }
  showStatus('loadStatus', `読み込み中: ${path} ...`, 'info');
  try {
    const result = await apiPost('/api/load_path', { path });
    showStatus('loadStatus', `完了: ${result.loaded} 件の軌道データを抽出 (${result.filename})`, 'success');
    await refreshTrajectoryList();
    await loadTimeline();
  } catch (e) {
    showStatus('loadStatus', `エラー: ${e.message}`, 'danger');
  }
}

// ===== 軌道一覧 =====

async function refreshTrajectoryList() {
  state.trajectories = await apiGet('/api/trajectories');
  state.selectedIds = new Set(state.trajectories.map(t => t.event_id));

  const tbody = document.getElementById('trajectoryTable');
  tbody.innerHTML = '';

  state.trajectories.forEach(t => {
    const hasRange = !!state.timeRanges[t.event_id];
    const tr = document.createElement('tr');
    tr.dataset.eventId = t.event_id;
    tr.innerHTML = `
      <td><input type="checkbox" class="m3-checkbox row-check" data-id="${t.event_id}" checked
        onchange="onRowCheck(${t.event_id}, this.checked)"></td>
      <td>${t.event_id}${hasRange ? ' <span class="material-symbols-outlined m3-text-warning" style="font-size:14px;vertical-align:middle;" title="時間範囲選択中">content_cut</span>' : ''}</td>
      <td>${t.data_points}</td>
      <td>${t.duration.toFixed(2)}</td>
      <td>${t.max_velocity.toFixed(2)}</td>
    `;
    tr.addEventListener('click', e => {
      if (e.target.type !== 'checkbox') showTrajectoryPreview(t.event_id);
    });
    tbody.appendChild(tr);
  });

  document.getElementById('trajectoryCount').textContent = `${state.trajectories.length}件`;
}

// ===== 軌道プレビュー & 時間範囲選択 =====

function makeRangeShape([x0, x1]) {
  return {
    type: 'rect', xref: 'x', yref: 'paper',
    x0, x1, y0: 0, y1: 1,
    fillcolor: '#4F378B', opacity: 0.25, line: { width: 0 },
  };
}

function updateTimeRangeBadge(eventId) {
  const range = state.timeRanges[eventId];
  const info = document.getElementById('timeRangeInfo');
  if (range) {
    document.getElementById('timeRangeBadge').textContent =
      `${range[0].toFixed(3)}s ~ ${range[1].toFixed(3)}s`;
    info.classList.remove('m3-hidden');
  } else {
    info.classList.add('m3-hidden');
  }
}

function resetCurrentTimeRange() {
  if (state.previewId === null) return;
  delete state.timeRanges[state.previewId];
  Plotly.relayout('previewVT', { shapes: [] });
  updateTimeRangeBadge(state.previewId);
  updateTrajectoryRowIcon(state.previewId, false);
}

function updateTrajectoryRowIcon(eventId, hasRange) {
  const tr = document.querySelector(`#trajectoryTable tr[data-event-id="${eventId}"]`);
  if (!tr) return;
  const idCell = tr.querySelector('td:nth-child(2)');
  if (!idCell) return;
  const icon = idCell.querySelector('span.material-symbols-outlined');
  if (hasRange && !icon) {
    idCell.insertAdjacentHTML('beforeend', ' <span class="material-symbols-outlined m3-text-warning" style="font-size:14px;vertical-align:middle;" title="時間範囲選択中">content_cut</span>');
  } else if (!hasRange && icon) {
    icon.remove();
  }
}

async function showTrajectoryPreview(eventId) {
  state.previewId = eventId;
  document.querySelectorAll('#trajectoryTable tr').forEach(tr => {
    tr.classList.toggle('selected-row', parseInt(tr.dataset.eventId) === eventId);
  });

  const badge = document.getElementById('previewEventId');
  badge.textContent = `Event #${eventId}`;
  badge.classList.remove('m3-hidden');
  document.getElementById('previewEmpty').classList.add('m3-hidden');

  try {
    const d = await apiGet(`/api/trajectory/${eventId}`);

    Plotly.newPlot('previewXY', [{
      x: d.positions_x, y: d.positions_y,
      mode: 'lines+markers',
      marker: { size: 4, color: '#A0C4FF' },
      line: { color: '#4F378B', width: 2 },
      name: 'XY軌跡',
    }], {
      ...PLOT_LAYOUT_BASE,
      title: { text: 'XY軌跡', font: { size: 12 } },
      xaxis: { ...PLOT_LAYOUT_BASE.xaxis, title: 'X (m)' },
      yaxis: { ...PLOT_LAYOUT_BASE.yaxis, title: 'Y (m)', scaleanchor: 'x' },
      margin: { ...PLOT_LAYOUT_BASE.margin, t: 25 },
    }, { displayModeBar: false, responsive: true });

    const existing = state.timeRanges[eventId];
    const vtLayout = {
      ...PLOT_LAYOUT_BASE,
      title: { text: '速度 - 時間 (ドラッグで範囲選択)', font: { size: 12 } },
      dragmode: 'select',
      selectdirection: 'h',
      xaxis: { ...PLOT_LAYOUT_BASE.xaxis, title: '時間 (s)' },
      yaxis: { ...PLOT_LAYOUT_BASE.yaxis, title: '速度 (m/s)' },
      margin: { ...PLOT_LAYOUT_BASE.margin, t: 35 },
      shapes: existing ? [makeRangeShape(existing)] : [],
    };

    Plotly.newPlot('previewVT', [{
      x: d.time_points, y: d.velocities,
      mode: 'lines+markers',
      marker: { size: 4, color: '#FFA726' },
      line: { color: '#FFA726', width: 2 },
      name: '速度',
    }], vtLayout, { displayModeBar: false, responsive: true });

    const vt = document.getElementById('previewVT');
    vt.on('plotly_selected', (ev) => {
      if (!ev || !ev.range) return;
      const [x0, x1] = ev.range.x;
      state.timeRanges[eventId] = [x0, x1];
      Plotly.relayout('previewVT', { shapes: [makeRangeShape([x0, x1])] });
      updateTimeRangeBadge(eventId);
      updateTrajectoryRowIcon(eventId, true);
    });
    vt.on('plotly_doubleclick', () => {
      delete state.timeRanges[eventId];
      Plotly.relayout('previewVT', { shapes: [] });
      updateTimeRangeBadge(eventId);
      updateTrajectoryRowIcon(eventId, false);
    });

    updateTimeRangeBadge(eventId);

  } catch (e) {
    console.error('プレビューエラー:', e);
  }
}

function onRowCheck(eventId, checked) {
  if (checked) state.selectedIds.add(eventId);
  else state.selectedIds.delete(eventId);
}

function selectAll(checked) {
  document.querySelectorAll('.row-check').forEach(cb => {
    cb.checked = checked;
    const id = parseInt(cb.dataset.id);
    if (checked) state.selectedIds.add(id);
    else state.selectedIds.delete(id);
  });
  document.getElementById('checkAll').checked = checked;
}

// ===== 最適化 =====

async function runOptimization() {
  const btn = document.getElementById('optimizeBtn');
  const spinner = document.getElementById('optimizeSpinner');
  btn.disabled = true;
  spinner.classList.remove('m3-hidden');

  try {
    const config = {
      min_trajectory_duration: parseFloat(document.getElementById('cfgMinDuration').value),
      min_fitting_r_squared: parseFloat(document.getElementById('cfgMinR2').value),
      min_data_points_per_trajectory: parseInt(document.getElementById('cfgMinPoints').value),
    };

    // 選択済み軌道のみ time_ranges を送信
    const timeRangesPayload = {};
    for (const id of state.selectedIds) {
      if (state.timeRanges[id]) timeRangesPayload[id] = state.timeRanges[id];
    }

    const result = await apiPost('/api/optimize', {
      enabled_event_ids: [...state.selectedIds],
      time_ranges: Object.keys(timeRangesPayload).length ? timeRangesPayload : null,
      config,
    });

    state.optimizationResult = result;

    // 結果サマリー表示
    const card = document.getElementById('resultSummaryCard');
    card.classList.remove('m3-hidden');
    document.getElementById('resultDecel').textContent = result.global_deceleration.toFixed(4);
    document.getElementById('resultRMSE').textContent = result.global_rmse.toFixed(4);
    document.getElementById('resultR2').textContent = result.global_r_squared.toFixed(4);
    document.getElementById('resultUsed').textContent =
      `${result.trajectories_used} / ${result.trajectories_analyzed}`;

    // 検証チャート・精度テーブルを自動描画
    const predResult = await apiPost('/api/predict', {
      deceleration: result.global_deceleration,
      event_ids: state.selectedIds.size > 0 ? [...state.selectedIds] : null,
    });
    state.predictedData = predResult.trajectories;

    document.getElementById('verifyCard').classList.remove('m3-hidden');
    document.getElementById('accuracyCard').classList.remove('m3-hidden');

    renderVerifyCharts();
    renderAccuracyTable();
    await refreshPreview();

  } catch (e) {
    alert(`最適化エラー: ${e.message}`);
  } finally {
    btn.disabled = false;
    spinner.classList.add('m3-hidden');
  }
}

// ===== 検証チャート =====

function renderVerifyCharts() {
  if (!state.predictedData || state.predictedData.length === 0) return;

  const colors = ['#A0C4FF', '#FFA726', '#FFB4AB', '#66BB6A', '#D0BCFF',
    '#4cc9f0', '#f72585', '#b5e48c', '#fca311', '#64dfdf'];

  const vtTraces = [];
  state.predictedData.forEach((traj, i) => {
    const color = colors[i % colors.length];
    vtTraces.push({
      x: traj.time_points, y: traj.actual_velocities,
      mode: 'markers', marker: { size: 3, color, opacity: 0.6 },
      name: `#${traj.event_id}`,
      legendgroup: `g${traj.event_id}`,
      showlegend: true,
    });
    vtTraces.push({
      x: traj.time_points, y: traj.predicted_velocities,
      mode: 'lines', line: { color, width: 2, dash: 'dash' },
      name: `#${traj.event_id} 予測`,
      legendgroup: `g${traj.event_id}`,
      showlegend: false,
    });
  });

  const decel = state.optimizationResult ? state.optimizationResult.global_deceleration : 0;
  Plotly.newPlot('vtChart', vtTraces, {
    ...PLOT_LAYOUT_BASE,
    title: { text: `速度 vs 時間 (decel = ${decel.toFixed(3)} m/s²)`, font: { size: 12 } },
    xaxis: { ...PLOT_LAYOUT_BASE.xaxis, title: '時間 (s)' },
    yaxis: { ...PLOT_LAYOUT_BASE.yaxis, title: '速度 (m/s)' },
    legend: { font: { size: 9 }, bgcolor: '#141C2E', bordercolor: '#2D3748' },
    margin: { ...PLOT_LAYOUT_BASE.margin, t: 35 },
  }, { responsive: true });

}

function renderAccuracyTable() {
  if (!state.optimizationResult) return;
  const tbody = document.getElementById('accuracyTable');
  tbody.innerHTML = '';

  const fits = state.optimizationResult.per_trajectory_fits || [];
  fits.forEach(k => {
    if (k.rejected) return;
    const r2Color = k.r_squared >= 0.8 ? 'm3-text-success' :
      k.r_squared >= 0.6 ? 'm3-text-warning' : 'm3-text-error';
    const tr = document.createElement('tr');
    tr.innerHTML = `
      <td>${k.event_id}</td>
      <td class="m3-text-mono">${k.v0.toFixed(4)}</td>
      <td class="m3-text-mono ${r2Color}">${k.r_squared.toFixed(4)}</td>
      <td>${(k.ci_v0 ? ((k.ci_v0[1] - k.ci_v0[0]) / 2).toFixed(2) : '-')}</td>
    `;
    tbody.appendChild(tr);
  });
}

// ===== エクスポート =====

async function refreshPreview() {
  try {
    await apiGet('/api/export/preview');
  } catch (e) {
    console.error('プレビュー更新エラー:', e);
  }
}

async function downloadYaml() {
  const status = document.getElementById('exportStatus');
  try {
    const res = await fetch('/api/export/download');
    if (!res.ok) {
      const err = await res.json().catch(() => ({ detail: res.statusText }));
      throw new Error(err.detail || res.statusText);
    }

    const blob = await res.blob();
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = 'calibrated_ball_physics.yaml';
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    URL.revokeObjectURL(url);

    const span = document.createElement('span');
    span.className = 'm3-text-success';
    span.textContent = '✓ calibrated_ball_physics.yaml をダウンロードしました';
    status.replaceChildren(span);
  } catch (e) {
    const span = document.createElement('span');
    span.className = 'm3-text-error';
    span.textContent = `エラー: ${e.message}`;
    status.replaceChildren(span);
  }
}

// ===== ログ全体タイムライン =====

async function loadTimeline() {
  const chartEl = document.getElementById('timelineChart');
  const emptyEl = document.getElementById('timelineEmpty');
  const controlsEl = document.getElementById('timelineControls');
  const statusEl = document.getElementById('timelineStatus');

  try {
    const data = await apiGet('/api/timeline');
    if (!data.time_points || data.time_points.length === 0) {
      emptyEl.classList.remove('m3-hidden');
      controlsEl.classList.add('m3-hidden');
      statusEl.textContent = '';
      return;
    }

    timelineState.totalDuration = data.total_duration;
    timelineState.range = null;

    emptyEl.classList.add('m3-hidden');
    controlsEl.classList.remove('m3-hidden');
    statusEl.textContent =
      `${data.downsampled_to} / ${data.total_samples} 点 (${data.total_duration.toFixed(1)}s)`;

    Plotly.newPlot('timelineChart', [{
      x: data.time_points,
      y: data.velocities,
      mode: 'lines',
      line: { color: '#A0C4FF', width: 1 },
      name: 'ボール速度',
      hovertemplate: 't=%{x:.3f}s<br>v=%{y:.3f}m/s<extra></extra>',
    }], {
      ...PLOT_LAYOUT_BASE,
      dragmode: 'select',
      selectdirection: 'h',
      xaxis: { ...PLOT_LAYOUT_BASE.xaxis, title: '時間 (s)' },
      yaxis: { ...PLOT_LAYOUT_BASE.yaxis, title: '速度 (m/s)' },
      margin: { l: 45, r: 15, t: 10, b: 40 },
      shapes: [],
    }, {
      responsive: true,
      displayModeBar: true,
      displaylogo: false,
      modeBarButtonsToRemove: ['lasso2d', 'autoScale2d', 'toggleSpikelines'],
    });

    chartEl.on('plotly_selected', onTimelineSelected);
    chartEl.on('plotly_deselect', () => {
      timelineState.range = null;
      updateTimelineRangeUI();
      Plotly.relayout('timelineChart', { shapes: [] });
    });

    updateTimelineRangeUI();
  } catch (e) {
    console.error('タイムライン読み込みエラー:', e);
    statusEl.textContent = `エラー: ${e.message}`;
  }
}

function onTimelineSelected(ev) {
  if (!ev || !ev.range) return;
  const [x0, x1] = ev.range.x;
  const start = Math.min(x0, x1);
  const end = Math.max(x0, x1);
  timelineState.range = [start, end];
  Plotly.relayout('timelineChart', { shapes: [makeRangeShape([start, end])] });
  updateTimelineRangeUI();
}

function updateTimelineRangeUI() {
  const badge = document.getElementById('timelineRangeBadge');
  const btn = document.getElementById('addTrajBtn');
  if (timelineState.range) {
    const [a, b] = timelineState.range;
    badge.textContent = `${a.toFixed(3)}s ~ ${b.toFixed(3)}s (${(b - a).toFixed(3)}s)`;
    badge.classList.remove('m3-badge--secondary');
    badge.classList.add('m3-badge--warning');
    btn.disabled = false;
  } else {
    badge.textContent = '未選択';
    badge.classList.remove('m3-badge--warning');
    badge.classList.add('m3-badge--secondary');
    btn.disabled = true;
  }
}

function clearTimelineSelection() {
  timelineState.range = null;
  Plotly.relayout('timelineChart', { shapes: [] });
  try { Plotly.restyle('timelineChart', { selectedpoints: [null] }); } catch (_) { /* noop */ }
  updateTimelineRangeUI();
  document.getElementById('addTrajStatus').replaceChildren();
}

async function addTrajectoryFromTimeline() {
  if (!timelineState.range) return;
  const [start, end] = timelineState.range;
  const status = document.getElementById('addTrajStatus');

  try {
    const res = await apiPost('/api/add_trajectory', {
      start_time: start,
      end_time: end,
    });
    const span = document.createElement('span');
    span.className = 'm3-text-success';
    span.textContent =
      `✓ 軌道 #${res.event_id} を追加 (${res.data_points}点, ${res.duration.toFixed(2)}s, 最大速度 ${res.max_velocity.toFixed(2)}m/s)`;
    status.replaceChildren(span);
    await refreshTrajectoryList();
  } catch (e) {
    const span = document.createElement('span');
    span.className = 'm3-text-error';
    span.textContent = `エラー: ${e.message}`;
    status.replaceChildren(span);
  }
}

// ===== ユーティリティ =====

const _M3_TYPE = { danger: 'error', secondary: 'on-surface-variant' };

function showStatus(elementId, message, type) {
  const el = document.getElementById(elementId);
  if (!el) return;
  const span = document.createElement('span');
  const m3Type = _M3_TYPE[type] || type;
  span.className = m3Type ? `m3-text-${m3Type}` : '';
  span.textContent = message;
  el.replaceChildren(span);
}
