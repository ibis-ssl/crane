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
  paper_bgcolor: '#0f2744',
  plot_bgcolor: '#0a1929',
  font: { color: '#e0e0e0', size: 11 },
  margin: { l: 45, r: 15, t: 30, b: 40 },
  xaxis: { gridcolor: '#1e3a5f', zerolinecolor: '#2d4a7a' },
  yaxis: { gridcolor: '#1e3a5f', zerolinecolor: '#2d4a7a' },
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
      <td><input type="checkbox" class="row-check" data-id="${t.event_id}" checked
        onchange="onRowCheck(${t.event_id}, this.checked)"></td>
      <td>${t.event_id}${hasRange ? ' <i class="fas fa-cut text-warning" title="時間範囲選択中"></i>' : ''}</td>
      <td>${t.kick_power.toFixed(2)}</td>
      <td><span class="badge ${t.is_chip_kick ? 'bg-warning text-dark' : 'bg-info'}">${t.is_chip_kick ? 'チップ' : 'ストレート'}</span></td>
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
    fillcolor: '#533483', opacity: 0.25, line: { width: 0 },
  };
}

function updateTimeRangeBadge(eventId) {
  const range = state.timeRanges[eventId];
  const info = document.getElementById('timeRangeInfo');
  if (range) {
    document.getElementById('timeRangeBadge').textContent =
      `${range[0].toFixed(3)}s ~ ${range[1].toFixed(3)}s`;
    info.classList.remove('d-none');
  } else {
    info.classList.add('d-none');
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
  const icon = idCell.querySelector('i');
  if (hasRange && !icon) {
    idCell.insertAdjacentHTML('beforeend', ' <i class="fas fa-cut text-warning" title="時間範囲選択中"></i>');
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
  badge.classList.remove('d-none');
  document.getElementById('previewEmpty').style.display = 'none';

  try {
    const d = await apiGet(`/api/trajectory/${eventId}`);

    Plotly.newPlot('previewXY', [{
      x: d.positions_x, y: d.positions_y,
      mode: 'lines+markers',
      marker: { size: 4, color: '#a8d8ea' },
      line: { color: '#533483', width: 2 },
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
      marker: { size: 4, color: '#e8a838' },
      line: { color: '#e8a838', width: 2 },
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
  spinner.classList.remove('d-none');

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
    card.style.removeProperty('display');
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

    document.getElementById('verifyCard').style.removeProperty('display');
    document.getElementById('pvCard').style.removeProperty('display');
    document.getElementById('accuracyCard').style.removeProperty('display');

    renderVerifyCharts();
    renderAccuracyTable();
    await refreshPreview();

  } catch (e) {
    alert(`最適化エラー: ${e.message}`);
  } finally {
    btn.disabled = false;
    spinner.classList.add('d-none');
  }
}

// ===== 検証チャート =====

function renderVerifyCharts() {
  if (!state.predictedData || state.predictedData.length === 0) return;

  const colors = ['#a8d8ea', '#e8a838', '#ff6b6b', '#6bcb77', '#c77dff',
    '#4cc9f0', '#f72585', '#b5e48c', '#fca311', '#64dfdf'];

  const vtTraces = [];
  state.predictedData.forEach((traj, i) => {
    const color = colors[i % colors.length];
    vtTraces.push({
      x: traj.time_points, y: traj.actual_velocities,
      mode: 'markers', marker: { size: 3, color, opacity: 0.6 },
      name: `#${traj.event_id} (p=${traj.kick_power.toFixed(2)})`,
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
    legend: { font: { size: 9 }, bgcolor: '#0f2744', bordercolor: '#2d4a7a' },
    margin: { ...PLOT_LAYOUT_BASE.margin, t: 35 },
  }, { responsive: true });

  if (state.optimizationResult && state.optimizationResult.kick_data) {
    const pwVel = state.optimizationResult.kick_data.filter(k => !k.is_chip_kick);
    const pvTrace = [{
      x: pwVel.map(k => k.kick_power),
      y: pwVel.map(k => k.estimated_initial_velocity),
      mode: 'markers',
      marker: { size: 6, color: '#a8d8ea', opacity: 0.8 },
      text: pwVel.map(k => `#${k.event_id}<br>R²=${k.fitting_r_squared.toFixed(3)}`),
      hovertemplate: '%{text}<br>power=%{x:.2f}<br>v0=%{y:.3f} m/s<extra></extra>',
      name: '推定初速度',
    }];

    const summary = state.optimizationResult.power_velocity_summary;
    if (summary) {
      const items = Object.entries(summary).sort();
      pvTrace.push({
        x: items.map(([k]) => parseInt(k.replace('power_', '')) / 100),
        y: items.map(([, v]) => v),
        mode: 'lines+markers',
        line: { color: '#e8a838', width: 2 },
        marker: { size: 6, color: '#e8a838' },
        name: '平均マッピング',
      });
    }

    Plotly.newPlot('powerVelocityChart', pvTrace, {
      ...PLOT_LAYOUT_BASE,
      title: { text: 'Kick Power vs 初速度', font: { size: 11 } },
      xaxis: { ...PLOT_LAYOUT_BASE.xaxis, title: 'キックパワー' },
      yaxis: { ...PLOT_LAYOUT_BASE.yaxis, title: '初速度 (m/s)' },
      legend: { font: { size: 8 }, bgcolor: '#0f2744' },
      margin: { l: 45, r: 10, t: 30, b: 40 },
    }, { responsive: true });
  }
}

function renderAccuracyTable() {
  if (!state.optimizationResult) return;
  const tbody = document.getElementById('accuracyTable');
  tbody.innerHTML = '';

  state.optimizationResult.kick_data.forEach(k => {
    const r2Color = k.fitting_r_squared >= 0.8 ? 'text-success' :
      k.fitting_r_squared >= 0.6 ? 'text-warning' : 'text-danger';
    const tr = document.createElement('tr');
    tr.innerHTML = `
      <td>${k.event_id}</td>
      <td>${k.kick_power.toFixed(2)}</td>
      <td class="font-monospace">${k.estimated_initial_velocity.toFixed(4)}</td>
      <td class="font-monospace ${r2Color}">${k.fitting_r_squared.toFixed(4)}</td>
      <td>${k.trajectory_duration.toFixed(2)}</td>
    `;
    tbody.appendChild(tr);
  });
}

// ===== エクスポート =====

async function refreshPreview() {
  try {
    const data = await apiGet('/api/export/preview');
    const la = data.launch_arrays;
    if (la && la.straight_kick_power_array) {
      document.getElementById('launchArrays').textContent =
        `straight_kick_power_array: [${la.straight_kick_power_array.join(', ')}]\n` +
        `straight_kick_speed_array:  [${la.straight_kick_speed_array.join(', ')}]`;
      document.getElementById('launchArraysCard').style.removeProperty('display');
    }
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
    span.className = 'text-success';
    span.textContent = '✓ calibrated_ball_physics.yaml をダウンロードしました';
    status.replaceChildren(span);
  } catch (e) {
    const span = document.createElement('span');
    span.className = 'text-danger';
    span.textContent = `エラー: ${e.message}`;
    status.replaceChildren(span);
  }
}

// ===== ユーティリティ =====

function showStatus(elementId, message, type) {
  const el = document.getElementById(elementId);
  if (!el) return;
  const span = document.createElement('span');
  span.className = `text-${type}`;
  span.textContent = message;
  el.replaceChildren(span);
}
