/**
 * Ball Calibration UI - フロントエンドロジック
 */

'use strict';

// ===== グローバル状態 =====
const state = {
  trajectories: [],
  selectedIds: new Set(),
  previewId: null,
  optimizationResult: null,
  currentDecel: 0.70,
  kickOverrides: {},
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

async function apiPut(url, body) {
  const res = await fetch(url, {
    method: 'PUT',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(body),
  });
  if (!res.ok) throw new Error(res.statusText);
  return res.json();
}

// ===== タブ1: ファイル読み込み =====

// パス指定による読み込み
document.addEventListener('DOMContentLoaded', () => {
  document.getElementById('pathInput').addEventListener('keydown', e => {
    if (e.key === 'Enter') loadFromPath();
  });
});

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

// ドラッグ&ドロップ設定
const dropZone = document.getElementById('dropZone');
dropZone.addEventListener('dragover', e => {
  e.preventDefault();
  dropZone.classList.add('dragover');
});
dropZone.addEventListener('dragleave', () => {
  dropZone.classList.remove('dragover');
});
dropZone.addEventListener('drop', e => {
  e.preventDefault();
  dropZone.classList.remove('dragover');
  const file = e.dataTransfer.files[0];
  if (file) uploadFile(file);
});

function onFileSelected(input) {
  if (input.files[0]) uploadFile(input.files[0]);
}

async function uploadFile(file) {
  const ext = file.name.split('.').pop().toLowerCase();
  if (ext !== 'mcap' && ext !== 'db3') {
    showStatus('loadStatus', '対応形式: .mcap または .db3', 'warning');
    return;
  }

  showStatus('loadStatus', `アップロード中: ${file.name} (${(file.size / 1024 / 1024).toFixed(1)} MB)...`, 'info');

  const progress = document.getElementById('uploadProgress');
  const bar = document.getElementById('uploadProgressBar');
  progress.classList.remove('d-none');
  bar.style.width = '30%';

  const formData = new FormData();
  formData.append('file', file);

  try {
    bar.style.width = '60%';
    const res = await fetch('/api/upload', { method: 'POST', body: formData });
    bar.style.width = '90%';

    if (!res.ok) {
      const err = await res.json().catch(() => ({ detail: res.statusText }));
      throw new Error(err.detail || res.statusText);
    }

    const result = await res.json();
    bar.style.width = '100%';
    showStatus('loadStatus', `完了: ${result.loaded} 件の軌道データを抽出 (${file.name})`, 'success');
    await refreshTrajectoryList();
  } catch (e) {
    showStatus('loadStatus', `エラー: ${e.message}`, 'danger');
  } finally {
    setTimeout(() => {
      progress.classList.add('d-none');
      bar.style.width = '0%';
    }, 800);
    // reset file input
    document.getElementById('fileInput').value = '';
  }
}

async function refreshTrajectoryList() {
  state.trajectories = await apiGet('/api/trajectories');
  state.selectedIds = new Set(state.trajectories.map(t => t.event_id));

  const tbody = document.getElementById('trajectoryTable');
  tbody.innerHTML = '';

  state.trajectories.forEach(t => {
    const tr = document.createElement('tr');
    tr.dataset.eventId = t.event_id;
    tr.innerHTML = `
      <td><input type="checkbox" class="row-check" data-id="${t.event_id}" checked
        onchange="onRowCheck(${t.event_id}, this.checked)"></td>
      <td>${t.event_id}</td>
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

    Plotly.newPlot('previewVT', [{
      x: d.time_points, y: d.velocities,
      mode: 'lines+markers',
      marker: { size: 4, color: '#e8a838' },
      line: { color: '#e8a838', width: 2 },
      name: '速度',
    }], {
      ...PLOT_LAYOUT_BASE,
      title: { text: '速度 - 時間', font: { size: 12 } },
      xaxis: { ...PLOT_LAYOUT_BASE.xaxis, title: '時間 (s)' },
      yaxis: { ...PLOT_LAYOUT_BASE.yaxis, title: '速度 (m/s)' },
      margin: { ...PLOT_LAYOUT_BASE.margin, t: 25 },
    }, { displayModeBar: false, responsive: true });

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

// ===== タブ2: 最適化 & 調整 =====

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

    const result = await apiPost('/api/optimize', {
      enabled_event_ids: [...state.selectedIds],
      config,
    });

    state.optimizationResult = result;
    state.currentDecel = result.global_deceleration;

    const card = document.getElementById('resultSummaryCard');
    card.style.removeProperty('display');
    document.getElementById('resultDecel').textContent = result.global_deceleration.toFixed(4);
    document.getElementById('resultRMSE').textContent = result.global_rmse.toFixed(4);
    document.getElementById('resultR2').textContent = result.global_r_squared.toFixed(4);
    document.getElementById('resultUsed').textContent =
      `${result.trajectories_used} / ${result.trajectories_analyzed}`;

    document.getElementById('decelSlider').value = result.global_deceleration;
    document.getElementById('decelValue').textContent = result.global_deceleration.toFixed(2);

    renderKickMappingTable(result.power_velocity_summary);
    await updatePredictions();

  } catch (e) {
    alert(`最適化エラー: ${e.message}`);
  } finally {
    btn.disabled = false;
    spinner.classList.add('d-none');
  }
}

function renderKickMappingTable(summary) {
  const container = document.getElementById('kickMappingTable');
  if (!summary || Object.keys(summary).length === 0) {
    container.innerHTML = '<span class="text-muted small">マッピングデータなし</span>';
    return;
  }

  let html = '<div class="table-responsive"><table class="table table-sm mb-0">';
  html += '<thead class="table-dark"><tr><th>キックパワー</th><th>推定初速度 (m/s)</th><th>手動上書き</th></tr></thead><tbody>';

  Object.entries(summary).sort().forEach(([key, vel]) => {
    const power = (parseInt(key.replace('power_', '')) / 100).toFixed(2);
    const override = state.kickOverrides[key] !== undefined ? state.kickOverrides[key] : '';
    html += `<tr>
      <td>${power}</td>
      <td class="font-monospace">${vel.toFixed(4)}</td>
      <td><input type="number" class="form-control form-control-sm" style="width:100px"
        step="0.01" placeholder="${vel.toFixed(4)}" value="${override}"
        data-key="${key}" onchange="onKickOverrideChange('${key}', this.value)"></td>
    </tr>`;
  });

  html += '</tbody></table></div>';
  container.innerHTML = html;
}

function onKickOverrideChange(key, value) {
  if (value === '' || value === null) {
    delete state.kickOverrides[key];
  } else {
    state.kickOverrides[key] = parseFloat(value);
  }
  apiPut('/api/manual_params', { kick_power_overrides: state.kickOverrides });
}

function resetKickOverrides() {
  state.kickOverrides = {};
  if (state.optimizationResult) {
    renderKickMappingTable(state.optimizationResult.power_velocity_summary);
  }
  apiPut('/api/manual_params', { kick_power_overrides: {} });
}

const _onDecelChangeDebounced = debounce(async (value) => {
  state.currentDecel = parseFloat(value);
  await apiPut('/api/manual_params', { deceleration: state.currentDecel });
  await updatePredictions();
}, 200);

function onDecelChange(value) {
  document.getElementById('decelValue').textContent = parseFloat(value).toFixed(2);
  _onDecelChangeDebounced(value);
}

async function updatePredictions() {
  try {
    const result = await apiPost('/api/predict', {
      deceleration: state.currentDecel,
      event_ids: state.selectedIds.size > 0 ? [...state.selectedIds] : null,
    });
    state.predictedData = result.trajectories;
    renderVerifyCharts();
    renderAccuracyTable();
  } catch (e) {
    console.error('予測エラー:', e);
  }
}

// ===== タブ3: 可視化 & 検証 =====

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

  Plotly.newPlot('vtChart', vtTraces, {
    ...PLOT_LAYOUT_BASE,
    title: { text: `速度 vs 時間 (decel = ${state.currentDecel.toFixed(3)} m/s²)`, font: { size: 13 } },
    xaxis: { ...PLOT_LAYOUT_BASE.xaxis, title: '時間 (s)' },
    yaxis: { ...PLOT_LAYOUT_BASE.yaxis, title: '速度 (m/s)' },
    legend: { font: { size: 9 }, bgcolor: '#0f2744', bordercolor: '#2d4a7a' },
    margin: { ...PLOT_LAYOUT_BASE.margin, t: 40 },
  }, { responsive: true });

  if (state.optimizationResult && state.optimizationResult.kick_data) {
    const pwVel = state.optimizationResult.kick_data.filter(k => !k.is_chip_kick);
    const pvTrace = [{
      x: pwVel.map(k => k.kick_power),
      y: pwVel.map(k => k.estimated_initial_velocity),
      mode: 'markers',
      marker: { size: 7, color: '#a8d8ea', opacity: 0.8 },
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
        marker: { size: 8, color: '#e8a838' },
        name: '平均マッピング',
      });
    }

    Plotly.newPlot('powerVelocityChart', pvTrace, {
      ...PLOT_LAYOUT_BASE,
      title: { text: 'Kick Power vs 初速度', font: { size: 13 } },
      xaxis: { ...PLOT_LAYOUT_BASE.xaxis, title: 'キックパワー' },
      yaxis: { ...PLOT_LAYOUT_BASE.yaxis, title: '初速度 (m/s)' },
      legend: { font: { size: 9 }, bgcolor: '#0f2744' },
      margin: { ...PLOT_LAYOUT_BASE.margin, t: 40 },
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
      <td class="font-monospace small">[${k.confidence_lower.toFixed(3)}, ${k.confidence_upper.toFixed(3)}]</td>
    `;
    tbody.appendChild(tr);
  });
}

// ===== タブ4: エクスポート =====

async function refreshPreview() {
  try {
    const data = await apiGet('/api/export/preview');
    document.getElementById('yamlPreview').value = data.yaml || '';

    const la = data.launch_arrays;
    if (la && la.straight_kick_power_array) {
      document.getElementById('launchArrays').textContent =
        `straight_kick_power_array: [${la.straight_kick_power_array.join(', ')}]\n` +
        `straight_kick_speed_array:  [${la.straight_kick_speed_array.join(', ')}]`;
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

function debounce(fn, ms) {
  let timer;
  return (...args) => { clearTimeout(timer); timer = setTimeout(() => fn(...args), ms); };
}

// タブ切り替え時に可視化を更新
document.addEventListener('DOMContentLoaded', () => {
  document.querySelectorAll('[data-bs-toggle="tab"]').forEach(tab => {
    tab.addEventListener('shown.bs.tab', e => {
      const target = e.target.getAttribute('href');
      if (target === '#tab-verify') {
        renderVerifyCharts();
        renderAccuracyTable();
      } else if (target === '#tab-export') {
        refreshPreview();
      }
    });
  });
});
