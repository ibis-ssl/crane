/**
 * Packet Forge — robot_packet.h の 64 バイトを組み立てて直接 UDP で送る GUI。
 *
 * フィールド表はここに持たない。/api/schema がバックエンド経由で robot_packet.h
 * 由来の定義を返すので、それを描く。ハードコードするとヘッダが動いたときに黙ってずれる。
 *
 * 送信ループはサーバ側にある。このページは spec を更新するだけで、1 フレームずつ
 * 送ったりはしない（WebSocket が一瞬詰まると check_counter が止まり AI 断になる）。
 */

const PUSH_DEBOUNCE_MS = 80;

const state = {
  schema: null,
  spec: {
    target: 'real',
    robot_id: 0,
    rate_hz: 62.5,
    base: 'neutral',
    broadcast: false,
    fields: {},
    raw_bytes: {},
  },
  snapshot: null,
  command: new Uint8Array(64),
  highlightBytes: [],
  dragStart: null,
  errors: [],
};

const $ = (id) => document.getElementById(id);

// ===== API =====

async function api(path, payload) {
  const options = payload === undefined
    ? {}
    : { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(payload) };
  const response = await fetch(path, options);
  const data = await response.json().catch(() => ({}));
  if (!response.ok) throw new Error(data.error || `${path}: ${response.status}`);
  return data;
}

let pushTimer = null;
function pushSpec() {
  clearTimeout(pushTimer);
  pushTimer = setTimeout(async () => {
    try {
      applySnapshot(await api('/api/spec', state.spec));
    } catch (err) {
      showError(err.message);
    }
  }, PUSH_DEBOUNCE_MS);
}

// ===== 状態の反映 =====

function applySnapshot(snapshot) {
  state.snapshot = snapshot;
  state.command = hexToBytes(snapshot.command_hex);
  renderWarnings(snapshot);
  renderBytes();
  renderValues(snapshot.fields);
  renderSenderStats(snapshot.sender);
  renderFeedback(snapshot.feedback);
  if (!$('steps-panel').hidden) renderSteps();
  drawField();
}

function hexToBytes(hex) {
  const out = new Uint8Array(hex.length / 2);
  for (let i = 0; i < out.length; i += 1) out[i] = parseInt(hex.substr(i * 2, 2), 16);
  return out;
}

// 操作エラーは state に持つ。renderWarnings が 100ms ごとに箱を作り直すので、
// DOM へ直接足すと次のティックで消えて誰も読めない。
function showError(message) {
  state.errors.push({ message, until: Date.now() + 8000 });
  renderWarnings(state.snapshot || { warnings: [], conflicts: [] });
}

function renderWarnings(snapshot) {
  const box = $('warnings');
  box.innerHTML = '';
  state.errors = state.errors.filter((e) => e.until > Date.now());
  for (const error of state.errors) box.appendChild(warningEl(error.message, true));
  for (const conflict of snapshot.conflicts || []) {
    box.appendChild(warningEl(
      `crane の ibis_sender_node が動いている。2 つの送信元が同じ CM4 へ送ると check_counter が入り乱れ、測定結果そのものが壊れる — ${conflict}`,
      true,
    ));
  }
  for (const warning of snapshot.warnings || []) box.appendChild(warningEl(warning, false));
}

function warningEl(text, isConflict) {
  const div = document.createElement('div');
  div.className = 'pf-warning' + (isConflict ? ' pf-warning--conflict' : '');
  div.innerHTML = '<span class="material-symbols-outlined icon-sm">warning</span><span></span>';
  div.lastElementChild.textContent = text;
  return div;
}

// ===== フィールド編集ペイン =====

function activeModeName() {
  // spec に明示が無いとき（初期表示）はサーバが組み立てたバイトを見る。
  // ここで undefined を返すと mode_args の両方が薄く見え、選択中のモードも分からない。
  const decoded = (state.snapshot?.fields || []).find((f) => f.key === 'control_mode');
  const modeValue = state.spec.fields.control_mode ?? decoded?.value;
  const modes = state.schema.control_modes;
  return Object.keys(modes).find((name) => modes[name] === modeValue) ?? null;
}

function currentModeValue() {
  const decoded = (state.snapshot?.fields || []).find((f) => f.key === 'control_mode');
  return state.spec.fields.control_mode ?? decoded?.value;
}

function buildEditor() {
  const editor = $('editor');
  editor.innerHTML = '';
  const byGroup = new Map();
  for (const field of state.schema.fields) {
    if (!byGroup.has(field.group)) byGroup.set(field.group, []);
    byGroup.get(field.group).push(field);
  }

  for (const [group, fields] of byGroup) {
    const section = document.createElement('div');
    section.className = 'pf-group';
    section.dataset.group = group;
    const title = document.createElement('div');
    title.className = 'pf-group__title';
    title.textContent = state.schema.group_labels[group] || group;
    section.appendChild(title);

    if (group === 'flags') section.appendChild(buildFlagGrid(fields));
    else if (group === 'mode') fields.forEach((f) => section.appendChild(buildModeRow(f)));
    else fields.forEach((f) => section.appendChild(buildRow(f)));

    editor.appendChild(section);
  }
}

/** フラグは 8 ビット全部を独立に出す。未定義ビットも立てられる。 */
function buildFlagGrid(fields) {
  const grid = document.createElement('div');
  grid.className = 'pf-flags';
  for (const field of fields) {
    const label = document.createElement('label');
    label.className = 'pf-flag' + (field.note ? ' pf-flag--undefined' : '');
    label.title = field.note || `${field.key} — byte ${field.offset} bit ${field.bit}`;
    const input = document.createElement('input');
    input.type = 'checkbox';
    input.className = 'm3-checkbox';
    input.dataset.key = field.key;
    input.addEventListener('change', () => {
      state.spec.fields[field.key] = input.checked;
      pushSpec();
    });
    const span = document.createElement('span');
    span.className = 'pf-flag__label';
    span.textContent = `${field.bit}: ${field.label}`;
    label.append(input, span);
    grid.appendChild(label);
    highlightOnHover(label, field.bytes);
  }
  return grid;
}

/** control_mode はセグメントボタン。押すと mode_args の有効側が入れ替わる。 */
function buildModeRow(field) {
  const wrap = document.createElement('div');
  wrap.className = 'pf-seg';
  for (const choice of field.choices) {
    const button = document.createElement('button');
    button.className = 'm3-btn m3-btn--outlined m3-btn--sm';
    button.dataset.modeValue = String(choice.value);
    button.textContent = `${choice.value}: ${choice.name.replace(/_MODE$/, '').toLowerCase()}`;
    button.title = field.note;
    button.addEventListener('click', () => {
      state.spec.fields.control_mode = choice.value;
      syncModeButtons();
      markInactiveRows();
      pushSpec();
    });
    wrap.appendChild(button);
  }
  const note = document.createElement('div');
  note.className = 'pf-row__note';
  note.style.padding = '0 10px 6px';
  note.textContent = field.note;
  const holder = document.createElement('div');
  holder.append(wrap, note);
  highlightOnHover(wrap, field.bytes);
  return holder;
}

function syncModeButtons() {
  const current = currentModeValue();
  document.querySelectorAll('[data-mode-value]').forEach((button) => {
    const selected = Number(button.dataset.modeValue) === current;
    button.classList.toggle('m3-btn--filled', selected);
    button.classList.toggle('m3-btn--outlined', !selected);
  });
}

/** 連続値はスライダー + 数値入力のペア。整数は数値入力のみ。 */
function buildRow(field) {
  const row = document.createElement('div');
  row.className = 'pf-row';
  row.dataset.key = field.key;
  if (field.mode) row.dataset.mode = field.mode;

  const head = document.createElement('div');
  head.className = 'pf-row__head';
  const label = document.createElement('span');
  label.className = 'pf-row__label';
  label.textContent = field.label;
  const key = document.createElement('span');
  key.className = 'pf-row__key';
  const span = field.bytes.length === 1 ? `b${field.bytes[0]}` : `b${field.bytes[0]}-${field.bytes[field.bytes.length - 1]}`;
  key.textContent = `${field.key}  ${span}`;
  head.append(label, key);

  const controls = document.createElement('div');
  controls.className = 'pf-row__controls';

  const isFloat = field.kind === 'two_byte' || field.kind === 'u8_scaled';
  const number = document.createElement('input');
  number.type = 'number';
  number.className = 'm3-text-input pf-row__num';
  number.step = String(field.ui_step ?? (isFloat ? 0.01 : 1));
  number.dataset.key = field.key;
  number.dataset.role = 'number';

  let slider = null;
  if (isFloat) {
    slider = document.createElement('input');
    slider.type = 'range';
    slider.min = String(field.ui_min ?? -1);
    slider.max = String(field.ui_max ?? 1);
    slider.step = String(field.ui_step ?? 0.01);
    slider.dataset.key = field.key;
    slider.dataset.role = 'slider';
    slider.addEventListener('input', () => {
      number.value = slider.value;
      commitValue(field, Number(slider.value));
    });
    controls.appendChild(slider);
  }

  number.addEventListener('input', () => {
    if (number.value === '') return;
    // 数値入力はワイヤ上のレンジまで受ける。スライダーの推奨範囲は見た目だけの都合。
    if (slider) slider.value = number.value;
    commitValue(field, Number(number.value));
  });
  controls.appendChild(number);

  const unit = document.createElement('span');
  unit.className = 'pf-row__unit';
  unit.textContent = field.unit || '';
  controls.appendChild(unit);

  const raw = document.createElement('span');
  raw.className = 'pf-row__raw';
  raw.dataset.role = 'raw';
  raw.dataset.key = field.key;  // renderValues は data-key で引くので必須
  controls.appendChild(raw);

  row.append(head, controls);
  if (field.note) {
    const note = document.createElement('div');
    note.className = 'pf-row__note';
    note.textContent = field.note;
    row.appendChild(note);
  }
  highlightOnHover(row, field.bytes);
  return row;
}

function commitValue(field, value) {
  state.spec.fields[field.key] = Number.isFinite(value) ? value : 0;
  pushSpec();
}

function highlightOnHover(element, bytes) {
  element.addEventListener('mouseenter', () => { state.highlightBytes = bytes; renderBytes(); });
  element.addEventListener('mouseleave', () => { state.highlightBytes = []; renderBytes(); });
}

/** union のうち control_mode と一致しない側を薄くする。 */
function markInactiveRows() {
  const active = activeModeName();
  document.querySelectorAll('.pf-row[data-mode]').forEach((row) => {
    row.classList.toggle('pf-row--inactive', row.dataset.mode !== active);
  });
}

/** サーバが返した実際の値（量子化後）を各コントロールへ書き戻す。 */
function renderValues(decoded) {
  for (const item of decoded || []) {
    const rowInputs = document.querySelectorAll(`[data-key="${CSS.escape(item.key)}"]`);
    rowInputs.forEach((input) => {
      if (input.type === 'checkbox') {
        input.checked = Boolean(item.value);
      } else if (input.dataset.role === 'raw') {
        input.textContent = `0x${item.raw.toString(16).padStart(4, '0')}`;
      } else if (document.activeElement !== input) {
        input.value = typeof item.value === 'number' && !Number.isInteger(item.value)
          ? item.value.toFixed(4)
          : item.value;
      }
    });
  }
  syncModeButtons();
  markInactiveRows();
}

// ===== バイトペイン =====

function buildBytes() {
  const grid = $('bytes');
  grid.innerHTML = '';
  for (let index = 0; index < state.schema.cmd_size; index += 1) {
    const group = state.schema.byte_groups[index];
    const cell = document.createElement('div');
    cell.className = 'pf-byte' + (group ? '' : ' pf-byte--unused');
    cell.dataset.index = String(index);
    if (group) cell.dataset.group = group;
    // 説明文はバックエンドが持つ（未使用の理由まで書き分けてある）
    cell.title = state.schema.byte_descriptions[index];
    cell.innerHTML = `<span class="pf-byte__index">${index}</span><span class="pf-byte__value">00</span>`;
    cell.addEventListener('click', () => editByte(index));
    grid.appendChild(cell);
  }
}

function renderBytes() {
  const grid = $('bytes');
  for (const cell of grid.children) {
    const index = Number(cell.dataset.index);
    cell.querySelector('.pf-byte__value').textContent =
      (state.command[index] ?? 0).toString(16).padStart(2, '0');
    cell.classList.toggle('pf-byte--override', String(index) in state.spec.raw_bytes);
    cell.classList.toggle('pf-byte--highlight', state.highlightBytes.includes(index));
  }
}

function editByte(index) {
  const current = (state.command[index] ?? 0).toString(16).padStart(2, '0');
  const answer = window.prompt(
    `byte ${index} の生値（16進。空欄で上書き解除）\n`
    + 'フィールドより後に適用されるので、この値が常に勝つ。',
    String(index) in state.spec.raw_bytes ? current : '',
  );
  if (answer === null) return;
  if (answer.trim() === '') delete state.spec.raw_bytes[String(index)];
  else {
    const value = parseInt(answer.trim(), 16);
    if (!Number.isFinite(value) || value < 0 || value > 255) { showError('0x00..0xFF で入れる'); return; }
    state.spec.raw_bytes[String(index)] = value;
  }
  pushSpec();
}

// ===== フィールドキャンバス =====
// 色は --crane-field-* だけを読む。クロームはライトだがフィールドは常にダーク。

const canvas = $('field-canvas');
const ctx = canvas.getContext('2d');
const view = { zoom: 1, lengthM: 12.0, widthM: 9.0 };

function fieldTokens() {
  const style = getComputedStyle(document.documentElement);
  const token = (name, fallback) => (style.getPropertyValue(name) || fallback).trim();
  return {
    turf: token('--crane-field-turf', '#12291B'),
    out: token('--crane-field-out', '#0C1C13'),
    grid: token('--crane-field-grid', '#1B3A26'),
    ink: token('--crane-field-ink', '#FFFFFF'),
    inkMuted: token('--crane-field-ink-muted', 'rgba(255,255,255,0.62)'),
    target: token('--crane-field-overlay-move', '#3DD68C'),
    vision: token('--crane-field-overlay-robot', '#7FE3FF'),
  };
}

function resizeCanvas() {
  const dpr = window.devicePixelRatio || 1;
  const rect = canvas.parentElement.getBoundingClientRect();
  canvas.width = Math.round(rect.width * dpr);
  canvas.height = Math.round(rect.height * dpr);
  drawField();
}

function scale() {
  const dpr = window.devicePixelRatio || 1;
  const w = canvas.width / dpr;
  const h = canvas.height / dpr;
  return Math.min(w / (view.lengthM + 1), h / (view.widthM + 1)) * view.zoom;
}

/** クリック位置 → フィールド座標 [m]。+Y は上（キャンバスは下向きなので反転）。 */
function clientToField(clientX, clientY) {
  const dpr = window.devicePixelRatio || 1;
  const rect = canvas.getBoundingClientRect();
  const s = scale();
  return {
    x: (clientX - rect.left - canvas.width / dpr / 2) / s,
    y: -(clientY - rect.top - canvas.height / dpr / 2) / s,
  };
}

function drawField() {
  const dpr = window.devicePixelRatio || 1;
  const tokens = fieldTokens();
  const w = canvas.width / dpr;
  const h = canvas.height / dpr;
  const s = scale();

  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.fillStyle = tokens.out;
  ctx.fillRect(0, 0, w, h);

  ctx.save();
  ctx.translate(w / 2, h / 2);

  ctx.fillStyle = tokens.turf;
  ctx.fillRect(-view.lengthM * s / 2, -view.widthM * s / 2, view.lengthM * s, view.widthM * s);

  ctx.strokeStyle = tokens.grid;
  ctx.lineWidth = 1;
  for (let x = -Math.floor(view.lengthM / 2); x <= view.lengthM / 2; x += 1) {
    ctx.beginPath(); ctx.moveTo(x * s, -view.widthM * s / 2); ctx.lineTo(x * s, view.widthM * s / 2); ctx.stroke();
  }
  for (let y = -Math.floor(view.widthM / 2); y <= view.widthM / 2; y += 1) {
    ctx.beginPath(); ctx.moveTo(-view.lengthM * s / 2, y * s); ctx.lineTo(view.lengthM * s / 2, y * s); ctx.stroke();
  }

  ctx.strokeStyle = tokens.inkMuted;
  ctx.lineWidth = 2;
  ctx.strokeRect(-view.lengthM * s / 2, -view.widthM * s / 2, view.lengthM * s, view.widthM * s);
  ctx.beginPath(); ctx.moveTo(0, -view.widthM * s / 2); ctx.lineTo(0, view.widthM * s / 2); ctx.stroke();
  ctx.beginPath(); ctx.arc(0, 0, 0.5 * s, 0, Math.PI * 2); ctx.stroke();

  // 両方が原点にあるとラベルが重なるので、上下にずらして描く
  drawMarker(ctx, s, tokens.vision, 'vision_global_pos', 'vision_global_theta', tokens, -12);
  drawMarker(ctx, s, tokens.target, 'target_global_pos', 'target_global_theta', tokens, 16);

  ctx.restore();
}

function drawMarker(context, s, color, posPrefix, thetaKey, tokens, labelOffsetY) {
  const x = state.spec.fields[`${posPrefix}.x`] ?? 0;
  const y = state.spec.fields[`${posPrefix}.y`] ?? 0;
  const theta = state.spec.fields[thetaKey] ?? 0;
  const px = x * s;
  const py = -y * s;

  context.strokeStyle = color;
  context.fillStyle = color;
  context.lineWidth = 2;
  context.beginPath();
  context.arc(px, py, 0.09 * s, 0, Math.PI * 2);
  context.stroke();
  context.beginPath();
  context.moveTo(px, py);
  context.lineTo(px + Math.cos(theta) * 0.22 * s, py - Math.sin(theta) * 0.22 * s);
  context.stroke();

  context.fillStyle = color;
  context.font = '11px "IBM Plex Mono", monospace';
  context.fillText(
    `${posPrefix.split('_')[0]} ${x.toFixed(2)}, ${y.toFixed(2)}`,
    px + 10,
    py + labelOffsetY,
  );
}

function setupCanvas() {
  new ResizeObserver(resizeCanvas).observe(canvas.parentElement);
  canvas.addEventListener('mousedown', (event) => {
    state.dragStart = clientToField(event.clientX, event.clientY);
    const prefix = $('click-mode').value;
    state.spec.fields[`${prefix}.x`] = round3(state.dragStart.x);
    state.spec.fields[`${prefix}.y`] = round3(state.dragStart.y);
    pushSpec();
  });
  canvas.addEventListener('mousemove', (event) => {
    if (!state.dragStart) return;
    const now = clientToField(event.clientX, event.clientY);
    const dx = now.x - state.dragStart.x;
    const dy = now.y - state.dragStart.y;
    if (Math.hypot(dx, dy) < 0.05) return;
    state.spec.fields.target_global_theta = round3(Math.atan2(dy, dx));
    pushSpec();
  });
  window.addEventListener('mouseup', () => { state.dragStart = null; });
  canvas.addEventListener('wheel', (event) => {
    event.preventDefault();
    view.zoom = Math.min(6, Math.max(0.4, view.zoom * (event.deltaY < 0 ? 1.1 : 1 / 1.1)));
    drawField();
  }, { passive: false });
  window.matchMedia('(prefers-color-scheme: dark)').addEventListener('change', drawField);
}

const round3 = (value) => Math.round(value * 1000) / 1000;

// ===== 送信バー =====

function renderSenderStats(sender) {
  const running = Boolean(sender && sender.running);
  $('btn-send').style.display = running ? 'none' : '';
  $('btn-stop').style.display = running ? '' : 'none';
  $('counter-readout').textContent = sender && sender.check_counter !== undefined ? sender.check_counter : '-';
  $('sent-readout').textContent = sender ? sender.sent ?? 0 : 0;
  $('rate-readout').textContent = sender ? (sender.rate ?? 0).toFixed(1) : '0.0';
  $('source-readout').textContent =
    running && sender.source ? `送信元 ${sender.source} → ${sender.address}` : '';
}

function setupSendBar() {
  const robot = $('robot-select');
  for (let id = 0; id <= state.schema.max_robot_id; id += 1) {
    const option = document.createElement('option');
    option.value = String(id);
    option.textContent = `#${id}`;
    robot.appendChild(option);
  }
  robot.value = String(state.spec.robot_id);
  robot.addEventListener('change', () => {
    state.spec.robot_id = Number(robot.value);
    pushSpec();
  });

  $('target-select').addEventListener('change', (event) => {
    const value = event.target.value;
    $('target-custom').style.display = value === 'custom' ? '' : 'none';
    state.spec.target = value === 'custom' ? ($('target-custom').value || '127.0.0.1') : value;
    pushSpec();
  });
  $('target-custom').addEventListener('input', (event) => {
    state.spec.target = event.target.value;
    pushSpec();
  });
  $('broadcast-check').addEventListener('change', (event) => {
    state.spec.broadcast = event.target.checked;
    pushSpec();
  });
  $('rate-input').addEventListener('input', (event) => {
    const value = Number(event.target.value);
    if (value > 0) { state.spec.rate_hz = value; pushSpec(); }
  });
  $('base-select').addEventListener('change', (event) => {
    state.spec.base = event.target.value;
    pushSpec();
  });

  $('btn-send').addEventListener('click', async () => {
    const duration = Number($('duration-input').value);
    try {
      applySnapshot(await api('/api/send/start', {
        duration_s: duration > 0 ? duration : null,
        freeze_counter: $('freeze-check').checked,
        stop_on_exit: $('stop-on-exit-check').checked,
      }));
    } catch (err) { showError(err.message); }
  });
  $('btn-stop').addEventListener('click', async () => {
    try { applySnapshot(await api('/api/send/stop', {})); } catch (err) { showError(err.message); }
  });
  $('btn-clear-overrides').addEventListener('click', () => {
    state.spec.raw_bytes = {};
    pushSpec();
  });
}

// ===== 区間スケジュール =====
// CLI の --schedule / spec の steps と同じもの。field=value をカンマ区切りで書く。

function stepFieldsToText(fields) {
  return Object.entries(fields)
    .map(([key, value]) => `${key}=${typeof value === 'boolean' ? (value ? 1 : 0) : value}`)
    .join(', ');
}

/** "key=value, key=value" を spec の fields へ。未知のキーはここで弾く。 */
function parseStepFields(text) {
  const out = {};
  for (const chunk of text.split(',')) {
    const trimmed = chunk.trim();
    if (!trimmed) continue;
    const index = trimmed.indexOf('=');
    if (index < 0) throw new Error(`key=value の形式で書く: ${trimmed}`);
    const key = trimmed.slice(0, index).trim();
    const raw = trimmed.slice(index + 1).trim();
    const field = state.schema.fields.find((f) => f.key === key);
    if (!field) throw new Error(`未知のフィールド: ${key}`);
    out[key] = field.kind === 'flag' ? !['0', 'false', 'off', ''].includes(raw.toLowerCase())
      : Number(raw);
    if (field.kind !== 'flag' && !Number.isFinite(out[key])) {
      throw new Error(`${key} の値が数値でない: ${raw}`);
    }
  }
  return out;
}

function renderSteps() {
  const rows = $('steps-rows');
  const activeIndex = state.snapshot?.sender?.running ? state.snapshot.sender.step_index : null;
  rows.innerHTML = '';

  state.spec.steps.forEach((step, index) => {
    const row = document.createElement('div');
    row.className = 'pf-step' + (index === activeIndex ? ' pf-step--active' : '');

    const label = document.createElement('span');
    label.className = 'pf-step__index';
    label.textContent = String(index);

    const duration = document.createElement('input');
    duration.type = 'number';
    duration.className = 'm3-text-input';
    duration.style.width = '72px';
    duration.min = '0.1';
    duration.step = '0.5';
    duration.value = String(step.duration_s);
    duration.title = '区間の長さ（秒）';
    duration.addEventListener('input', () => {
      const value = Number(duration.value);
      if (value > 0) { step.duration_s = value; pushSpec(); }
    });

    const seconds = document.createElement('span');
    seconds.className = 'pf-bar__label';
    seconds.textContent = 's';

    const fields = document.createElement('input');
    fields.type = 'text';
    fields.className = 'm3-text-input pf-step__fields';
    fields.value = stepFieldsToText(step.fields);
    fields.placeholder = 'polar.target_global_velocity_r=0.2, flags.is_vision_available=1';
    const error = document.createElement('span');
    error.className = 'pf-step__error';
    fields.addEventListener('change', () => {
      try {
        step.fields = parseStepFields(fields.value);
        error.textContent = '';
        pushSpec();
      } catch (err) {
        error.textContent = err.message;
      }
    });

    const remove = document.createElement('button');
    remove.className = 'm3-icon-btn m3-icon-btn--sm';
    remove.title = 'この区間を削除';
    remove.innerHTML = '<span class="material-symbols-outlined icon-sm">delete</span>';
    remove.addEventListener('click', () => {
      state.spec.steps.splice(index, 1);
      renderSteps();
      pushSpec();
    });

    row.append(label, duration, seconds, fields, error, remove);
    rows.appendChild(row);
  });

  $('steps-count').textContent = state.spec.steps.length ? ` (${state.spec.steps.length})` : '';
}

function setupSteps() {
  // spec に区間があるなら最初から開く（読み込んだ spec の中身が隠れると気づけない）
  $('steps-panel').hidden = state.spec.steps.length === 0;
  renderSteps();
  $('btn-steps').addEventListener('click', () => {
    const panel = $('steps-panel');
    panel.hidden = !panel.hidden;
    if (!panel.hidden) renderSteps();
  });
  $('btn-step-add').addEventListener('click', () => {
    state.spec.steps.push({ duration_s: 2, fields: {} });
    $('steps-panel').hidden = false;
    renderSteps();
    pushSpec();
  });
}

// ===== フィードバック帯 =====

function severityForVoltage(v) {
  if (!v) return 'muted';
  if (v < 21.0) return 'crit';
  if (v < 22.5) return 'warn';
  return 'ok';
}

function severityForTemperature(t) {
  if (t >= 75) return 'crit';
  if (t >= 60) return 'warn';
  return 'ok';
}

function renderFeedback(feedback) {
  const stats = $('feedback-stats');
  const endpoint = $('feedback-endpoint');
  const button = $('btn-watch');
  if (!feedback) {
    endpoint.textContent = '';
    stats.innerHTML = '<span class="pf-stat__label">未購読</span>';
    button.textContent = 'フィードバック購読';
    return;
  }
  button.textContent = '購読を止める';
  endpoint.textContent = `${feedback.group}:${feedback.port}`;

  const last = feedback.last;
  const cells = [
    statCell('rate', `${feedback.rate.toFixed(0)} pkt/s`, feedback.silent ? 'crit' : 'ok'),
    statCell('sync不正', String(feedback.bad_sync), feedback.bad_sync > 0 ? 'warn' : 'ok'),
  ];
  if (last) {
    // 送った check_counter が返ってきているかを並べる。切り分けの起点になる。
    const sent = state.snapshot?.sender?.check_counter;
    const echo = sent === undefined ? String(last.counter) : `${last.counter} / 送信 ${sent}`;
    cells.push(statCell('counter', echo, 'ok'));
    cells.push(statCell('電圧', last.voltage.map((v) => v.toFixed(1)).join(' / '),
      severityForVoltage(last.voltage[0])));
    const maxTemp = Math.max(...last.temperature);
    cells.push(statCell('温度max', `${maxTemp}°C`, severityForTemperature(maxTemp)));
    cells.push(statCell('ball', last.ball_detection.join(','), 'ok'));
    cells.push(statCell('kick', String(last.kick_state), 'ok'));
    cells.push(statCell('odom', last.odom.map((v) => v.toFixed(2)).join(', '), 'ok'));
    if (last.error_id) cells.push(statCell('error', `${last.error_id}/${last.error_info}`, 'crit'));
    if (last.boot_like) cells.push(statCell('状態', 'BOOT_LIKE (リセット直後)', 'warn'));
  } else if (feedback.received === 0) {
    cells.push(statCell('受信', 'まだ 1 つも来ていない（NIC 違い / CM4 停止）', 'warn'));
  }
  stats.innerHTML = '';
  cells.forEach((cell) => stats.appendChild(cell));
}

function statCell(label, value, severity) {
  const wrap = document.createElement('span');
  wrap.className = 'pf-stat';
  const labelEl = document.createElement('span');
  labelEl.className = 'pf-stat__label';
  labelEl.textContent = label;
  const valueEl = document.createElement('span');
  valueEl.className = 'pf-stat__value';
  valueEl.dataset.severity = severity;
  valueEl.textContent = value;
  wrap.append(labelEl, valueEl);
  return wrap;
}

function setupFeedback() {
  $('btn-watch').addEventListener('click', async () => {
    try {
      const path = state.snapshot?.feedback ? '/api/watch/stop' : '/api/watch/start';
      applySnapshot(await api(path, { robot_id: state.spec.robot_id }));
    } catch (err) { showError(err.message); }
  });
}

// ===== spec ダイアログ =====

function cliCommand() {
  const parts = ['crane-forge send'];
  parts.push(`--robot ${state.spec.robot_id}`);
  if (state.spec.target !== 'real') parts.push(`--target ${state.spec.target}`);
  if (state.spec.broadcast) parts.push('--broadcast');
  if (state.spec.rate_hz !== 62.5) parts.push(`--rate-hz ${state.spec.rate_hz}`);
  if (state.spec.base !== 'neutral') parts.push(`--base ${state.spec.base}`);
  for (const [key, value] of Object.entries(state.spec.fields)) {
    parts.push(`--set ${key}=${typeof value === 'boolean' ? (value ? 1 : 0) : value}`);
  }
  for (const [index, value] of Object.entries(state.spec.raw_bytes)) {
    parts.push(`--raw-byte ${index}=0x${value.toString(16).padStart(2, '0')}`);
  }
  return parts.join(' \\\n    ');
}

function setupSpecDialog() {
  const backdrop = $('spec-backdrop');
  $('btn-spec').addEventListener('click', () => {
    $('spec-text').value = JSON.stringify(state.spec, null, 2);
    $('spec-cli').value = cliCommand();
    backdrop.dataset.open = 'true';
  });
  $('btn-spec-close').addEventListener('click', () => { backdrop.dataset.open = 'false'; });
  $('btn-spec-apply').addEventListener('click', async () => {
    try {
      const parsed = JSON.parse($('spec-text').value);
      applySnapshot(await api('/api/spec', parsed));
      state.spec = normalizeSpec(state.snapshot.spec);
      syncBarFromSpec();
      backdrop.dataset.open = 'false';
    } catch (err) { showError(err.message); }
  });
  document.addEventListener('keydown', (event) => {
    if (event.key === 'Escape' && backdrop.dataset.open === 'true') backdrop.dataset.open = 'false';
  });
}

function syncBarFromSpec() {
  $('robot-select').value = String(state.spec.robot_id);
  $('rate-input').value = String(state.spec.rate_hz);
  $('base-select').value = state.spec.base;
  $('broadcast-check').checked = Boolean(state.spec.broadcast);
  const known = ['real', 'sim'].includes(state.spec.target);
  $('target-select').value = known ? state.spec.target : 'custom';
  $('target-custom').style.display = known ? 'none' : '';
  if (!known) $('target-custom').value = state.spec.target;
}

// ===== WebSocket =====

function connect() {
  const ws = new WebSocket(`ws://${location.host}/ws`);
  ws.onmessage = (event) => {
    const snapshot = JSON.parse(event.data);
    // ページ側で編集中の spec は上書きしない。サーバは組み立て結果と実測値だけを返す。
    state.snapshot = snapshot;
    state.command = hexToBytes(snapshot.command_hex);
    renderWarnings(snapshot);
    renderBytes();
    renderValues(snapshot.fields);
    renderSenderStats(snapshot.sender);
    renderFeedback(snapshot.feedback);
  };
  ws.onclose = () => setTimeout(connect, 1500);
}

// ===== 起動 =====

/** 欠けているキーを埋める。手書きの spec JSON を読み込む経路もあるので必ず通す。 */
function normalizeSpec(spec) {
  return {
    target: 'real',
    robot_id: 0,
    rate_hz: 62.5,
    base: 'neutral',
    broadcast: false,
    ...spec,
    fields: spec.fields || {},
    raw_bytes: spec.raw_bytes || {},
    steps: spec.steps || [],
  };
}

async function init() {
  state.schema = await api('/api/schema');
  // 既定値はサーバが持つ neutral に任せる。ページ側は空の fields から始める。
  const initial = await api('/api/state');
  state.spec = normalizeSpec(initial.spec);

  buildEditor();
  buildBytes();
  setupCanvas();
  setupSendBar();
  setupSteps();
  setupFeedback();
  setupSpecDialog();
  syncBarFromSpec();
  applySnapshot(initial);
  resizeCanvas();
  connect();
}

init().catch((err) => showError(`初期化に失敗: ${err.message}`));
