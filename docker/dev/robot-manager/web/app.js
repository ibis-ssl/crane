'use strict';

const tableBody = document.getElementById('table-body');
const connDot = document.getElementById('conn-dot');
const connText = document.getElementById('conn-text');

const runningCount = document.getElementById('running-count');
const stoppedCount = document.getElementById('stopped-count');
const offlineCount = document.getElementById('offline-count');

const refreshBtn = document.getElementById('refresh-btn');
const startAllBtn = document.getElementById('start-all-btn');
const stopAllBtn = document.getElementById('stop-all-btn');
const NUM_ROBOTS = 13;

let robots = [];
let busy = false;

function offlineRobots() {
  return Array.from({ length: NUM_ROBOTS }, (_, id) => ({
    robot_id: id,
    ip: `192.168.20.${100 + id}`,
    success: false,
    status: 'Offline',
  }));
}

function classifyStatus(status) {
  const s = (status || '').toLowerCase();
  if (s.includes('run')) return 'running';
  if (s.includes('stop')) return 'stopped';
  if (s.includes('off')) return 'offline';
  if (s.includes('error')) return 'error';
  return 'offline';
}

function setConnected(ok) {
  connDot.className = `dot ${ok ? 'ok' : 'ng'}`;
  connText.textContent = ok ? 'connected' : 'disconnected';
}

function setBusy(v) {
  busy = v;
  refreshBtn.disabled = v;
  startAllBtn.disabled = v;
  stopAllBtn.disabled = v;
}

function renderSummary() {
  let run = 0;
  let stop = 0;
  let off = 0;
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

function rowHtml(robot) {
  const cls = classifyStatus(robot.status);
  return `
    <tr>
      <td>#${robot.robot_id}</td>
      <td><span class="status ${cls}">${robot.status || 'Offline'}</span></td>
      <td>${robot.ip || '--'}</td>
      <td>
        <button onclick="controlRobot(${robot.robot_id}, 'start')">Start</button>
        <button class="danger" onclick="controlRobot(${robot.robot_id}, 'stop')">Stop</button>
      </td>
    </tr>`;
}

function renderTable() {
  tableBody.innerHTML = robots.map(rowHtml).join('');
  renderSummary();
}

async function fetchRobots() {
  try {
    const res = await fetch('/api/robots', { cache: 'no-store' });
    if (!res.ok) throw new Error(`HTTP ${res.status}`);
    const data = await res.json();
    robots = data.robots || [];
    renderTable();
    setConnected(true);
  } catch (e) {
    setConnected(false);
    // 通信断時は前回値を残さず、即座に Offline 表示へフォールバックする
    robots = offlineRobots();
    renderTable();
  }
}

async function controlRobot(robotId, command) {
  if (busy) return;
  setBusy(true);
  try {
    const res = await fetch(`/api/robots/${robotId}/${command}`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: '{}',
    });
    if (!res.ok) throw new Error(`HTTP ${res.status}`);
    await fetchRobots();
  } catch (e) {
    setConnected(false);
  } finally {
    setBusy(false);
  }
}

window.controlRobot = controlRobot;

async function startAll() {
  if (busy) return;
  setBusy(true);
  try {
    await Promise.all(robots.map((r) => fetch(`/api/robots/${r.robot_id}/start`, { method: 'POST' })));
    await fetchRobots();
  } catch (e) {
    setConnected(false);
    robots = offlineRobots();
    renderTable();
  } finally {
    setBusy(false);
  }
}

async function stopAll() {
  if (busy) return;
  if (!window.confirm('全ロボットを停止しますか？')) return;
  setBusy(true);
  try {
    await Promise.all(robots.map((r) => fetch(`/api/robots/${r.robot_id}/stop`, { method: 'POST' })));
    await fetchRobots();
  } catch (e) {
    setConnected(false);
    robots = offlineRobots();
    renderTable();
  } finally {
    setBusy(false);
  }
}

refreshBtn.addEventListener('click', fetchRobots);
startAllBtn.addEventListener('click', startAll);
stopAllBtn.addEventListener('click', stopAll);

robots = offlineRobots();
renderTable();
fetchRobots();
setInterval(fetchRobots, 4000);
