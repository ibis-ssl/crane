import { CONTROL_MODE_SHORT, CONTROL_MODE_LONG, ROBOT_HIT_RADIUS_M, BALL_HIT_RADIUS_M } from './renderer/constants.js';
import { SvgPrimitiveParser } from './renderer/SvgPrimitiveParser.js';
import { CanvasRenderer } from './renderer/CanvasRenderer.js';
import { FieldLayer } from './renderer/FieldLayer.js';
import { ThemeTokens } from './renderer/ThemeTokens.js';
import { GameControlClient } from './ws/GameControlClient.js';
import { DockLayout } from './ui/DockLayout.js';
import { LogPanel } from './ui/LogPanel.js';

class CraneViewer {
    constructor() {
        this.websocket = null;
        this.layerStore = new Map();
        this.visibleLayers = new Set();
        this.seenLayers = new Set();
        this.zoomLevel = 1.0;
        this.panOffset = { x: 0, y: 0 };
        this.isPanning = false;
        this.lastPanPoint = { x: 0, y: 0 };
        this.pendingUpdateBatch = [];
        this.flushTimer = null;
        this.flushIntervalMs = 50;

        this.robotsOurs = {};
        this.robotsTheirs = {};
        this.controlTargets = {};

        this.moveMode = false;
        this.selectedRobotId = null;
        this.isYellow = false;
        this.onPositiveHalf = false;
        this._prevFieldLength = null;

        this.ballPos = { x: 0, y: 0 };
        this.simEditMode = false;
        this.simSelectedObj = null;
        this.placeBallPending = null;
        this.gcClient = new GameControlClient();
        this._dragStartX = 0;
        this._dragStartY = 0;
        this._dragMoved = false;
        this._lastMouseField = { x: 0, y: 0 };
        this._haltPending = false;
        this._haltTimer = null;

        this.robotUpdateTimer = null;
        this.parser = new SvgPrimitiveParser();
        this.fieldLayer = new FieldLayer();
        this.themeTokens = new ThemeTokens();
        this.renderer = null;
        this.dockLayout = null;
        this.logPanel = null;

        this.init();
    }

    init() {
        const canvas = document.getElementById('field-canvas');
        if (canvas) {
            this.renderer = new CanvasRenderer(canvas, this, this.fieldLayer, this.themeTokens);
        }
        this.themeTokens.mount(() => this.renderer?.invalidate());
        this.dockLayout = new DockLayout();
        const logBody = document.getElementById('log-panel-body');
        if (logBody) this.logPanel = new LogPanel(logBody);
        this.setupWebSocket();
        this.setupEventListeners();
        this.setupDelegatedListeners();
        this.setupLogPanelControls();
        this.updateConnectionStatus(false);
        this.gcClient.connect(window.location.hostname);
        this.gcClient.onStateChange = (state) => this.updateGcPanel(state);
    }

    setupWebSocket() {
        const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
        const wsUrl = `${protocol}//${window.location.hostname}:8091`;
        this.websocket = new WebSocket(wsUrl);
        this.websocket.onopen = () => {
            this.updateConnectionStatus(true);
            this.logPanel?.appendLog('info', 'WS', `接続 ${wsUrl}`);
        };
        this.websocket.onmessage = (event) => {
            try { this.handleMessage(JSON.parse(event.data)); } catch (e) {
                console.error('メッセージ解析エラー:', e);
                this.logPanel?.appendLog('error', 'WS', `メッセージ解析エラー: ${e.message}`);
            }
        };
        this.websocket.onclose = () => {
            this.updateConnectionStatus(false);
            this.logPanel?.appendLog('warn', 'WS', '切断 — 3秒後に再接続');
            setTimeout(() => {
                if (!this.websocket || this.websocket.readyState === WebSocket.CLOSED) this.setupWebSocket();
            }, 3000);
        };
        this.websocket.onerror = () => {
            this.updateConnectionStatus(false);
            this.logPanel?.appendLog('error', 'WS', '接続エラー');
        };
    }

    handleMessage(data) {
        switch (data.type) {
            case 'svg_data':        this.handleSvgData(data); break;
            case 'svg_update':      this.handleSvgUpdate(data); break;
            case 'world_model':     this.handleWorldModel(data); break;
            case 'control_targets': this.handleControlTargets(data); break;
            case 'robot_commands':  this.handleRobotCommands(data); break;
            case 'game_info':       this.handleGameInfo(data); break;
        }
    }

    _registerLayer(name) {
        if (!this.seenLayers.has(name)) {
            this.seenLayers.add(name);
            this.visibleLayers.add(name);
        }
    }

    handleSvgData(data) {
        this.layerStore.clear();
        if (data.layers) {
            for (const layer of data.layers) {
                this._registerLayer(layer.layer);
                this.layerStore.set(layer.layer, {
                    primitives: [...layer.svg_primitives],
                    commands: [],
                    dirty: true,
                });
            }
        }
        this.renderer?.invalidate();
        this.updateLayerList();
        this.updateStats();
    }

    handleSvgUpdate(data) {
        if (Array.isArray(data.updates) && data.updates.length > 0) {
            this.pendingUpdateBatch.push(...data.updates);
        }
        this.scheduleFlushUpdates();
    }

    scheduleFlushUpdates() {
        if (this.flushTimer) return;
        this.flushTimer = setTimeout(() => {
            const batch = this.pendingUpdateBatch.splice(0);
            this.flushTimer = null;
            if (batch.length === 0) return;
            this.applySvgUpdates(this.coalesceLayerUpdates(batch));
            this.renderer?.invalidate();
            this.updateLayerList();
            this.updateStats();
        }, this.flushIntervalMs);
    }

    coalesceLayerUpdates(updates) {
        const byLayer = new Map();
        for (const upd of updates) {
            const layer = upd.layer;
            const op = (upd.operation || '').toLowerCase();
            const prim = Array.isArray(upd.svg_primitives) ? upd.svg_primitives : [];
            if (!layer || !op) continue;
            if (!byLayer.has(layer)) byLayer.set(layer, { operation: null, svg_primitives: [] });
            const entry = byLayer.get(layer);
            if (op === 'replace') {
                entry.operation = 'replace';
                entry.svg_primitives = [...prim];
            } else if (op === 'clear') {
                entry.operation = 'clear';
                entry.svg_primitives = [];
            } else if (op === 'append') {
                if (entry.operation === 'clear') {
                    entry.operation = 'replace';
                    entry.svg_primitives = [...prim];
                } else {
                    if (!entry.operation) entry.operation = 'append';
                    entry.svg_primitives.push(...prim);
                }
            }
        }
        return Array.from(byLayer.entries()).map(([layer, v]) => ({ layer, ...v }));
    }

    applySvgUpdates(updates) {
        if (!Array.isArray(updates) || updates.length === 0) return;
        for (const upd of updates) {
            const op = (upd.operation || '').toLowerCase();
            const prims = Array.isArray(upd.svg_primitives) ? upd.svg_primitives : [];
            const existing = this.layerStore.get(upd.layer);
            if (op === 'replace') {
                this._registerLayer(upd.layer);
                this.layerStore.set(upd.layer, { primitives: [...prims], commands: [], dirty: true });
            } else if (op === 'append') {
                if (existing) {
                    existing.primitives.push(...prims);
                    existing.dirty = true;
                } else {
                    this._registerLayer(upd.layer);
                    this.layerStore.set(upd.layer, { primitives: [...prims], commands: [], dirty: true });
                }
            } else if (op === 'clear') {
                if (existing) {
                    existing.primitives = []; existing.commands = []; existing.dirty = false;
                } else {
                    this.layerStore.set(upd.layer, { primitives: [], commands: [], dirty: false });
                }
            }
        }
    }

    handleWorldModel(data) {
        if (data.is_yellow !== undefined) this.isYellow = data.is_yellow;
        if (data.on_positive_half !== undefined) this.onPositiveHalf = data.on_positive_half;
        if (data.ball) this.ballPos = { x: data.ball.x, y: data.ball.y };
        if (data.robots_ours) {
            this.robotsOurs = {};
            for (const robot of data.robots_ours) this.robotsOurs[robot.id] = robot;
        }
        if (data.robots_theirs) {
            this.robotsTheirs = {};
            for (const robot of data.robots_theirs) this.robotsTheirs[robot.id] = robot;
        }

        // field_info 動的 viewBox: フィールドサイズが変わった時だけ zoom リセット
        if (data.field_info) {
            const changed = this.fieldLayer.updateFromFieldInfo(data.field_info);
            if (changed) {
                const newLen = data.field_info.length;
                if (this._prevFieldLength !== null && newLen !== this._prevFieldLength) {
                    this.zoomLevel = 1.0;
                    this.panOffset = { x: 0, y: 0 };
                }
                this._prevFieldLength = newLen;
                // viewBox 変化でパーサーのキャッシュを無効化
                const fl = this.fieldLayer;
                this.parser.setViewBox(fl.vbX, fl.vbY, fl.vbW, fl.vbH);
                // 全レイヤーを再パース対象にする
                for (const layer of this.layerStore.values()) layer.dirty = true;
                this.renderer?.invalidate();
            }
        }

        this.scheduleRobotUpdate();
        if (this.moveMode && this.selectedRobotId !== null) this.renderer?.invalidate();
        if (this.simEditMode) this.renderer?.invalidate();
    }

    handleControlTargets(data) {
        if (data.commands) {
            this.controlTargets = {};
            for (const cmd of data.commands) this.controlTargets[cmd.robot_id] = cmd;
        }
        this.scheduleRobotUpdate();
    }

    handleRobotCommands(data) {
        if (data.commands && Object.keys(this.controlTargets).length === 0) {
            for (const cmd of data.commands) this.controlTargets[cmd.robot_id] = cmd;
            this.scheduleRobotUpdate();
        }
    }

    handleGameInfo(data) {
        const set = (id, val) => { const el = document.getElementById(id); if (el) el.textContent = val; };
        set('score-our', data.our_score ?? 0);
        set('score-their', data.their_score ?? 0);
        set('play-situation', data.play_situation || '--');
        set('game-stage', data.game_stage || '--');
    }

    scheduleRobotUpdate() {
        if (this.robotUpdateTimer) return;
        this.robotUpdateTimer = setTimeout(() => {
            this.robotUpdateTimer = null;
            this.renderRobotCards();
        }, 500);
    }

    updateConnectionStatus(connected) {
        const dot = document.getElementById('connection-dot');
        const label = document.getElementById('connection-label');
        const banner = document.getElementById('offline-banner');
        if (dot) dot.classList.toggle('connected', connected);
        if (label) label.textContent = connected ? 'connected' : 'disconnected';
        if (banner) banner.classList.toggle('visible', !connected);
    }

    renderRobotCards() {
        const container = document.getElementById('robot-cards-grid');
        if (!container) return;
        const ids = Object.keys(this.robotsOurs).map(Number).sort((a, b) => a - b);
        if (ids.length === 0) {
            container.innerHTML = '<div style="font-size:0.7rem;color:var(--md-sys-color-on-surface-variant);text-align:center;grid-column:span 4;padding:8px;">No robot data</div>';
            return;
        }
        container.innerHTML = '';
        for (const id of ids) {
            const cmd = this.controlTargets[id];
            const modeName = cmd ? (CONTROL_MODE_SHORT[cmd.control_mode] ?? `M${cmd.control_mode}`) : '--';
            const plannerName = cmd?.planner_name || '--';
            const card = document.createElement('div');
            card.className = 'robot-card';
            card.innerHTML = `<div class="robot-id">${id}</div><div class="robot-mode">${modeName}</div><div class="robot-planner" title="${plannerName}">${plannerName}</div>`;
            card.addEventListener('click', () => this.showRobotDetail(id));
            container.appendChild(card);
        }
    }

    showRobotDetail(id) {
        const robot = this.robotsOurs[id];
        const cmd = this.controlTargets[id];
        if (!robot) return;
        const modal = document.getElementById('robot-detail-modal');
        const scrim = document.getElementById('robot-detail-scrim');
        if (!modal) return;
        const modeName = cmd ? (CONTROL_MODE_LONG[cmd.control_mode] ?? `MODE_${cmd.control_mode}`) : '--';
        document.getElementById('robot-detail-title').textContent = `Robot ${id}`;
        let targetHtml = '';
        if (cmd?.position_target_mode) {
            targetHtml = `<tr><td>Target Pos</td><td>(${cmd.position_target_mode.target_x?.toFixed(3)}, ${cmd.position_target_mode.target_y?.toFixed(3)})</td></tr>`;
        } else if (cmd?.simple_velocity_target_mode) {
            targetHtml = `<tr><td>Target Vel</td><td>(${cmd.simple_velocity_target_mode.target_vx?.toFixed(3)}, ${cmd.simple_velocity_target_mode.target_vy?.toFixed(3)})</td></tr>`;
        }
        document.getElementById('robot-detail-body').innerHTML = `
            <table class="m3-data-table m3-data-table--borderless m3-mb-sm"><tbody>
                <tr><td style="width:6em">Control Mode</td><td>${modeName}</td></tr>
                <tr><td>Planner</td><td>${cmd?.planner_name || '--'}</td></tr>
                <tr><td>Est. X</td><td>${robot.x?.toFixed(3)} m</td></tr>
                <tr><td>Est. Y</td><td>${robot.y?.toFixed(3)} m</td></tr>
                <tr><td>Est. θ</td><td>${robot.theta?.toFixed(3)} rad</td></tr>
                <tr><td>Vel Vx</td><td>${robot.vx?.toFixed(3)} m/s</td></tr>
                <tr><td>Vel Vy</td><td>${robot.vy?.toFixed(3)} m/s</td></tr>
                <tr><td>ω</td><td>${robot.omega?.toFixed(3)} rad/s</td></tr>
                <tr><td>Vision X</td><td>${robot.vision_x?.toFixed(3) ?? '--'}</td></tr>
                <tr><td>Vision Y</td><td>${robot.vision_y?.toFixed(3) ?? '--'}</td></tr>
                ${targetHtml}
                <tr><td>Target θ</td><td>${cmd?.target_theta?.toFixed(3) ?? '--'}</td></tr>
            </tbody></table>
            <a href="/robot_telemetry.html?id=${id}" class="m3-btn m3-btn--filled m3-btn--sm m3-w-full">
                <span class="material-symbols-outlined icon-sm">show_chart</span> Open Telemetry
            </a>
        `;
        modal.classList.add('open');
        if (scrim) scrim.classList.add('open');
        window.__closeRobotDialog = () => {
            modal.classList.remove('open');
            if (scrim) scrim.classList.remove('open');
        };
        const onKey = (e) => {
            if (e.key === 'Escape') {
                window.__closeRobotDialog();
                document.removeEventListener('keydown', onKey);
            }
        };
        document.addEventListener('keydown', onKey);
    }

    updateLayerList() {
        const container = document.getElementById('layer-list');
        if (!container) return;
        if (this.layerStore.size === 0) {
            container.innerHTML = '<div style="font-size:0.7rem;color:var(--md-sys-color-on-surface-variant);text-align:center;padding:6px;">No layers</div>';
            return;
        }
        container.innerHTML = '';
        for (const [name, layer] of this.layerStore) {
            const item = document.createElement('div');
            item.className = 'layer-item';
            item.innerHTML = `
                <input type="checkbox" class="m3-checkbox layer-cb" id="layer-${name}"
                       data-layer="${name}" ${this.visibleLayers.has(name) ? 'checked' : ''}>
                <label for="layer-${name}">${name}</label>
                <span class="m3-ms-auto m3-text-on-surface-variant" style="font-size:0.65rem">${layer.primitives.length}</span>
            `;
            container.appendChild(item);
        }
        container.querySelectorAll('.layer-cb').forEach(cb => {
            cb.addEventListener('change', (e) => {
                const name = e.target.dataset.layer;
                if (e.target.checked) this.visibleLayers.add(name);
                else this.visibleLayers.delete(name);
                this.renderer?.invalidate();
            });
        });
    }

    updateStats() {
        const layerEl = document.getElementById('layer-count');
        const primEl = document.getElementById('prim-count');
        if (layerEl) layerEl.textContent = this.layerStore.size;
        if (primEl) {
            let total = 0;
            for (const layer of this.layerStore.values()) total += layer.primitives.length;
            primEl.textContent = total;
        }
    }

    clientToFieldCoords(clientX, clientY) {
        return this.renderer ? this.renderer.clientToFieldCoords(clientX, clientY) : { x: 0, y: 0 };
    }

    findRobotAtPosition(fieldX, fieldY) {
        let closest = null;
        let minDist = ROBOT_HIT_RADIUS_M;
        for (const [id, robot] of Object.entries(this.robotsOurs)) {
            const dx = robot.x - fieldX;
            const dy = robot.y - fieldY;
            const dist = Math.sqrt(dx * dx + dy * dy);
            if (dist < minDist) { minDist = dist; closest = Number(id); }
        }
        return closest;
    }

    _nearestRobot(robotMap, fieldX, fieldY) {
        let id = null, dist = ROBOT_HIT_RADIUS_M;
        for (const [k, r] of Object.entries(robotMap)) {
            const d = Math.hypot(r.x - fieldX, r.y - fieldY);
            if (d < dist) { dist = d; id = Number(k); }
        }
        return id !== null ? { id, dist } : null;
    }

    simSelectAt(fieldX, fieldY) {
        const candidates = [];
        const ourHit = this._nearestRobot(this.robotsOurs, fieldX, fieldY);
        const theirHit = this._nearestRobot(this.robotsTheirs, fieldX, fieldY);
        const ballDist = Math.hypot(this.ballPos.x - fieldX, this.ballPos.y - fieldY);

        if (ourHit) candidates.push({ kind: 'our', dist: ourHit.dist, id: ourHit.id });
        if (ballDist < BALL_HIT_RADIUS_M) candidates.push({ kind: 'ball', dist: ballDist });
        if (theirHit) candidates.push({ kind: 'their', dist: theirHit.dist, id: theirHit.id });

        candidates.sort((a, b) => {
            if (Math.abs(a.dist - b.dist) > 1e-6) return a.dist - b.dist;
            const rank = { ball: 0, our: 1, their: 2 };
            return rank[a.kind] - rank[b.kind];
        });

        const best = candidates[0] ?? null;
        if (!best) {
            this.simSelectedObj = null;
        } else if (best.kind === 'ball') {
            this.simSelectedObj = { type: 'ball' };
        } else if (best.kind === 'our') {
            const r = this.robotsOurs[best.id];
            this.simSelectedObj = { type: 'robot', id: best.id, yellow: this.isYellow, theta: r?.theta ?? 0 };
        } else {
            const r = this.robotsTheirs[best.id];
            this.simSelectedObj = { type: 'robot', id: best.id, yellow: !this.isYellow, theta: r?.theta ?? 0 };
        }
        this._updateSimLabel();
        this.renderer?.invalidate();
    }

    simTeleportTo(fieldX, fieldY) {
        if (!this.simSelectedObj) return;
        const obj = this.simSelectedObj;
        if (obj.type === 'ball') {
            this.websocket?.send(JSON.stringify({ type: 'sim_teleport_ball', x: fieldX, y: fieldY, vx: 0, vy: 0 }));
        } else {
            this.websocket?.send(JSON.stringify({
                type: 'sim_teleport_robot',
                id: obj.id,
                team: obj.yellow ? 'yellow' : 'blue',
                x: fieldX, y: fieldY,
                orientation_deg: (obj.theta ?? 0) * 180 / Math.PI
            }));
        }
    }

    resetBallToCenter() {
        this.websocket?.send(JSON.stringify({ type: 'sim_teleport_ball', x: 0, y: 0, vx: 0, vy: 0 }));
    }

    applySimEndpoint() {
        const host = document.getElementById('sim-host-input')?.value || '127.0.0.1';
        const port = parseInt(document.getElementById('sim-port-input')?.value || '10300', 10);
        this.websocket?.send(JSON.stringify({ type: 'sim_set_endpoint', host, port }));
    }

    toggleSimEditMode() {
        this.simEditMode = !this.simEditMode;
        const btn = document.getElementById('btn-sim-edit');
        const canvas = document.getElementById('field-canvas');
        if (this.simEditMode) {
            if (this.moveMode) this.deactivateMoveMode();
            this.cancelBallPlacement();
            this.simSelectedObj = null;
            btn?.classList.add('active');
            if (canvas) canvas.style.cursor = 'crosshair';
        } else {
            this.simSelectedObj = null;
            btn?.classList.remove('active');
            if (canvas) canvas.style.cursor = 'grab';
        }
        this._updateSimLabel();
        this.renderer?.invalidate();
    }

    activateBallPlacement(team) {
        if (this.simEditMode) this.toggleSimEditMode();
        this.cancelBallPlacement();
        this.placeBallPending = team;
        const canvas = document.getElementById('field-canvas');
        if (canvas) canvas.style.cursor = 'crosshair';
        document.getElementById(`btn-place-ball-${team.toLowerCase()}`)?.classList.add('active');
    }

    cancelBallPlacement() {
        if (!this.placeBallPending) return;
        const team = this.placeBallPending;
        this.placeBallPending = null;
        document.getElementById('field-canvas').style.cursor =
            this.simEditMode ? 'crosshair' : 'grab';
        document.getElementById(`btn-place-ball-${team.toLowerCase()}`)?.classList.remove('active');
    }

    _updateSimLabel() {
        const el = document.getElementById('sim-selected-label');
        if (!el) return;
        if (!this.simEditMode) { el.textContent = 'mode off'; return; }
        if (!this.simSelectedObj) { el.textContent = 'click to select'; return; }
        const obj = this.simSelectedObj;
        el.textContent = obj.type === 'ball' ? 'Ball' : `${obj.yellow ? 'Yellow' : 'Blue'} #${obj.id}`;
    }

    updateGcPanel(state) {
        if (!state) return;
        const ts = state.teamState ?? {};
        const y = ts['YELLOW'] ?? {};
        const b = ts['BLUE'] ?? {};
        const scoreEl = document.getElementById('gc-score');
        if (scoreEl) scoreEl.textContent = `${y.goals ?? 0} : ${b.goals ?? 0}`;
        const stageEl = document.getElementById('gc-stage');
        if (stageEl) stageEl.textContent = (state.stage ?? '-').replace('NORMAL_', '').replace('_', ' ');
        const cmdEl = document.getElementById('gc-command');
        if (cmdEl) cmdEl.textContent = state.command?.type ?? '-';
    }

    activateMoveMode() {
        this.moveMode = true;
        document.getElementById('btn-move-mode')?.classList.add('active');
        if (this.websocket?.readyState === WebSocket.OPEN) {
            this.websocket.send(JSON.stringify({ type: 'activate_move_mode' }));
        }
    }

    deactivateMoveMode() {
        this.moveMode = false;
        this.selectedRobotId = null;
        document.getElementById('btn-move-mode')?.classList.remove('active');
        const label = document.getElementById('selected-robot-label');
        if (label) label.textContent = '--';
        this.renderer?.invalidate();
    }

    sendMoveCommand(robotId, targetX, targetY) {
        const robot = this.robotsOurs[robotId];
        if (!robot || this.websocket?.readyState !== WebSocket.OPEN) return;
        this.websocket.send(JSON.stringify({
            type: 'move_robot', robot_id: robotId,
            target_x: targetX, target_y: targetY, target_theta: robot.theta ?? 0,
        }));
    }

    // HALT を安全に実行: モード外から Escape を押した場合は確認ダイアログを挟む
    _requestHalt() {
        if (this._haltPending) {
            // 2秒以内に再度 Escape → 確定 HALT
            clearTimeout(this._haltTimer);
            this._haltPending = false;
            this._closeHaltDialog();
            this.gcClient.newCommand('HALT');
            return;
        }
        this._haltPending = true;
        this._showHaltDialog();
        this._haltTimer = setTimeout(() => {
            this._haltPending = false;
            this._closeHaltDialog();
        }, 2000);
    }

    _showHaltDialog() {
        let dlg = document.getElementById('halt-confirm-dialog');
        if (!dlg) {
            dlg = document.createElement('div');
            dlg.id = 'halt-confirm-dialog';
            dlg.style.cssText = [
                'position:fixed', 'top:50%', 'left:50%',
                'transform:translate(-50%,-50%)',
                'background:var(--md-sys-color-error-container)',
                'color:var(--md-sys-color-on-error-container)',
                'border:2px solid var(--md-sys-color-error)',
                'border-radius:var(--md-sys-shape-md)',
                'padding:16px 24px', 'z-index:9999',
                'font-size:0.9rem', 'text-align:center',
                'pointer-events:none',
            ].join(';');
            document.body.appendChild(dlg);
        }
        dlg.textContent = 'HALT ALL ROBOTS? — Press Escape again within 2s to confirm';
        dlg.style.display = 'block';
    }

    _closeHaltDialog() {
        const dlg = document.getElementById('halt-confirm-dialog');
        if (dlg) dlg.style.display = 'none';
    }

    setupDelegatedListeners() {
        document.addEventListener('click', (e) => {
            const btn = e.target.closest('[data-action]');
            if (!btn) return;
            const action = btn.dataset.action;
            const team = btn.dataset.team;
            const type = btn.dataset.type;

            switch (action) {
                case 'gc-command':
                    this.gcClient.newCommand(type, team || 'UNKNOWN');
                    this.logPanel?.appendLog('action', 'GC', `command ${type}${team ? ' for ' + team : ''}`);
                    break;
                case 'gc-goals': {
                    const delta = parseInt(btn.dataset.delta ?? '0', 10);
                    this.gcClient.updateGoals(team, delta);
                    this.logPanel?.appendLog('action', 'GC', `goals ${team} ${delta >= 0 ? '+' : ''}${delta}`);
                    break;
                }
                case 'gc-card-yellow':
                    this.gcClient.addYellowCard(team);
                    this.logPanel?.appendLog('action', 'GC', `yellow card for ${team}`);
                    break;
                case 'gc-card-red':
                    this.gcClient.addRedCard(team);
                    this.logPanel?.appendLog('action', 'GC', `red card for ${team}`);
                    break;
                case 'gc-next-stage':
                    this.gcClient.nextStage();
                    this.logPanel?.appendLog('action', 'GC', 'next stage');
                    break;
                case 'ball-place':
                    this.activateBallPlacement(team);
                    this.logPanel?.appendLog('action', 'GC', `ball placement mode: ${team}`);
                    break;
                case 'sim-edit':
                    this.toggleSimEditMode();
                    break;
                case 'sim-reset-ball':
                    this.resetBallToCenter();
                    this.logPanel?.appendLog('action', 'Sim', 'ball reset to center');
                    break;
                case 'sim-set-endpoint':
                    this.applySimEndpoint();
                    this.logPanel?.appendLog('action', 'Sim', 'endpoint updated');
                    break;
            }
        });
    }

    setupLogPanelControls() {
        document.getElementById('btn-log-clear')?.addEventListener('click', () => {
            this.logPanel?.clear();
        });
        document.querySelectorAll('.log-level-cb').forEach(cb => {
            cb.addEventListener('change', () => {
                const active = [...document.querySelectorAll('.log-level-cb:checked')].map(c => c.dataset.level);
                this.logPanel?.setLevelFilter(active);
            });
        });
        const textFilter = document.getElementById('log-text-filter');
        textFilter?.addEventListener('input', () => {
            this.logPanel?.setTextFilter(textFilter.value);
        });
    }

    setupEventListeners() {
        document.getElementById('btn-select-all')?.addEventListener('click', () => {
            for (const name of this.layerStore.keys()) this.visibleLayers.add(name);
            this.updateLayerList();
            this.renderer?.invalidate();
        });
        document.getElementById('btn-deselect-all')?.addEventListener('click', () => {
            this.visibleLayers.clear();
            this.updateLayerList();
            this.renderer?.invalidate();
        });
        document.getElementById('btn-move-mode')?.addEventListener('click', () => {
            this.moveMode ? this.deactivateMoveMode() : this.activateMoveMode();
        });
        document.getElementById('btn-zoom-in')?.addEventListener('click', () => {
            this.zoomLevel = Math.min(this.zoomLevel * 1.2, 5.0);
            this.renderer?.invalidate();
        });
        document.getElementById('btn-zoom-out')?.addEventListener('click', () => {
            this.zoomLevel = Math.max(this.zoomLevel / 1.2, 0.1);
            this.renderer?.invalidate();
        });
        document.getElementById('btn-zoom-reset')?.addEventListener('click', () => {
            this.zoomLevel = 1.0;
            this.panOffset = { x: 0, y: 0 };
            this.renderer?.invalidate();
        });

        const canvas = document.getElementById('field-canvas');
        if (!canvas) return;

        canvas.addEventListener('wheel', (e) => {
            e.preventDefault();
            const factor = e.deltaY < 0 ? 1.1 : 0.9;
            // zoom-to-cursor: カーソル位置を固定点として拡縮
            const fp = this.clientToFieldCoords(e.clientX, e.clientY);
            const newZoom = Math.min(Math.max(this.zoomLevel * factor, 0.1), 5.0);
            const zoomRatio = newZoom / this.zoomLevel;
            // パンオフセットを調整してカーソル下のフィールド座標を固定
            const fl = this.fieldLayer;
            const svgX = fp.x * 1000;
            const svgY = -fp.y * 1000;
            this.panOffset.x = svgX + (this.panOffset.x - svgX) * zoomRatio;
            this.panOffset.y = svgY + (this.panOffset.y - svgY) * zoomRatio;
            this.zoomLevel = newZoom;
            this.renderer?.invalidate();
        }, { passive: false });

        canvas.addEventListener('mousedown', (e) => {
            if (e.button !== 0) return;
            this._dragStartX = e.clientX;
            this._dragStartY = e.clientY;
            this._dragMoved = false;
            if (e.shiftKey && this.moveMode) {
                const fp = this.clientToFieldCoords(e.clientX, e.clientY);
                const hitId = this.findRobotAtPosition(fp.x, fp.y);
                if (hitId !== null) {
                    this.selectedRobotId = hitId;
                    const label = document.getElementById('selected-robot-label');
                    if (label) label.textContent = hitId;
                    this.renderer?.invalidate();
                } else if (this.selectedRobotId !== null) {
                    this.sendMoveCommand(this.selectedRobotId, fp.x, fp.y);
                }
            } else {
                this.isPanning = true;
                this.lastPanPoint = { x: e.clientX, y: e.clientY };
                if (!this.simEditMode && !this.placeBallPending) canvas.style.cursor = 'grabbing';
            }
        });

        canvas.addEventListener('mousemove', (e) => {
            const dx = e.clientX - this._dragStartX, dy = e.clientY - this._dragStartY;
            if (Math.hypot(dx, dy) > 5) this._dragMoved = true;
            this._lastMouseField = this.clientToFieldCoords(e.clientX, e.clientY);
            if (!this.isPanning) return;
            if (this.renderer) {
                const d = this.renderer.pixelDeltaToSvgDelta(
                    e.clientX - this.lastPanPoint.x,
                    e.clientY - this.lastPanPoint.y
                );
                this.panOffset.x += d.dx;
                this.panOffset.y += d.dy;
            }
            this.lastPanPoint = { x: e.clientX, y: e.clientY };
            this.renderer?.invalidate();
        });

        canvas.addEventListener('mouseup', (e) => {
            this.isPanning = false;
            if (!this.simEditMode && !this.placeBallPending) canvas.style.cursor = 'grab';
            if (!this._dragMoved) {
                const fp = this.clientToFieldCoords(e.clientX, e.clientY);
                if (this.placeBallPending) {
                    const team = this.placeBallPending;
                    this.cancelBallPlacement();
                    this.gcClient.setBallPlacementPos(fp.x, fp.y);
                    this.gcClient.newCommand('BALL_PLACEMENT', team);
                } else if (this.simEditMode) {
                    this.simSelectAt(fp.x, fp.y);
                }
            }
        });

        canvas.addEventListener('dblclick', (e) => {
            if (!this.simEditMode || !this.simSelectedObj) return;
            if (this._dragMoved) return;
            const fp = this.clientToFieldCoords(e.clientX, e.clientY);
            this.simTeleportTo(fp.x, fp.y);
        });

        canvas.addEventListener('mouseleave', () => {
            this.isPanning = false;
            if (!this.simEditMode && !this.placeBallPending) canvas.style.cursor = 'default';
        });

        document.addEventListener('keydown', (e) => {
            if (e.key === 'Escape') {
                if (this.placeBallPending) { this.cancelBallPlacement(); return; }
                if (this.simEditMode) { this.toggleSimEditMode(); return; }
                if (this.moveMode) { this.deactivateMoveMode(); return; }
                // モード外の Escape → 確認ダイアログ付き HALT
                this._requestHalt();
                return;
            }
            if (e.key === 'Enter' && this.simEditMode && this.simSelectedObj) {
                this.simTeleportTo(this._lastMouseField.x, this._lastMouseField.y);
            }
        });

        canvas.style.cursor = 'grab';
    }
}

document.addEventListener('DOMContentLoaded', () => {
    window.craneViewer = new CraneViewer();
});
