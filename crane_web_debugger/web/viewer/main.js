// Viewer の組み立て役。
//
// 役割は「各モジュールを繋ぐこと」と「カメラ（ズーム・パン）を持つこと」だけ。
// ワールドモデルは ViewerState、操作モードは ModeMachine、canvas 入力は
// PointerRouter、data-action は ActionDispatcher、WebSocket は WsHub が持つ。
// ここに状態や分岐を書き足しそうになったら、置き場所を間違えている。

import { ROBOT_HIT_RADIUS_M, ZOOM_MIN, ZOOM_MAX } from './renderer/constants.js';
import { getFsmState } from './renderer/formatters.js';
import { SvgPrimitiveParser } from './renderer/SvgPrimitiveParser.js';
import { CanvasRenderer } from './renderer/CanvasRenderer.js';
import { FieldLayer } from './renderer/FieldLayer.js';
import { ThemeTokens } from './renderer/ThemeTokens.js';
import { PointerRouter } from './renderer/PointerRouter.js';
import { WsHub } from './ws/WsHub.js';
import { GameControlClient } from './ws/GameControlClient.js';
import { ViewerState } from './state/ViewerState.js';
import { ModeMachine } from './state/ModeMachine.js';
import { LayerStore } from './state/LayerStore.js';
import { Drawers } from './ui/Drawers.js';
import { RobotRail } from './ui/RobotRail.js';
import { StatusStrip } from './ui/StatusStrip.js';
import { CommandPalette } from './ui/CommandPalette.js';
import { PositionControlPanel } from './ui/PositionControlPanel.js';
import { ActionDispatcher } from './ui/ActionDispatcher.js';
import { FocusSidebar } from './ui/FocusSidebar.js';
import { OverviewTab } from './sidebar/OverviewTab.js';
import { TelemetryTab } from './sidebar/TelemetryTab.js';
import { TestTab } from './sidebar/TestTab.js';
import { ROUTE_DIRECT } from './state/TestSession.js';
import { LogTab } from './sidebar/LogTab.js';
import { LogPanel } from './ui/LogPanel.js';
import { RingBuffer } from './replay/RingBuffer.js';
import { TimeScrubber } from './ui/TimeScrubber.js';

const WS_PORT = 8091;
const DETAIL_REFRESH_MS = 500;
const KEYBOARD_PAN_SPEED = 80;
const KEYBOARD_PAN_FAST = 3;
const HALT_CONFIRM_MS = 2000;
const ZOOM_BUTTON_STEP = 1.2;
const HOVER_RADIUS_M = ROBOT_HIT_RADIUS_M * 2;

class CraneViewer {
    constructor() {
        // --- カメラ ---
        this.zoomLevel = 1.0;
        this.panOffset = { x: 0, y: 0 };
        this.isPanning = false;
        this.lastPanPoint = { x: 0, y: 0 };
        this.lastMouseField = { x: 0, y: 0 };
        this._prevFieldLength = null;

        // --- 状態 ---
        this.state = new ViewerState();
        this.modes = new ModeMachine();
        this.layerStore = new LayerStore(() => this._refreshLayers());

        // --- 通信 ---
        this.hub = new WsHub(`${location.protocol === 'https:' ? 'wss:' : 'ws:'}//${location.hostname}:${WS_PORT}`);
        this.gcClient = new GameControlClient();

        // --- 描画 ---
        this.parser = new SvgPrimitiveParser();
        this.fieldLayer = new FieldLayer();
        this.themeTokens = new ThemeTokens();
        this.renderer = null;
        this.pointer = null;
        this.ringBuffer = new RingBuffer();
        this.timeScrubber = null;
        this._replayMode = false;

        // --- UI ---
        this.drawers = null;
        this.statusStrip = null;
        this.palette = null;
        this.positionControl = null;
        this.actions = null;
        this.sidebar = null;
        this.logPanel = null;

        this._detailTimer = null;
        this._keysDown = new Set();
        this._keyboardLoopRunning = false;
        this._haltPending = false;
        this._haltTimer = null;
        this._hoveredTooltipId = null;
        this.testSession = null;

        this.init();
    }

    init() {
        const canvas = document.getElementById('field-canvas');
        if (canvas) this.renderer = new CanvasRenderer(canvas, this, this.fieldLayer, this.themeTokens);
        this.themeTokens.mount(() => this.renderer?.invalidate());

        const logBody = document.getElementById('log-panel-body');
        if (logBody) this.logPanel = new LogPanel(logBody);

        this.sidebar = new FocusSidebar(this);
        this.sidebar.register('overview', new OverviewTab(this.state, this.themeTokens));
        this.sidebar.register('telemetry', new TelemetryTab(this));
        this.sidebar.register('test', new TestTab(this));
        this.sidebar.register('log', new LogTab(this.logPanel));
        this.drawers = new Drawers();
        this.rail = new RobotRail(this);
        this.actions = new ActionDispatcher(this);
        this.statusStrip = new StatusStrip(this.actions);
        this.palette = new CommandPalette(this.actions);

        const pcRoot = document.getElementById('position-control-panel');
        if (pcRoot) this.positionControl = new PositionControlPanel(this, pcRoot);

        this.modes.setHooks({
            onEnterMove: () => this.hub.send({ type: 'activate_move_mode' }),
            onEnterTest: () => this._syncTestChrome(),
            // 解除の送信はここに置く。Escape ラダーからの exitTop() も
            // ボタンからの deactivateTest() も必ずここを通るため、
            // 「UI は off なのに crane 側はテストセッションのまま」にならない。
            //
            // 指令経路が「直接」でも同じ deactivate_robot_test を送ってよい。
            // websocket_server.cpp の 3 ハンドラはいずれもサーバ側に状態を持たず、
            // session_injection を publish するだけだから:
            //   activate_move_mode   → "HALT"
            //   activate_robot_test  → "ROBOT_TEST"
            //   deactivate_robot_test→ "HALT"
            // 直接経路はもともと HALT なので、解除は HALT の再送になって無害。
            // メッセージ名に robot_test と付くが、実体は「HALT へ戻す」汎用の解除。
            onExitTest: () => {
                this.hub.send({ type: 'deactivate_robot_test' });
                this.logPanel?.appendLog('action', 'TEST', 'deactivate → HALT');
                this._syncTestChrome();
            },
            onChange: () => this.renderer?.invalidate(),
        });

        this._setupHub();
        this._setupStaticControls();
        if (canvas) this.pointer = new PointerRouter(this, canvas);
        this._setupKeyboard();
        this._setupLogPanelControls();

        document.getElementById('btn-robot-detail-close')
            ?.addEventListener('click', () => this.closeRobotDetail());

        const tsContainer = document.getElementById('time-scrubber-container');
        if (tsContainer) this.timeScrubber = new TimeScrubber(tsContainer, this.ringBuffer, this);

        this._setConnected(false);
        this.sidebar.applyUrlParams();
        this.gcClient.connect(window.location.hostname);
        this.gcClient.onStateChange = (state) => this.statusStrip.updateFromGc(state);
    }

    // ===== 互換アクセサ =====
    // CanvasRenderer / RobotHud / PositionControlPanel が viewer 直下を読むため、
    // ViewerState・ModeMachine への委譲を残す。新しいコードは state / modes を直接使うこと。
    get robotsOurs() { return this.state.robotsOurs; }
    get robotsTheirs() { return this.state.robotsTheirs; }
    get controlTargets() { return this.state.controlTargets; }
    get robotFeedback() { return this.state.robotFeedback; }
    get latencyEstimation() { return this.state.latencyEstimation; }
    get ballPos() { return this.state.ballPos; }
    get isYellow() { return this.state.isYellow; }
    get focusedRobotId() { return this.state.focusedRobotId; }
    get _multiSelect() { return this.state.multiSelect; }
    get _hoveredRobotId() { return this.state.hoveredRobotId; }
    get _feedbackTimestamp() { return this.state.feedbackTimestamp; }
    get moveMode() { return this.modes.move; }
    get simEditMode() { return this.modes.simEdit; }
    get simSelectedObj() { return this.modes.simSelectedObj; }
    get websocket() { return this.hub.socket; }
    get visibleLayers() { return this.layerStore.visible; }

    // ===== 受信 =====

    _setupHub() {
        const hub = this.hub;
        hub.onStatus((ok) => {
            this._setConnected(ok);
            if (ok) this.positionControl?.requestConfig();
        });
        hub.onLog((level, tag, msg) => this.logPanel?.appendLog(level, tag, msg));

        hub.subscribe('svg_data', (d) => this._onSvgData(d));
        hub.subscribe('svg_update', (d) => this._onSvgUpdate(d));
        hub.subscribe('world_model', (d) => this._onWorldModel(d));
        hub.subscribe('control_targets', (d) => this._onControlTargets(d));
        hub.subscribe('robot_commands', (d) => {
            // robot_commands は control_targets が流れてこない構成向けのフォールバック
            if (d.commands && Object.keys(this.state.controlTargets).length === 0) {
                this.state.ingestControlTargets(d.commands);
                this._scheduleDetailRefresh();
            }
        });
        hub.subscribe('robot_feedback', (d) => {
            // websocket_server.cpp は "robots" キーで送る
            if (d.robots) this.state.ingestFeedback(d.robots);
            if (this.state.focusedRobotId !== null) this._scheduleDetailRefresh();
        });
        hub.subscribe('latency_estimation', (d) => {
            if (!d.estimations) return;
            this.state.ingestLatency(d.estimations);
            this.sidebar.refresh();
            this.renderer?.invalidate();
        });
        hub.subscribe('game_info', (d) => this.statusStrip.updateFromGameInfo(d));
        hub.subscribe('situations_list', (d) => this._onSituationsList(d));
        hub.subscribe('session_injection_current', (d) => this._onSessionInjection(d));
        hub.subscribe('position_control_config', (d) => this.positionControl?.handleConfig(d));
        hub.subscribe('set_position_control_param_result', (d) => this.positionControl?.handleSetResult(d));
        hub.connect();
    }

    _refreshLayers() {
        this.renderer?.invalidate();
        this.layerStore.renderList(() => this.renderer?.invalidate());
        this.layerStore.renderStats();
    }

    _onSvgData(data) {
        const tsMs = data.stamp_ns ? data.stamp_ns / 1e6 : Date.now();
        if (this._replayMode) {
            this.ringBuffer.addDelta(tsMs, 'svg_data', data);
            return;
        }
        this.layerStore.replaceAll(data.layers);
        this.ringBuffer.addKeyframe(tsMs, {
            layerStore: this.layerStore.layers,
            robotsOurs: this.state.robotsOurs,
            robotsTheirs: this.state.robotsTheirs,
            ball: this.state.ballPos,
            controlTargets: this.state.controlTargets,
        });
        this._refreshLayers();
    }

    _onSvgUpdate(data) {
        this.ringBuffer.addDelta(Date.now(), 'svg_update', data);
        if (this._replayMode) return;
        this.layerStore.queueUpdates(data.updates);
    }

    _onWorldModel(data) {
        const tsMs = data.timestamp ? data.timestamp * 1e-6 : Date.now();
        this.ringBuffer.addDelta(tsMs, 'world_model', data);
        if (this._replayMode) return;

        this.state.ingestWorldModel(data);

        // field_info 動的 viewBox: フィールドサイズが変わった時だけズームをリセットする
        if (data.field_info && this.fieldLayer.updateFromFieldInfo(data.field_info)) {
            const newLen = data.field_info.length;
            if (this._prevFieldLength !== null && newLen !== this._prevFieldLength) {
                this.zoomLevel = 1.0;
                this.panOffset = { x: 0, y: 0 };
            }
            this._prevFieldLength = newLen;
            const fl = this.fieldLayer;
            this.parser.setViewBox(fl.vbX, fl.vbY, fl.vbW, fl.vbH);
            this.layerStore.markAllDirty();
            this.renderer?.invalidate();
        }

        this._scheduleDetailRefresh();
        if (this.modes.move && this.state.focusedRobotId !== null) this.renderer?.invalidate();
        if (this.modes.simEdit) this.renderer?.invalidate();
    }

    _onControlTargets(data) {
        this.ringBuffer.addDelta(Date.now(), 'control_targets', data);
        if (this._replayMode) return;
        if (data.commands) this.state.ingestControlTargets(data.commands);
        this._scheduleDetailRefresh();
    }

    _onSituationsList(data) {
        const sel = document.getElementById('session-select');
        if (!sel) return;
        const current = sel.value;
        sel.innerHTML = '';
        for (const name of (data.items || [])) {
            const opt = document.createElement('option');
            opt.value = name;
            opt.textContent = name;
            if (name === current) opt.selected = true;
            sel.appendChild(opt);
        }
        if (!sel.options.length) {
            const opt = document.createElement('option');
            opt.value = '';
            opt.textContent = 'No situations loaded';
            sel.appendChild(opt);
        }
    }

    _onSessionInjection(data) {
        this.statusStrip.updateSession(data.name);
        const currentEl = document.getElementById('session-current');
        if (currentEl) currentEl.textContent = data.name || '-';
        const historyEl = document.getElementById('session-history');
        if (!historyEl) return;
        historyEl.innerHTML = '';
        for (const entry of (data.history || [])) {
            const li = document.createElement('li');
            li.textContent = `${new Date(entry.timestamp_ms).toLocaleTimeString()}  ${entry.name}`;
            historyEl.appendChild(li);
        }
    }

    _setConnected(connected) {
        const dot = document.getElementById('connection-dot');
        const label = document.getElementById('connection-label');
        const banner = document.getElementById('offline-banner');
        if (dot) dot.classList.toggle('connected', connected);
        if (label) label.textContent = connected ? 'connected' : 'disconnected';
        if (banner) banner.classList.toggle('visible', !connected);
    }

    // リプレイ: RingBuffer から復元したフレームで現在の表示状態を差し替える。
    // state / layerStore は getter 経由なので、外から直接代入させない。
    applyReplayFrame(frame) {
        this._replayMode = true;
        this.layerStore.layers = frame.layerStore;
        this.state.robotsOurs = frame.robotsOurs;
        this.state.robotsTheirs = frame.robotsTheirs;
        this.state.ballPos = frame.ball;
        this.state.controlTargets = frame.controlTargets;
        this.renderer?.invalidate();
    }

    // ===== フォーカス =====

    toggleRobotDetail(id) {
        if (this.state.focusedRobotId === id) this.closeRobotDetail();
        else this.showRobotDetail(id);
    }

    // tabName を渡すとそのタブで開く（URL 契約 ?robot=&tab= の受け口）
    showRobotDetail(id, tabName = null) {
        this.state.setFocus(id);
        if (!this.sidebar.open(id, tabName)) return;
        this._syncFocusLabel();
        this.renderer?.invalidate();
    }

    closeRobotDetail() {
        this.sidebar.close();
        this.state.setFocus(null);
        this._syncFocusLabel();
        this.renderer?.invalidate();
    }

    _syncFocusLabel() {
        const label = document.getElementById('selected-robot-label');
        if (label) label.textContent = this.state.focusedRobotId ?? '--';
    }

    _scheduleDetailRefresh() {
        if (this._detailTimer) return;
        this._detailTimer = setTimeout(() => {
            this._detailTimer = null;
            this.sidebar.refresh();
        }, DETAIL_REFRESH_MS);
    }

    // ===== 座標とホバー =====

    clientToFieldCoords(clientX, clientY) {
        return this.renderer ? this.renderer.clientToFieldCoords(clientX, clientY) : { x: 0, y: 0 };
    }

    fieldToClientCoords(fieldX, fieldY) {
        if (!this.renderer) return { x: 0, y: 0 };
        const { vs, ox, oy } = this.renderer._getVP();
        const fl = this.fieldLayer;
        const svgX = (fieldX * 1000 - fl.vbX + this.panOffset.x) * this.zoomLevel;
        const svgY = (-fieldY * 1000 - fl.vbY + this.panOffset.y) * this.zoomLevel;
        const rect = this.renderer.canvas.getBoundingClientRect();
        return { x: rect.left + svgX * vs + ox, y: rect.top + svgY * vs + oy };
    }

    updateHover(fieldX, fieldY) {
        // ホバー検出半径は選択より広め
        const closest = this.state.nearestRobot(this.state.robotsOurs, fieldX, fieldY, HOVER_RADIUS_M)?.id ?? null;
        if (closest !== this.state.hoveredRobotId) {
            this.state.hoveredRobotId = closest;
            this.renderer?.invalidate();
        }
        this.updateTooltip(closest);
    }

    updateTooltip(robotId) {
        let tt = document.getElementById('robot-hover-tooltip');
        if (!tt) {
            tt = document.createElement('div');
            tt.id = 'robot-hover-tooltip';
            tt.className = 'cv-hover-tooltip';
            document.body.appendChild(tt);
        }
        const robot = robotId === null ? null : this.state.robotsOurs[robotId];
        if (!robot) {
            tt.style.display = 'none';
            this._hoveredTooltipId = null;
            return;
        }
        const pos = this.fieldToClientCoords(robot.x, robot.y);
        tt.style.left = `${pos.x + 16}px`;
        tt.style.top = `${pos.y - 16}px`;
        tt.style.display = 'block';
        if (this._hoveredTooltipId === robotId) return;

        this._hoveredTooltipId = robotId;
        const cmd = this.state.controlTargets[robotId];
        tt.innerHTML = `
                <b>Robot ${robotId}</b><br>
                Pos: (${robot.x?.toFixed(2)}, ${robot.y?.toFixed(2)}) θ=${robot.theta?.toFixed(2)}<br>
                FSM: ${getFsmState(cmd) ?? '--'}<br>
                Planner: ${cmd?.planner_name ?? '--'}
            `;
    }

    // ===== 指令 =====

    sendMoveCommand(robotId, targetX, targetY) {
        const robot = this.state.robotsOurs[robotId];
        if (!robot) return;
        this.hub.send({
            type: 'move_robot', robot_id: robotId,
            target_x: targetX, target_y: targetY, target_theta: robot.theta ?? 0,
        });
    }

    // ===== テストモード =====
    // C-3。ロボット単位ではなく crane 全体のモードで、解除は HALT を注入する。

    setTestSession(session) {
        this.testSession = session;
        session.onChange(() => this.renderer?.invalidate());
        this.renderer?.invalidate();
    }

    activateTest() {
        const s = this.testSession;
        if (!s) return;
        // 指令経路で有効化のメッセージが変わる。
        // プランナ経由 → セッション注入 ROBOT_TEST / 直接 → HALT
        this.hub.send({
            type: s.route === ROUTE_DIRECT ? 'activate_move_mode' : 'activate_robot_test',
            robot_id: s.robotId,
            max_velocity: s.maxVelocity,
            max_acceleration: s.maxAcceleration,
        });
        this.modes.enterTest();
        this.logPanel?.appendLog('action', 'TEST', `activate (${s.route}) robot #${s.robotId}`);
    }

    deactivateTest() { this.modes.exitTest(); }

    sendTestTarget() {
        const s = this.testSession;
        if (!s || !this.modes.test || !s.targetPos) return;
        if (s.route === ROUTE_DIRECT) {
            // プランナを迂回して /control_targets へ。速度上限は乗らない
            this.sendMoveCommand(s.robotId, s.targetPos.x, s.targetPos.y);
            return;
        }
        this.hub.send({
            type: 'robot_test_target',
            robot_id: s.robotId,
            target_x: s.targetPos.x,
            target_y: s.targetPos.y,
            target_theta: s.targetTheta,
            max_velocity: s.maxVelocity,
            max_acceleration: s.maxAcceleration,
        });
    }

    sendPlannerDamping() {
        const s = this.testSession;
        if (!s) return;
        this.hub.send({ type: 'set_planner_param', velocity_damping_gain: s.dampingGain });
    }

    // C-3 のピルとヘルプバー。所有権がテストレイヤーにあることを画面上で示す
    _syncTestChrome() {
        const pill = document.getElementById('test-ownership-pill');
        const help = document.getElementById('test-help-bar');
        const on = this.modes.test;
        if (pill) {
            pill.classList.toggle('visible', on);
            pill.textContent = `TEST MODE · #${this.testSession?.robotId ?? '--'}`;
        }
        help?.classList.toggle('visible', on);
        this.renderer?.invalidate();
    }

    simSelectAt(fieldX, fieldY) {
        this.modes.simSelectedObj = this.state.pickSimObject(fieldX, fieldY);
        this.modes.syncSimLabel();
        this.renderer?.invalidate();
    }

    simTeleportTo(fieldX, fieldY) {
        const obj = this.modes.simSelectedObj;
        if (!obj) return;
        if (obj.type === 'ball') {
            this.hub.send({ type: 'sim_teleport_ball', x: fieldX, y: fieldY, vx: 0, vy: 0 });
        } else {
            this.hub.send({
                type: 'sim_teleport_robot',
                id: obj.id,
                team: obj.yellow ? 'yellow' : 'blue',
                x: fieldX, y: fieldY,
                orientation_deg: (obj.theta ?? 0) * 180 / Math.PI,
            });
        }
    }

    // HALT を安全に実行: モード外から Escape を押した場合だけ確認を挟む。
    // コマンドバーの HALT ボタンは data-action 経由で即時発火する
    // （緊急停止に確認を挟まない）。この非対称は意図的。
    _requestHalt() {
        if (this._haltPending) {
            clearTimeout(this._haltTimer);
            this._haltPending = false;
            this._toggleHaltDialog(false);
            this.gcClient.newCommand('HALT');
            return;
        }
        this._haltPending = true;
        this._toggleHaltDialog(true);
        this._haltTimer = setTimeout(() => {
            this._haltPending = false;
            this._toggleHaltDialog(false);
        }, HALT_CONFIRM_MS);
    }

    _toggleHaltDialog(show) {
        let dlg = document.getElementById('halt-confirm-dialog');
        if (!dlg) {
            dlg = document.createElement('div');
            dlg.id = 'halt-confirm-dialog';
            dlg.className = 'cv-halt-dialog';
            dlg.textContent = 'HALT ALL ROBOTS? — Press Escape again within 2s to confirm';
            document.body.appendChild(dlg);
        }
        dlg.style.display = show ? 'block' : 'none';
    }

    // ===== 固定コントロール =====

    _setupStaticControls() {
        const redraw = () => this.renderer?.invalidate();
        document.getElementById('btn-select-all')?.addEventListener('click', () => {
            this.layerStore.selectAll();
            this.layerStore.renderList(redraw);
            redraw();
        });
        document.getElementById('btn-deselect-all')?.addEventListener('click', () => {
            this.layerStore.deselectAll();
            this.layerStore.renderList(redraw);
            redraw();
        });
        document.getElementById('btn-move-mode')?.addEventListener('click', () => {
            if (this.modes.test) {
                this.deactivateTest();
            } else {
                const targetId = this.focusSidebar?.currentRobotId ?? this.selectedRobotId ?? Object.keys(this.state.robotsOurs)[0];
                if (targetId !== undefined && targetId !== null) {
                    this.focusSidebar?.open(Number(targetId), 'test');
                    this.activateTest();
                } else {
                    this.modes.toggleMove();
                }
            }
        });
        document.getElementById('btn-zoom-in')?.addEventListener('click', () => {
            this.zoomLevel = Math.min(this.zoomLevel * ZOOM_BUTTON_STEP, ZOOM_MAX);
            redraw();
        });
        document.getElementById('btn-zoom-out')?.addEventListener('click', () => {
            this.zoomLevel = Math.max(this.zoomLevel / ZOOM_BUTTON_STEP, ZOOM_MIN);
            redraw();
        });
        document.getElementById('btn-zoom-reset')?.addEventListener('click', () => {
            this.zoomLevel = 1.0;
            this.panOffset = { x: 0, y: 0 };
            redraw();
        });
    }

    _setupLogPanelControls() {
        document.getElementById('btn-log-clear')?.addEventListener('click', () => this.logPanel?.clear());
        document.querySelectorAll('.log-level-cb').forEach(cb => {
            cb.addEventListener('change', () => {
                const active = [...document.querySelectorAll('.log-level-cb:checked')].map(c => c.dataset.level);
                this.logPanel?.setLevelFilter(active);
            });
        });
        const textFilter = document.getElementById('log-text-filter');
        textFilter?.addEventListener('input', () => this.logPanel?.setTextFilter(textFilter.value));
    }

    // ===== キーボード =====

    _setupKeyboard() {
        document.addEventListener('keydown', (e) => {
            const tag = document.activeElement?.tagName;
            const isInput = tag === 'INPUT' || tag === 'TEXTAREA';

            if ((e.metaKey || e.ctrlKey) && e.key.toLowerCase() === 'k') {
                e.preventDefault();
                this.palette?.toggle();
                return;
            }
            if (e.key === 'Escape') { this._onEscape(); return; }
            if (e.key === 'Enter' && this.modes.simEdit && this.modes.simSelectedObj) {
                this.simTeleportTo(this.lastMouseField.x, this.lastMouseField.y);
                return;
            }
            if (isInput) return;
            const panKeys = ['w', 'a', 's', 'd', 'ArrowUp', 'ArrowLeft', 'ArrowDown', 'ArrowRight', 'Shift'];
            if (panKeys.includes(e.key)) {
                e.preventDefault();
                this._keysDown.add(e.key);
                this._startKeyboardLoop();
            }
        });
        document.addEventListener('keyup', (e) => this._keysDown.delete(e.key));
    }

    // Escape ラダー。3 段目までの正本は ModeMachine の LADDER。
    //   1. コマンドパレット
    //   2. ドロワー          ← Drawers が capture 段階で処理して伝播を止める
    //   3. 指令モード        ← ModeMachine.exitTop()（test → move → ballPlacement → simEdit）
    //   4. フォーカス
    //   5. 複数選択
    //   6. 何も無ければ確認ダイアログ付き HALT
    _onEscape() {
        if (this.palette?.isOpen) { this.palette.close(); return; }
        if (this.modes.exitTop()) { this.renderer?.invalidate(); return; }
        if (this.state.focusedRobotId !== null) { this.closeRobotDetail(); return; }
        if (this.state.multiSelect.size > 0) {
            this.state.multiSelect.clear();
            this.renderer?.invalidate();
            return;
        }
        this._requestHalt();
    }

    _startKeyboardLoop() {
        if (this._keyboardLoopRunning) return;
        this._keyboardLoopRunning = true;
        const loop = () => {
            if (this._keysDown.size === 0) { this._keyboardLoopRunning = false; return; }
            const speed = KEYBOARD_PAN_SPEED * (this._keysDown.has('Shift') ? KEYBOARD_PAN_FAST : 1);
            if (this._keysDown.has('a') || this._keysDown.has('ArrowLeft')) this.panOffset.x += speed;
            if (this._keysDown.has('d') || this._keysDown.has('ArrowRight')) this.panOffset.x -= speed;
            if (this._keysDown.has('w') || this._keysDown.has('ArrowUp')) this.panOffset.y += speed;
            if (this._keysDown.has('s') || this._keysDown.has('ArrowDown')) this.panOffset.y -= speed;
            this.renderer?.invalidate();
            requestAnimationFrame(loop);
        };
        requestAnimationFrame(loop);
    }
}

document.addEventListener('DOMContentLoaded', () => {
    window.craneViewer = new CraneViewer();
});
