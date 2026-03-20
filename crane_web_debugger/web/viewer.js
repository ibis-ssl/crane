class CraneViewer {
    constructor() {
        this.websocket = null;
        this.currentSvgData = null;
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
        this.controlTargets = {};

        this.fieldUpdateTimer = null;
        this.robotUpdateTimer = null;

        this.init();
    }

    init() {
        this.setupWebSocket();
        this.setupEventListeners();
        this.updateConnectionStatus(false);
    }

    setupWebSocket() {
        const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
        const wsUrl = `${protocol}//${window.location.hostname}:8091`;
        this.websocket = new WebSocket(wsUrl);

        this.websocket.onopen = () => this.updateConnectionStatus(true);

        this.websocket.onmessage = (event) => {
            try {
                this.handleMessage(JSON.parse(event.data));
            } catch (e) {
                console.error('メッセージ解析エラー:', e);
            }
        };

        this.websocket.onclose = () => {
            this.updateConnectionStatus(false);
            setTimeout(() => {
                if (!this.websocket || this.websocket.readyState === WebSocket.CLOSED) {
                    this.setupWebSocket();
                }
            }, 3000);
        };

        this.websocket.onerror = () => this.updateConnectionStatus(false);
    }

    handleMessage(data) {
        switch (data.type) {
            case 'svg_data': this.handleSvgData(data); break;
            case 'svg_update': this.handleSvgUpdate(data); break;
            case 'world_model': this.handleWorldModel(data); break;
            case 'control_targets': this.handleControlTargets(data); break;
            case 'robot_commands': this.handleRobotCommands(data); break;
            case 'game_info': this.handleGameInfo(data); break;
        }
    }

    handleSvgData(data) {
        this.currentSvgData = data;
        if (data.layers) {
            data.layers.forEach(layer => {
                if (!this.seenLayers.has(layer.layer)) {
                    this.seenLayers.add(layer.layer);
                    this.visibleLayers.add(layer.layer);
                }
            });
        }
        this.scheduleFieldUpdate();
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
            if (!this.currentSvgData) this.currentSvgData = { layers: [] };
            this.applySvgUpdates(this.coalesceLayerUpdates(batch));
            this.scheduleFieldUpdate();
            this.updateLayerList();
            this.updateStats();
        }, this.flushIntervalMs);
    }

    scheduleFieldUpdate() {
        if (this.fieldUpdateTimer) return;
        this.fieldUpdateTimer = setTimeout(() => {
            this.fieldUpdateTimer = null;
            this.renderSvgField();
        }, 200);
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
        const findIdx = (name) =>
            this.currentSvgData.layers ? this.currentSvgData.layers.findIndex(l => l.layer === name) : -1;

        for (const upd of updates) {
            const op = (upd.operation || '').toLowerCase();
            const primitives = Array.isArray(upd.svg_primitives) ? upd.svg_primitives : [];
            const idx = findIdx(upd.layer);
            if (op === 'append') {
                if (idx >= 0) this.currentSvgData.layers[idx].svg_primitives.push(...primitives);
            } else if (op === 'replace') {
                if (idx >= 0) {
                    this.currentSvgData.layers[idx].svg_primitives = [...primitives];
                } else {
                    this.currentSvgData.layers.push({ layer: upd.layer, svg_primitives: [...primitives] });
                    if (!this.seenLayers.has(upd.layer)) {
                        this.seenLayers.add(upd.layer);
                        this.visibleLayers.add(upd.layer);
                    }
                }
            } else if (op === 'clear') {
                if (idx >= 0) this.currentSvgData.layers[idx].svg_primitives = [];
            }
        }
    }

    handleWorldModel(data) {
        if (data.robots_ours) {
            this.robotsOurs = {};
            for (const robot of data.robots_ours) {
                this.robotsOurs[robot.id] = robot;
            }
        }
        this.scheduleRobotUpdate();
    }

    handleControlTargets(data) {
        if (data.commands) {
            this.controlTargets = {};
            for (const cmd of data.commands) {
                this.controlTargets[cmd.robot_id] = cmd;
            }
        }
        this.scheduleRobotUpdate();
    }

    handleRobotCommands(data) {
        if (data.commands && Object.keys(this.controlTargets).length === 0) {
            for (const cmd of data.commands) {
                this.controlTargets[cmd.robot_id] = cmd;
            }
            this.scheduleRobotUpdate();
        }
    }

    handleGameInfo(data) {
        const ourScore = document.getElementById('score-our');
        const theirScore = document.getElementById('score-their');
        const situation = document.getElementById('play-situation');
        const stage = document.getElementById('game-stage');
        if (ourScore) ourScore.textContent = data.our_score ?? 0;
        if (theirScore) theirScore.textContent = data.their_score ?? 0;
        if (situation) situation.textContent = data.play_situation || '--';
        if (stage) stage.textContent = data.game_stage || '--';
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
        if (dot) {
            dot.classList.toggle('connected', connected);
        }
        if (label) {
            label.textContent = connected ? '接続済み' : '未接続';
        }
    }

    renderSvgField() {
        const container = document.getElementById('svg-field');
        if (!container) return;

        if (!this.currentSvgData || !this.currentSvgData.layers || this.currentSvgData.layers.length === 0) {
            container.innerHTML = '';
            const text = document.createElementNS('http://www.w3.org/2000/svg', 'text');
            text.setAttribute('x', '50%'); text.setAttribute('y', '50%');
            text.setAttribute('text-anchor', 'middle'); text.setAttribute('dominant-baseline', 'middle');
            text.setAttribute('fill', '#444'); text.setAttribute('font-size', '20');
            text.textContent = 'SVGデータなし - ROS topicを待機中...';
            container.appendChild(text);
            return;
        }

        container.innerHTML = '';

        const defs = document.createElementNS('http://www.w3.org/2000/svg', 'defs');
        const pattern = document.createElementNS('http://www.w3.org/2000/svg', 'pattern');
        pattern.setAttribute('id', 'vgrid');
        pattern.setAttribute('width', '1000'); pattern.setAttribute('height', '1000');
        pattern.setAttribute('patternUnits', 'userSpaceOnUse');
        const gpath = document.createElementNS('http://www.w3.org/2000/svg', 'path');
        gpath.setAttribute('d', 'M 1000 0 L 0 0 0 1000');
        gpath.setAttribute('fill', 'none'); gpath.setAttribute('stroke', '#2d8a2d'); gpath.setAttribute('stroke-width', '15');
        pattern.appendChild(gpath); defs.appendChild(pattern);

        container.setAttribute('viewBox', '-6000 -4500 12000 9000');
        container.setAttribute('preserveAspectRatio', 'xMidYMid meet');
        container.style.background = '#1a5c1a';
        container.appendChild(defs);

        const fieldBg = document.createElementNS('http://www.w3.org/2000/svg', 'rect');
        fieldBg.setAttribute('x', '-6000'); fieldBg.setAttribute('y', '-4500');
        fieldBg.setAttribute('width', '12000'); fieldBg.setAttribute('height', '9000');
        fieldBg.setAttribute('fill', '#1a5c1a');
        container.appendChild(fieldBg);

        const gridRect = document.createElementNS('http://www.w3.org/2000/svg', 'rect');
        gridRect.setAttribute('x', '-6000'); gridRect.setAttribute('y', '-4500');
        gridRect.setAttribute('width', '12000'); gridRect.setAttribute('height', '9000');
        gridRect.setAttribute('fill', 'url(#vgrid)'); gridRect.setAttribute('opacity', '0.25');
        container.appendChild(gridRect);

        // transform group for pan/zoom
        const transformGroup = document.createElementNS('http://www.w3.org/2000/svg', 'g');
        transformGroup.id = 'svg-transform-group';

        this.currentSvgData.layers.forEach(layer => {
            if (!this.visibleLayers.has(layer.layer)) return;
            const g = document.createElementNS('http://www.w3.org/2000/svg', 'g');
            g.setAttribute('class', `layer-${layer.layer}`);
            layer.svg_primitives.forEach(primitive => {
                try {
                    const parser = new DOMParser();
                    const doc = parser.parseFromString(`<svg xmlns="http://www.w3.org/2000/svg">${primitive}</svg>`, 'image/svg+xml');
                    for (const child of doc.documentElement.children) {
                        g.appendChild(document.importNode(child, true));
                    }
                } catch (e) { /* skip */ }
            });
            transformGroup.appendChild(g);
        });

        container.appendChild(transformGroup);
        this.applyTransform();
    }

    applyTransform() {
        const group = document.getElementById('svg-transform-group');
        if (group) {
            group.setAttribute('transform', `translate(${this.panOffset.x}, ${this.panOffset.y}) scale(${this.zoomLevel})`);
        }
    }

    renderRobotCards() {
        const container = document.getElementById('robot-cards-grid');
        if (!container) return;
        const ids = Object.keys(this.robotsOurs).map(Number).sort((a, b) => a - b);

        if (ids.length === 0) {
            container.innerHTML = '<div style="font-size:0.7rem; color:#666; text-align:center; grid-column:span 4; padding:8px;">ロボットデータなし</div>';
            return;
        }

        container.innerHTML = '';
        const CONTROL_MODE_NAMES = { 0: 'CAM', 1: 'POS', 2: 'VEL', 3: 'POL' };

        for (const id of ids) {
            const robot = this.robotsOurs[id];
            const cmd = this.controlTargets[id];
            const modeName = cmd ? (CONTROL_MODE_NAMES[cmd.control_mode] ?? `M${cmd.control_mode}`) : '--';
            const plannerName = cmd?.planner_name || '--';

            const card = document.createElement('div');
            card.className = 'robot-card';
            card.innerHTML = `
                <div class="robot-id">${id}</div>
                <div class="robot-mode">${modeName}</div>
                <div class="robot-planner" title="${plannerName}">${plannerName.substring(0, 6)}</div>
            `;
            card.addEventListener('click', () => this.showRobotDetail(id));
            container.appendChild(card);
        }
    }

    showRobotDetail(id) {
        const robot = this.robotsOurs[id];
        const cmd = this.controlTargets[id];
        if (!robot) return;

        const modal = document.getElementById('robot-detail-modal');
        if (!modal) return;

        const CONTROL_MODE_NAMES = { 0: 'LOCAL_CAMERA', 1: 'POSITION_TARGET', 2: 'SIMPLE_VELOCITY', 3: 'POLAR_VELOCITY' };
        const modeName = cmd ? (CONTROL_MODE_NAMES[cmd.control_mode] ?? `MODE_${cmd.control_mode}`) : '--';

        document.getElementById('robot-detail-title').textContent = `Robot ${id}`;

        let targetHtml = '';
        if (cmd?.position_target_mode) {
            targetHtml = `<tr><td class="text-muted">目標位置</td><td>(${cmd.position_target_mode.target_x?.toFixed(3)}, ${cmd.position_target_mode.target_y?.toFixed(3)})</td></tr>`;
        } else if (cmd?.simple_velocity_target_mode) {
            targetHtml = `<tr><td class="text-muted">目標速度</td><td>(${cmd.simple_velocity_target_mode.target_vx?.toFixed(3)}, ${cmd.simple_velocity_target_mode.target_vy?.toFixed(3)})</td></tr>`;
        }

        document.getElementById('robot-detail-body').innerHTML = `
            <table class="table table-sm table-dark table-borderless mb-2">
                <tbody>
                    <tr><td class="text-muted" style="width:6em">制御モード</td><td>${modeName}</td></tr>
                    <tr><td class="text-muted">プランナー</td><td>${cmd?.planner_name || '--'}</td></tr>
                    <tr><td class="text-muted">推定 X</td><td>${robot.x?.toFixed(3)} m</td></tr>
                    <tr><td class="text-muted">推定 Y</td><td>${robot.y?.toFixed(3)} m</td></tr>
                    <tr><td class="text-muted">推定 θ</td><td>${robot.theta?.toFixed(3)} rad</td></tr>
                    <tr><td class="text-muted">速度 Vx</td><td>${robot.vx?.toFixed(3)} m/s</td></tr>
                    <tr><td class="text-muted">速度 Vy</td><td>${robot.vy?.toFixed(3)} m/s</td></tr>
                    <tr><td class="text-muted">ω</td><td>${robot.omega?.toFixed(3)} rad/s</td></tr>
                    <tr><td class="text-muted">Vision X</td><td>${robot.vision_x?.toFixed(3) ?? '--'}</td></tr>
                    <tr><td class="text-muted">Vision Y</td><td>${robot.vision_y?.toFixed(3) ?? '--'}</td></tr>
                    ${targetHtml}
                    <tr><td class="text-muted">目標 θ</td><td>${cmd?.target_theta?.toFixed(3) ?? '--'}</td></tr>
                </tbody>
            </table>
            <a href="/robot_telemetry.html?id=${id}" class="btn btn-sm btn-primary w-100">
                <i class="fas fa-chart-line me-1"></i>テレメトリを開く
            </a>
        `;

        bootstrap.Modal.getOrCreateInstance(modal).show();
    }

    updateLayerList() {
        const container = document.getElementById('layer-list');
        if (!container) return;

        if (!this.currentSvgData?.layers?.length) {
            container.innerHTML = '<div style="font-size:0.7rem; color:#666; text-align:center; padding:6px;">レイヤーなし</div>';
            return;
        }

        container.innerHTML = '';
        this.currentSvgData.layers.forEach(layer => {
            const item = document.createElement('div');
            item.className = 'layer-item';
            item.innerHTML = `
                <input type="checkbox" class="form-check-input layer-cb" id="layer-${layer.layer}"
                       data-layer="${layer.layer}" ${this.visibleLayers.has(layer.layer) ? 'checked' : ''}>
                <label class="form-check-label" for="layer-${layer.layer}">${layer.layer}</label>
                <span class="ms-auto text-muted" style="font-size:0.65rem">${layer.svg_primitives.length}</span>
            `;
            container.appendChild(item);
        });

        container.querySelectorAll('.layer-cb').forEach(cb => {
            cb.addEventListener('change', (e) => {
                const name = e.target.dataset.layer;
                if (e.target.checked) this.visibleLayers.add(name);
                else this.visibleLayers.delete(name);
                this.scheduleFieldUpdate();
            });
        });
    }

    updateStats() {
        const layerEl = document.getElementById('layer-count');
        const primEl = document.getElementById('prim-count');
        if (!this.currentSvgData?.layers) return;
        if (layerEl) layerEl.textContent = this.currentSvgData.layers.length;
        if (primEl) {
            const total = this.currentSvgData.layers.reduce((s, l) => s + l.svg_primitives.length, 0);
            primEl.textContent = total;
        }
    }

    setupEventListeners() {
        document.getElementById('btn-select-all')?.addEventListener('click', () => {
            this.currentSvgData?.layers?.forEach(l => this.visibleLayers.add(l.layer));
            this.updateLayerList();
            this.scheduleFieldUpdate();
        });

        document.getElementById('btn-deselect-all')?.addEventListener('click', () => {
            this.visibleLayers.clear();
            this.updateLayerList();
            this.scheduleFieldUpdate();
        });

        document.getElementById('btn-zoom-in')?.addEventListener('click', () => {
            this.zoomLevel = Math.min(this.zoomLevel * 1.2, 5.0);
            this.applyTransform();
        });

        document.getElementById('btn-zoom-out')?.addEventListener('click', () => {
            this.zoomLevel = Math.max(this.zoomLevel / 1.2, 0.1);
            this.applyTransform();
        });

        document.getElementById('btn-zoom-reset')?.addEventListener('click', () => {
            this.zoomLevel = 1.0;
            this.panOffset = { x: 0, y: 0 };
            this.applyTransform();
        });

        const svgContainer = document.getElementById('svg-container');
        if (svgContainer) {
            svgContainer.addEventListener('wheel', (e) => {
                e.preventDefault();
                const factor = e.deltaY < 0 ? 1.1 : 0.9;
                this.zoomLevel = Math.min(Math.max(this.zoomLevel * factor, 0.1), 5.0);
                this.applyTransform();
            });

            svgContainer.addEventListener('mousedown', (e) => {
                if (e.button === 0) {
                    this.isPanning = true;
                    this.lastPanPoint = { x: e.clientX, y: e.clientY };
                    svgContainer.style.cursor = 'grabbing';
                }
            });

            svgContainer.addEventListener('mousemove', (e) => {
                if (!this.isPanning) return;
                // pan in SVG coordinates
                const svgEl = document.getElementById('svg-field');
                if (svgEl) {
                    const vb = svgEl.viewBox.baseVal;
                    const rect = svgContainer.getBoundingClientRect();
                    const scaleX = vb.width / rect.width;
                    const scaleY = vb.height / rect.height;
                    this.panOffset.x += (e.clientX - this.lastPanPoint.x) * scaleX / this.zoomLevel;
                    this.panOffset.y += (e.clientY - this.lastPanPoint.y) * scaleY / this.zoomLevel;
                }
                this.lastPanPoint = { x: e.clientX, y: e.clientY };
                this.applyTransform();
            });

            svgContainer.addEventListener('mouseup', () => {
                this.isPanning = false;
                svgContainer.style.cursor = 'grab';
            });

            svgContainer.addEventListener('mouseleave', () => {
                this.isPanning = false;
                svgContainer.style.cursor = 'default';
            });

            svgContainer.style.cursor = 'grab';
        }
    }
}

document.addEventListener('DOMContentLoaded', () => {
    window.craneViewer = new CraneViewer();
});
