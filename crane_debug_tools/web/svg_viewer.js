class SvgViewer {
    constructor() {
        this.websocket = null;
        this.currentSvgData = null;
        this.visibleLayers = new Set();
        this.seenLayers = new Set(); // 一度見たレイヤーを記録
        this.zoomLevel = 1.0;
        this.panOffset = { x: 0, y: 0 };
        this.isPanning = false;
        this.lastPanPoint = { x: 0, y: 0 };

        this.lastSnapshotReceivedAt = null;
        this.lastUpdateReceivedAt = null;

        // 高頻度更新の取りこぼし対策（クライアント側でバッファリング＆凝縮）
        this.pendingUpdateBatch = [];
        this.flushTimer = null;
        this.flushIntervalMs = 50; // 最大20FPSで適用

        this.init();
    }

    init() {
        this.setupWebSocket();
        this.setupEventListeners();
        this.updateConnectionStatus(false);

        // 定期的に「〜前」を更新
        setInterval(() => this.updateTimingAges(), 1000);
    }

    setupWebSocket() {
        const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
        const host = window.location.hostname;
        const port = 8091;
        const wsUrl = `${protocol}//${host}:${port}`;

        console.log('WebSocketに接続中:', wsUrl);

        this.websocket = new WebSocket(wsUrl);

        this.websocket.onopen = () => {
            console.log('WebSocket接続が確立されました');
            this.updateConnectionStatus(true);
        };

        this.websocket.onmessage = (event) => {
            try {
                const data = JSON.parse(event.data);
                this.handleWebSocketMessage(data);
            } catch (error) {
                console.error('WebSocketメッセージの解析エラー:', error);
            }
        };

        this.websocket.onclose = () => {
            console.log('WebSocket接続が閉じられました');
            this.updateConnectionStatus(false);

            // 3秒後に再接続を試行
            setTimeout(() => {
                if (!this.websocket || this.websocket.readyState === WebSocket.CLOSED) {
                    console.log('WebSocketの再接続を試行中...');
                    this.setupWebSocket();
                }
            }, 3000);
        };

        this.websocket.onerror = (error) => {
            console.error('WebSocketエラー:', error);
            this.updateConnectionStatus(false);
        };
    }

    handleWebSocketMessage(data) {
        if (data.type === 'svg_data') {
            this.currentSvgData = data;

            // 新しいレイヤーのみを自動的に表示リストに追加
            if (data.layers) {
                data.layers.forEach(layer => {
                    if (!this.seenLayers.has(layer.layer)) {
                        this.seenLayers.add(layer.layer);
                        this.visibleLayers.add(layer.layer);
                    }
                });
            }

            // 更新タイミング表示
            this.lastSnapshotReceivedAt = new Date();
            this.updateTimingDisplays({ snapshot: data });

            this.updateSvgDisplay();
            this.updateLayerList();
            this.updateStats();
        } else if (data.type === 'svg_update') {
            // 高頻度更新はバッファに追加し、一定間隔で凝縮適用
            if (Array.isArray(data.updates) && data.updates.length > 0) {
                this.pendingUpdateBatch.push(...data.updates);
            }

            // 更新タイミング（受信時点）を即時反映
            this.lastUpdateReceivedAt = new Date();
            this.updateTimingDisplays({ update: data });

            // 遅延フラッシュをスケジュール
            this.scheduleFlushUpdates();
        }
    }

    scheduleFlushUpdates() {
        if (this.flushTimer) return;
        this.flushTimer = setTimeout(() => {
            const batch = this.pendingUpdateBatch.splice(0, this.pendingUpdateBatch.length);
            this.flushTimer = null;
            if (batch.length === 0) return;

            // スナップショット未受信時のベース
            if (!this.currentSvgData) {
                this.currentSvgData = { layers: [] };
            }

            // レイヤーごとに凝縮
            const coalesced = this.coalesceLayerUpdates(batch);
            this.applySvgUpdates(coalesced);

            this.updateSvgDisplay();
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
                if (entry.operation === 'replace' || entry.operation === 'append') {
                    entry.svg_primitives.push(...prim);
                    if (!entry.operation) entry.operation = 'append';
                } else if (entry.operation === 'clear') {
                    // clear の後の append は置換として扱う
                    entry.operation = 'replace';
                    entry.svg_primitives = [...prim];
                } else if (!entry.operation) {
                    entry.operation = 'append';
                    entry.svg_primitives = [...prim];
                }
            }
        }

        // 連想配列を配列に戻す
        return Array.from(byLayer.entries()).map(([layer, v]) => ({ layer, ...v }));
    }

    updateTimingDisplays({ snapshot = null, update = null } = {}) {
        if (snapshot) {
            const t = this.formatTime(this.lastSnapshotReceivedAt);
            const el = document.getElementById('lastSnapshotTime');
            if (el) { el.textContent = t; el.title = `ROS stamp_ns: ${snapshot.stamp_ns ?? '-'}`; }
            const seq = document.getElementById('snapshotSeq');
            if (seq) seq.textContent = `(e:${snapshot.epoch ?? '-'} s:${snapshot.seq ?? '-'})`;
        }
        if (update) {
            const t = this.formatTime(this.lastUpdateReceivedAt);
            const el = document.getElementById('lastUpdateTime');
            if (el) { el.textContent = t; el.title = `ROS stamp_ns: ${update.stamp_ns ?? '-'}`; }
            const seq = document.getElementById('updateSeq');
            if (seq) seq.textContent = `(e:${update.epoch ?? '-'} s:${update.seq ?? '-'})`;

            // スナップショット未受信だが更新は来ている場合のヒント表示
            if (!this.lastSnapshotReceivedAt) {
                const sseq = document.getElementById('snapshotSeq');
                if (sseq) sseq.textContent = '(更新のみ受信)';
            }
        }

        // age表示も更新
        this.updateTimingAges();
    }

    formatTime(dateObj) {
        if (!dateObj) return '--:--:--';
        const pad = (n, w = 2) => String(n).padStart(w, '0');
        const h = pad(dateObj.getHours());
        const m = pad(dateObj.getMinutes());
        const s = pad(dateObj.getSeconds());
        return `${h}:${m}:${s}`;
    }

    updateTimingAges() {
        const now = Date.now();
        const ageText = (t) => {
            if (!t) return '';
            const diffMs = Math.max(0, now - t.getTime());
            if (diffMs < 1500) return '(1s未満)';
            if (diffMs < 60000) return `(${Math.floor(diffMs / 1000)}s前)`;
            const m = Math.floor(diffMs / 60000);
            return `(${m}m前)`;
        };

        const snapAgeEl = document.getElementById('snapshotAge');
        if (snapAgeEl) snapAgeEl.textContent = ageText(this.lastSnapshotReceivedAt);

        const updAgeEl = document.getElementById('updateAge');
        if (updAgeEl) updAgeEl.textContent = ageText(this.lastUpdateReceivedAt);
    }

    applySvgUpdates(updates) {
        if (!Array.isArray(updates) || updates.length === 0) return;

        // レイヤー検索ヘルパ
        const findLayerIndex = (name) => {
            if (!this.currentSvgData || !Array.isArray(this.currentSvgData.layers)) return -1;
            return this.currentSvgData.layers.findIndex(l => l.layer === name);
        };

        for (const upd of updates) {
            const layerName = upd.layer;
            const op = (upd.operation || '').toLowerCase();
            const primitives = Array.isArray(upd.svg_primitives) ? upd.svg_primitives : [];

            let idx = findLayerIndex(layerName);

            if (op === 'append') {
                if (idx >= 0) {
                    this.currentSvgData.layers[idx].svg_primitives.push(...primitives);
                } else {
                    // ベースがない場合のappendは無視（ドキュメント方針に準拠）
                }
            } else if (op === 'replace') {
                if (idx >= 0) {
                    this.currentSvgData.layers[idx].svg_primitives = [...primitives];
                } else {
                    // レイヤーが無ければ新規作成
                    this.currentSvgData.layers.push({ layer: layerName, svg_primitives: [...primitives] });
                    // 新規レイヤーは既定で可視化
                    if (!this.seenLayers.has(layerName)) {
                        this.seenLayers.add(layerName);
                        this.visibleLayers.add(layerName);
                    }
                }
            } else if (op === 'clear') {
                if (idx >= 0) {
                    this.currentSvgData.layers[idx].svg_primitives = [];
                } else {
                    // レイヤーが無い場合は何もしない
                }
            } else {
                // 未知のオペレーションは無視
            }
        }
    }

    updateConnectionStatus(connected) {
        const indicator = document.getElementById('connectionIndicator');
        const status = document.getElementById('connectionStatus');

        if (connected) {
            indicator.className = 'status-indicator connected';
            status.textContent = '接続済み';
        } else {
            indicator.className = 'status-indicator disconnected';
            status.textContent = '未接続';
        }
    }

    updateSvgDisplay() {
        const svgViewer = document.getElementById('svgViewer');

        if (!this.currentSvgData || !this.currentSvgData.layers || this.currentSvgData.layers.length === 0) {
            svgViewer.innerHTML = `
                <div class="no-data-message">
                    <div class="text-center">
                        <i class="fas fa-vector-square fa-3x mb-3"></i><br>
                        SVGデータなし<br>
                        <small class="text-muted">レイヤーデータが見つかりません</small>
                    </div>
                </div>
            `;
            return;
        }

        // SVG要素を作成
        const svg = document.createElementNS('http://www.w3.org/2000/svg', 'svg');
        svg.setAttribute('width', '100%');
        svg.setAttribute('height', '100%');
        svg.setAttribute('viewBox', '-6000 -4500 12000 9000'); // SSL field dimensions in mm
        svg.setAttribute('preserveAspectRatio', 'xMidYMid meet');
        svg.style.background = '#6c757d';

        // 座標グリッドを追加
        const defs = document.createElementNS('http://www.w3.org/2000/svg', 'defs');
        const pattern = document.createElementNS('http://www.w3.org/2000/svg', 'pattern');
        pattern.setAttribute('id', 'grid');
        pattern.setAttribute('width', '1000');
        pattern.setAttribute('height', '1000');
        pattern.setAttribute('patternUnits', 'userSpaceOnUse');

        const path = document.createElementNS('http://www.w3.org/2000/svg', 'path');
        path.setAttribute('d', 'M 1000 0 L 0 0 0 1000');
        path.setAttribute('fill', 'none');
        path.setAttribute('stroke', '#adb5bd');
        path.setAttribute('stroke-width', '20');

        pattern.appendChild(path);
        defs.appendChild(pattern);
        svg.appendChild(defs);

        // フィールド背景を追加
        const fieldBackground = document.createElementNS('http://www.w3.org/2000/svg', 'rect');
        fieldBackground.setAttribute('x', '-6000');
        fieldBackground.setAttribute('y', '-4500');
        fieldBackground.setAttribute('width', '12000');
        fieldBackground.setAttribute('height', '9000');
        fieldBackground.setAttribute('fill', '#6c757d');
        svg.appendChild(fieldBackground);

        // グリッドの背景を追加
        const gridRect = document.createElementNS('http://www.w3.org/2000/svg', 'rect');
        gridRect.setAttribute('x', '-6000');
        gridRect.setAttribute('y', '-4500');
        gridRect.setAttribute('width', '12000');
        gridRect.setAttribute('height', '9000');
        gridRect.setAttribute('fill', 'url(#grid)');
        gridRect.setAttribute('opacity', '0.3');
        svg.appendChild(gridRect);

        // 各レイヤーのSVG要素を追加
        this.currentSvgData.layers.forEach(layer => {
            if (this.visibleLayers.has(layer.layer)) {
                const layerGroup = document.createElementNS('http://www.w3.org/2000/svg', 'g');
                layerGroup.setAttribute('class', `layer-${layer.layer}`);

                layer.svg_primitives.forEach(primitive => {
                    try {
                        // SVG文字列をパース
                        const parser = new DOMParser();
                        const svgDoc = parser.parseFromString(`<svg xmlns="http://www.w3.org/2000/svg">${primitive}</svg>`, 'image/svg+xml');

                        // パースしたSVG要素を追加
                        const svgElement = svgDoc.documentElement;
                        for (const child of svgElement.children) {
                            const importedNode = document.importNode(child, true);
                            layerGroup.appendChild(importedNode);
                        }
                    } catch (error) {
                        console.warn('SVG要素のパースエラー:', error, primitive);
                    }
                });

                svg.appendChild(layerGroup);
            }
        });

        // SVGビューアを更新
        svgViewer.innerHTML = '';
        svgViewer.appendChild(svg);

        // パンとズームを適用
        this.applySvgTransform(svg);

        // フォールバック: 表示できているのにラベルが未受信のままの場合に更新印を付ける
        try {
            const hasLayers = Array.isArray(this.currentSvgData?.layers) && this.currentSvgData.layers.length > 0;
            if (hasLayers) {
                if (!this.lastUpdateReceivedAt) {
                    this.lastUpdateReceivedAt = new Date();
                    this.updateTimingDisplays({ update: {} });
                }
                const sseq = document.getElementById('snapshotSeq');
                if (sseq && /未受信/.test(sseq.textContent)) {
                    sseq.textContent = '(表示中)';
                }
                const useq = document.getElementById('updateSeq');
                if (useq && /未受信/.test(useq.textContent)) {
                    useq.textContent = '(表示中)';
                }
            }
        } catch (_) {
            // no-op
        }
    }

    applySvgTransform(svg) {
        const transform = `translate(${this.panOffset.x}px, ${this.panOffset.y}px) scale(${this.zoomLevel})`;
        svg.style.transform = transform;
        svg.style.transformOrigin = 'center';
    }

    updateLayerList() {
        const layerList = document.getElementById('layerList');

        if (!this.currentSvgData || !this.currentSvgData.layers || this.currentSvgData.layers.length === 0) {
            layerList.innerHTML = `
                <div class="text-muted text-center py-3">
                    <i class="fas fa-layer-group fa-2x mb-2"></i><br>
                    レイヤーなし
                </div>
            `;
            return;
        }

        layerList.innerHTML = '';

        this.currentSvgData.layers.forEach(layer => {
            const layerItem = document.createElement('div');
            layerItem.className = 'layer-item';

            const isVisible = this.visibleLayers.has(layer.layer);

            layerItem.innerHTML = `
                <div class="d-flex align-items-center">
                    <input type="checkbox" class="form-check-input layer-checkbox"
                           ${isVisible ? 'checked' : ''}
                           data-layer="${layer.layer}">
                    <span class="layer-name">${layer.layer}</span>
                </div>
                <span class="layer-count">${layer.svg_primitives.length}</span>
            `;

            layerList.appendChild(layerItem);
        });

        // レイヤーチェックボックスのイベントリスナーを追加
        layerList.querySelectorAll('.layer-checkbox').forEach(checkbox => {
            checkbox.addEventListener('change', (e) => {
                const layerName = e.target.dataset.layer;
                if (e.target.checked) {
                    this.visibleLayers.add(layerName);
                } else {
                    this.visibleLayers.delete(layerName);
                }
                this.updateSvgDisplay();
            });
        });
    }

    updateStats() {
        const layerCountEl = document.getElementById('layerCount');
        const primitiveCountEl = document.getElementById('primitiveCount');

        if (this.currentSvgData && this.currentSvgData.layers) {
            layerCountEl.textContent = this.currentSvgData.layers.length;

            const totalPrimitives = this.currentSvgData.layers.reduce(
                (total, layer) => total + layer.svg_primitives.length, 0
            );
            primitiveCountEl.textContent = totalPrimitives;
        } else {
            layerCountEl.textContent = '0';
            primitiveCountEl.textContent = '0';
        }
    }

    setupEventListeners() {
        // レイヤー制御ボタン
        document.getElementById('selectAllLayers').addEventListener('click', () => {
            if (this.currentSvgData && this.currentSvgData.layers) {
                this.currentSvgData.layers.forEach(layer => {
                    this.visibleLayers.add(layer.layer);
                });
                this.updateLayerList();
                this.updateSvgDisplay();
            }
        });

        document.getElementById('deselectAllLayers').addEventListener('click', () => {
            this.visibleLayers.clear();
            this.updateLayerList();
            this.updateSvgDisplay();
        });

        // ズーム制御
        document.getElementById('zoomIn').addEventListener('click', () => {
            this.zoomLevel = Math.min(this.zoomLevel * 1.2, 5.0);
            this.updateSvgDisplay();
        });

        document.getElementById('zoomOut').addEventListener('click', () => {
            this.zoomLevel = Math.max(this.zoomLevel / 1.2, 0.1);
            this.updateSvgDisplay();
        });

        document.getElementById('zoomReset').addEventListener('click', () => {
            this.zoomLevel = 1.0;
            this.panOffset = { x: 0, y: 0 };
            this.updateSvgDisplay();
        });

        // 更新ボタン
        document.getElementById('refreshButton').addEventListener('click', () => {
            if (this.websocket && this.websocket.readyState === WebSocket.OPEN) {
                // WebSocket経由でデータ更新をリクエスト（必要に応じて）
                console.log('手動更新がリクエストされました');
            } else {
                // WebSocket再接続を試行
                this.setupWebSocket();
            }
        });

        // SVGビューアでのマウス操作
        const svgViewer = document.getElementById('svgViewer');

        svgViewer.addEventListener('wheel', (e) => {
            e.preventDefault();

            const rect = svgViewer.getBoundingClientRect();
            const centerX = rect.width / 2;
            const centerY = rect.height / 2;

            const zoomFactor = e.deltaY < 0 ? 1.1 : 0.9;
            const newZoomLevel = Math.min(Math.max(this.zoomLevel * zoomFactor, 0.1), 5.0);

            if (newZoomLevel !== this.zoomLevel) {
                this.zoomLevel = newZoomLevel;
                this.updateSvgDisplay();
            }
        });

        svgViewer.addEventListener('mousedown', (e) => {
            if (e.button === 0) { // 左クリック
                this.isPanning = true;
                this.lastPanPoint = { x: e.clientX, y: e.clientY };
                svgViewer.style.cursor = 'grabbing';
            }
        });

        svgViewer.addEventListener('mousemove', (e) => {
            if (this.isPanning) {
                const deltaX = e.clientX - this.lastPanPoint.x;
                const deltaY = e.clientY - this.lastPanPoint.y;

                this.panOffset.x += deltaX;
                this.panOffset.y += deltaY;

                this.lastPanPoint = { x: e.clientX, y: e.clientY };
                this.updateSvgDisplay();
            }
        });

        svgViewer.addEventListener('mouseup', () => {
            this.isPanning = false;
            svgViewer.style.cursor = 'grab';
        });

        svgViewer.addEventListener('mouseleave', () => {
            this.isPanning = false;
            svgViewer.style.cursor = 'default';
        });

        svgViewer.style.cursor = 'grab';
    }
}

// ページ読み込み完了時にSvgViewerを初期化
document.addEventListener('DOMContentLoaded', () => {
    window.svgViewer = new SvgViewer();
});
