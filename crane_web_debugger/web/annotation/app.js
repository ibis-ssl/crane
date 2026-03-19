// Crane Annotation Tool - Main Application
class CraneAnnotationApp {
  constructor() {
    this.ws = null;
    this.isConnected = false;
    this.timeOffset = 0;
    this.rewindSeconds = 0;
    this.rewindLocked = false;
    this.history = [];
    this.speechRecognition = null;
    this.isRecording = false;

    // モーダル内のコンテキスト選択状態（アノテーション送信時のみ使用）
    this.modalRobots = []; // { id: number, isOurs: boolean }[]
    this.modalPosition = null; // { x: number, y: number } | null
    this.tempRobotSelection = { team: 'ours', ids: [] };
    this.tempPosition = null;

    // モーダルのプリセット情報
    this.modalPreset = { category: null, label: null };

    // PWAインストール
    this.deferredPrompt = null;

    this.init();
  }

  async init() {
    console.log('Crane Annotation Tool initializing...');

    // PWA登録
    if ('serviceWorker' in navigator) {
      navigator.serviceWorker.register('sw.js')
        .then(() => console.log('Service Worker registered'))
        .catch(err => console.error('Service Worker registration failed:', err));
    }

    // PWAインストールプロンプト
    this.setupPWAInstall();

    this.setupSpeechRecognition();
    this.setupEventListeners();
    try {
      await this.connect();
    } catch (err) {
      console.warn('Initial connection failed:', err);
    }
    this.startTimeSyncLoop();
  }

  setupPWAInstall() {
    // beforeinstallpromptイベントをキャプチャ
    window.addEventListener('beforeinstallprompt', (e) => {
      console.log('PWA install prompt available');
      e.preventDefault();
      this.deferredPrompt = e;
      // インストールボタンを表示
      document.getElementById('install-btn').classList.remove('hidden');
    });

    // アプリがインストールされたらボタンを非表示
    window.addEventListener('appinstalled', () => {
      console.log('PWA installed');
      this.deferredPrompt = null;
      document.getElementById('install-btn').classList.add('hidden');
    });
  }

  // WebSocket接続
  async connect() {
    const host = window.location.hostname || 'localhost';
    const port = 8091;
    const url = `ws://${host}:${port}`;

    console.log(`Connecting to WebSocket: ${url}`);

    return new Promise((resolve, reject) => {
      this.ws = new WebSocket(url);

      this.ws.onopen = () => {
        console.log('WebSocket connected');
        this.isConnected = true;
        this.updateConnectionStatus(true);
        this.requestTimeSync();
        resolve();
      };

      this.ws.onmessage = (event) => {
        try {
          const data = JSON.parse(event.data);
          this.handleMessage(data);
        } catch (e) {
          console.error('Failed to parse message:', e);
        }
      };

      this.ws.onclose = () => {
        console.log('WebSocket disconnected');
        this.isConnected = false;
        this.updateConnectionStatus(false);
        // 3秒後に再接続
        setTimeout(() => this.connect(), 3000);
      };

      this.ws.onerror = (err) => {
        console.error('WebSocket error:', err);
        reject(err);
      };
    });
  }

  // メッセージハンドラ
  handleMessage(data) {
    switch (data.type) {
      case 'time_sync_response':
        this.handleTimeSyncResponse(data);
        break;
      case 'annotation_ack':
        console.log('Annotation acknowledged:', data);
        break;
      case 'game_info':
        this.updateGameInfo(data);
        break;
      case 'error':
        console.error('Server error:', data.message);
        alert('エラー: ' + data.message);
        break;
      default:
        console.log('Unknown message type:', data.type);
    }
  }

  updateGameInfo(data) {
    // プレイ状況
    const playSituation = data.play_situation || '---';
    document.getElementById('play-situation').textContent = playSituation;

    // スコア
    const ourScore = data.our_score || 0;
    const theirScore = data.their_score || 0;
    document.getElementById('score').textContent = `${ourScore} - ${theirScore}`;

    // 試合時間
    const gameTime = data.game_time || 0;
    const minutes = Math.floor(gameTime / 60);
    const seconds = Math.floor(gameTime % 60);
    document.getElementById('game-time').textContent = `${minutes}:${seconds.toString().padStart(2, '0')}`;

    // ゲームステージ
    const gameStage = data.game_stage || '---';
    document.getElementById('game-stage').textContent = gameStage;

    // ゲームイベント（ファールなど）
    const gameEvent = data.game_event || '';
    const banner = document.getElementById('game-event-banner');
    const eventText = document.getElementById('game-event-text');

    if (gameEvent && gameEvent.trim() !== '') {
      eventText.textContent = gameEvent;
      banner.classList.remove('hidden');
    } else {
      banner.classList.add('hidden');
    }
  }

  // 時刻同期
  requestTimeSync() {
    if (!this.isConnected) return;

    const T1 = Date.now();
    this.ws.send(JSON.stringify({
      type: 'time_sync_request',
      client_send_time_ms: T1
    }));
  }

  handleTimeSyncResponse(data) {
    const T4 = Date.now();
    const T1 = data.client_send_time_ms;
    const T2 = data.server_receive_time_ms;
    const T3 = data.server_send_time_ms;

    // RTT計算
    const rtt = (T4 - T1) - (T3 - T2);
    // オフセット計算
    this.timeOffset = Math.round(((T2 - T1) + (T3 - T4)) / 2);

    this.updateTimeSyncStatus(this.timeOffset, rtt);
  }

  startTimeSyncLoop() {
    // 10秒ごとに時刻同期
    setInterval(() => {
      if (this.isConnected) {
        this.requestTimeSync();
      }
    }, 10000);
  }

  updateConnectionStatus(connected) {
    const indicator = document.getElementById('connection-indicator');
    const status = document.getElementById('connection-status');

    if (connected) {
      indicator.className = 'indicator online';
      status.textContent = '接続中';
    } else {
      indicator.className = 'indicator offline';
      status.textContent = '未接続';
    }
  }

  updateTimeSyncStatus(offset, rtt) {
    const status = document.getElementById('time-sync-status');
    status.textContent = `同期: ${offset >= 0 ? '+' : ''}${offset}ms (RTT:${Math.round(rtt)}ms)`;
  }

  // アノテーション送信
  sendAnnotation(category, label, description = '', priority = 1, options = {}) {
    if (!this.isConnected) {
      alert('サーバーに接続されていません');
      return;
    }

    const now = Date.now();
    let eventTime;

    if (this.rewindLocked && this.rewindSeconds < 0) {
      // リワインドがロックされている場合は過去の時刻
      eventTime = now + (this.rewindSeconds * 1000) + this.timeOffset;
    } else {
      // リアルタイム
      eventTime = now + this.timeOffset;
    }

    const annotation = {
      type: 'annotation',
      category: parseInt(category),
      priority: parseInt(priority),
      label: label,
      description: description,
      event_timestamp_ns: eventTime * 1000000, // ms -> ns
      client_timestamp_ms: now,
      time_offset_ms: this.timeOffset,
      ...options
    };

    // コンテキスト情報を追加（options経由で渡される）
    // modalRobotsとmodalPositionは呼び出し側で処理される

    console.log('Sending annotation:', annotation);
    this.ws.send(JSON.stringify(annotation));
    this.addToHistory(annotation);
    this.vibrate();
  }

  // 履歴追加
  addToHistory(annotation) {
    this.history.unshift({
      ...annotation,
      timestamp: new Date()
    });

    // 最大50件まで保持
    if (this.history.length > 50) {
      this.history.pop();
    }

    this.updateHistoryUI();
  }

  updateHistoryUI() {
    const list = document.getElementById('history-list');
    list.innerHTML = '';

    this.history.forEach((item, index) => {
      const li = document.createElement('li');

      const categoryName = ['問題', '良い動き', 'メモ'][item.category] || 'その他';

      li.innerHTML = `
        <div><strong>${item.label}</strong> (${categoryName})</div>
        ${item.description ? `<div style="margin-top:4px;font-size:12px;">${item.description}</div>` : ''}
        <div class="time">${item.timestamp.toLocaleTimeString()}</div>
      `;

      list.appendChild(li);
    });
  }

  // 触覚フィードバック
  vibrate() {
    if ('vibrate' in navigator) {
      navigator.vibrate(50);
    }
  }

  // 音声認識セットアップ
  setupSpeechRecognition() {
    if ('webkitSpeechRecognition' in window || 'SpeechRecognition' in window) {
      const SpeechRecognition = window.SpeechRecognition || window.webkitSpeechRecognition;
      this.speechRecognition = new SpeechRecognition();
      this.speechRecognition.lang = 'ja-JP';
      this.speechRecognition.continuous = false;
      this.speechRecognition.interimResults = false;

      this.speechRecognition.onresult = (event) => {
        const transcript = event.results[0][0].transcript;
        document.getElementById('modal-description').value = transcript;
        this.isRecording = false;
        this.updateVoiceButton();
      };

      this.speechRecognition.onerror = (event) => {
        console.error('Speech recognition error:', event.error);
        this.isRecording = false;
        this.updateVoiceButton();
      };

      this.speechRecognition.onend = () => {
        this.isRecording = false;
        this.updateVoiceButton();
      };
    } else {
      console.warn('Speech recognition not supported');
    }
  }

  startVoiceInput() {
    if (!this.speechRecognition) {
      alert('音声入力はこのブラウザではサポートされていません');
      return;
    }

    if (this.isRecording) {
      this.speechRecognition.stop();
      this.isRecording = false;
    } else {
      this.speechRecognition.start();
      this.isRecording = true;
    }

    this.updateVoiceButton();
  }

  updateVoiceButton() {
    const btn = document.getElementById('voice-input-btn');
    const icon = document.getElementById('voice-icon');
    const text = btn.querySelector('span:last-child');

    if (this.isRecording) {
      btn.classList.add('recording');
      icon.textContent = '⏹️';
      text.textContent = '停止';
    } else {
      btn.classList.remove('recording');
      icon.textContent = '🎤';
      text.textContent = '音声入力';
    }
  }

  // イベントリスナーセットアップ
  setupEventListeners() {
    // クイックボタン（モーダルを開く）
    document.querySelectorAll('.annotation-btn').forEach(btn => {
      btn.addEventListener('click', () => {
        const category = btn.dataset.category;
        const label = btn.dataset.label;
        this.showDetailModal({ category, label });
      });
    });

    // リワインドスライダー
    const slider = document.getElementById('rewind-slider');
    slider.addEventListener('input', (e) => {
      this.rewindSeconds = parseInt(e.target.value);
      this.updateRewindDisplay();
    });

    // リワインドロックボタン
    document.getElementById('rewind-lock').addEventListener('click', () => {
      this.rewindLocked = !this.rewindLocked;
      this.updateRewindLockDisplay();
    });

    // モーダル
    document.getElementById('modal-overlay').addEventListener('click', () => {
      this.hideDetailModal();
    });

    document.getElementById('modal-cancel').addEventListener('click', () => {
      this.hideDetailModal();
    });

    document.getElementById('modal-submit').addEventListener('click', () => {
      this.submitDetailAnnotation();
    });

    // 音声入力
    document.getElementById('voice-input-btn').addEventListener('click', () => {
      this.startVoiceInput();
    });

    // 履歴トグル
    document.getElementById('history-toggle').addEventListener('click', () => {
      this.toggleHistory();
    });

    document.getElementById('history-close').addEventListener('click', () => {
      this.hideHistory();
    });

    // モーダル内のコンテキスト選択
    document.getElementById('modal-robot-select-btn').addEventListener('click', () => {
      this.showRobotModal();
    });

    document.getElementById('modal-position-select-btn').addEventListener('click', () => {
      this.showFieldModal();
    });

    // ロボット選択モーダル
    document.getElementById('robot-modal-overlay').addEventListener('click', () => {
      this.hideRobotModal();
    });

    document.getElementById('robot-modal-cancel').addEventListener('click', () => {
      this.hideRobotModal();
    });

    document.getElementById('robot-modal-ok').addEventListener('click', () => {
      this.applyRobotSelection();
    });

    document.getElementById('team-ours').addEventListener('click', () => {
      this.selectTeam('ours');
    });

    document.getElementById('team-theirs').addEventListener('click', () => {
      this.selectTeam('theirs');
    });

    // フィールドマップモーダル
    document.getElementById('field-modal-overlay').addEventListener('click', () => {
      this.hideFieldModal();
    });

    document.getElementById('field-modal-cancel').addEventListener('click', () => {
      this.hideFieldModal();
    });

    document.getElementById('field-modal-ok').addEventListener('click', () => {
      this.applyFieldSelection();
    });

    const fieldSvg = document.getElementById('field-svg');
    fieldSvg.addEventListener('click', (e) => {
      this.handleFieldClick(e);
    });
    fieldSvg.addEventListener('touchstart', (e) => {
      e.preventDefault();
      this.handleFieldClick(e.touches[0]);
    });

    // PWAインストールボタン
    document.getElementById('install-btn').addEventListener('click', () => {
      this.installPWA();
    });

    // ロボットIDグリッドの初期化
    this.initRobotIdGrid();
  }

  async installPWA() {
    if (!this.deferredPrompt) {
      return;
    }

    // インストールプロンプトを表示
    this.deferredPrompt.prompt();

    // ユーザーの選択を待つ
    const { outcome } = await this.deferredPrompt.userChoice;
    console.log(`User response to install prompt: ${outcome}`);

    if (outcome === 'accepted') {
      console.log('User accepted the install prompt');
    } else {
      console.log('User dismissed the install prompt');
    }

    // プロンプトを使用したのでクリア
    this.deferredPrompt = null;
    document.getElementById('install-btn').classList.add('hidden');
  }

  updateRewindDisplay() {
    const value = document.getElementById('rewind-value');
    value.textContent = this.rewindSeconds === 0 ? 'リアルタイム' : `${this.rewindSeconds}秒前`;
  }

  updateRewindLockDisplay() {
    const btn = document.getElementById('rewind-lock');
    const icon = document.getElementById('lock-icon');
    const text = document.getElementById('lock-text');

    if (this.rewindLocked) {
      btn.classList.add('locked');
      icon.textContent = '🔒';
      text.textContent = 'ロック中';
    } else {
      btn.classList.remove('locked');
      icon.textContent = '🔓';
      text.textContent = 'リアルタイム';
    }
  }

  showDetailModal(preset = {}) {
    // プリセット情報を保存
    this.modalPreset = preset;

    // タイトルを更新
    const categoryNames = ['⚠️ 問題', '✨ 良い動き', '📝 メモ'];
    const title = preset.label || categoryNames[preset.category] || 'アノテーション';
    document.getElementById('modal-title').textContent = title;

    // フォームを初期化
    document.getElementById('modal-description').value = '';

    // コンテキスト選択をリセット
    this.modalRobots = [];
    this.modalPosition = null;
    this.updateModalContextDisplay();

    document.getElementById('detail-modal').classList.remove('hidden');
  }

  hideDetailModal() {
    document.getElementById('detail-modal').classList.add('hidden');
    // フォームリセット
    document.getElementById('modal-description').value = '';

    // コンテキストリセット
    this.modalRobots = [];
    this.modalPosition = null;
    this.modalPreset = { category: null, label: null };
  }

  submitDetailAnnotation() {
    // プリセットからカテゴリとラベルを取得
    const category = this.modalPreset.category || '0';
    const label = this.modalPreset.label || 'アノテーション';
    const description = document.getElementById('modal-description').value.trim();
    const priority = 1; // 固定値（中）

    // コンテキスト情報をoptionsとして渡す
    const options = {};

    if (this.modalRobots.length > 0) {
      options.related_robot_ids = this.modalRobots.map(r => r.id);
      options.robot_is_ours = this.modalRobots.map(r => r.isOurs);
    }

    if (this.modalPosition) {
      options.position = {
        x: this.modalPosition.x,
        y: this.modalPosition.y,
        z: 0
      };
    }

    this.sendAnnotation(category, label, description, priority, options);
    this.hideDetailModal();
  }

  toggleHistory() {
    const panel = document.getElementById('history-panel');
    panel.classList.toggle('hidden');
  }

  hideHistory() {
    document.getElementById('history-panel').classList.add('hidden');
  }

  // ロボットIDグリッドの初期化
  initRobotIdGrid() {
    const grid = document.getElementById('robot-id-grid');
    grid.innerHTML = '';

    for (let i = 0; i < 16; i++) {
      const btn = document.createElement('button');
      btn.className = 'robot-id-btn';
      btn.textContent = i;
      btn.dataset.id = i;
      btn.addEventListener('click', () => this.toggleRobotId(i));
      grid.appendChild(btn);
    }
  }

  // ロボット選択モーダル表示
  showRobotModal() {
    // 現在のモーダル選択状態をtempにコピー
    this.tempRobotSelection = {
      team: this.modalRobots.length > 0 ? (this.modalRobots[0].isOurs ? 'ours' : 'theirs') : 'ours',
      ids: this.modalRobots.map(r => r.id)
    };

    // UIを更新
    this.updateRobotModalUI();

    document.getElementById('robot-modal').classList.remove('hidden');
  }

  hideRobotModal() {
    document.getElementById('robot-modal').classList.add('hidden');
  }

  selectTeam(team) {
    this.tempRobotSelection.team = team;
    this.updateRobotModalUI();
  }

  toggleRobotId(id) {
    const index = this.tempRobotSelection.ids.indexOf(id);
    if (index >= 0) {
      this.tempRobotSelection.ids.splice(index, 1);
    } else {
      this.tempRobotSelection.ids.push(id);
    }
    this.updateRobotModalUI();
  }

  updateRobotModalUI() {
    // チームボタン
    document.getElementById('team-ours').classList.toggle('active', this.tempRobotSelection.team === 'ours');
    document.getElementById('team-theirs').classList.toggle('active', this.tempRobotSelection.team === 'theirs');

    // IDボタン
    document.querySelectorAll('.robot-id-btn').forEach(btn => {
      const id = parseInt(btn.dataset.id);
      btn.classList.toggle('selected', this.tempRobotSelection.ids.includes(id));
    });
  }

  applyRobotSelection() {
    this.modalRobots = this.tempRobotSelection.ids.map(id => ({
      id: id,
      isOurs: this.tempRobotSelection.team === 'ours'
    }));

    this.updateModalContextDisplay();
    this.hideRobotModal();
    this.vibrate();
  }

  // フィールドマップモーダル表示
  showFieldModal() {
    this.tempPosition = this.modalPosition ? { ...this.modalPosition } : null;
    this.updateFieldMarker();
    document.getElementById('field-modal').classList.remove('hidden');
  }

  hideFieldModal() {
    document.getElementById('field-modal').classList.add('hidden');
  }

  handleFieldClick(event) {
    const svg = document.getElementById('field-svg');
    const rect = svg.getBoundingClientRect();

    // クリック位置をSVG座標に変換
    const clientX = event.clientX !== undefined ? event.clientX : event.pageX;
    const clientY = event.clientY !== undefined ? event.clientY : event.pageY;

    const x = (clientX - rect.left) / rect.width * 9 - 4.5;
    const y = (clientY - rect.top) / rect.height * 6 - 3;

    this.tempPosition = {
      x: Math.round(x * 100) / 100,
      y: Math.round(y * 100) / 100
    };

    this.updateFieldMarker();
  }

  updateFieldMarker() {
    const marker = document.getElementById('position-marker');
    const coords = document.getElementById('position-coords');

    if (this.tempPosition) {
      marker.setAttribute('cx', this.tempPosition.x);
      marker.setAttribute('cy', this.tempPosition.y);
      marker.style.display = 'block';
      coords.textContent = `x: ${this.tempPosition.x.toFixed(2)}m, y: ${this.tempPosition.y.toFixed(2)}m`;
    } else {
      marker.style.display = 'none';
      coords.textContent = 'x: --, y: --';
    }
  }

  applyFieldSelection() {
    this.modalPosition = this.tempPosition;
    this.updateModalContextDisplay();
    this.hideFieldModal();
    this.vibrate();
  }

  // モーダル内のコンテキスト表示更新
  updateModalContextDisplay() {
    const robotBtn = document.getElementById('modal-robot-select-btn');
    const robotLabel = document.getElementById('modal-robot-label');
    const positionBtn = document.getElementById('modal-position-select-btn');
    const positionLabel = document.getElementById('modal-position-label');

    // ロボット選択表示
    if (this.modalRobots.length > 0) {
      const teamName = this.modalRobots[0].isOurs ? '自チーム' : '相手チーム';
      const ids = this.modalRobots.map(r => r.id).sort((a, b) => a - b).join(', ');
      robotLabel.textContent = `${teamName}: #${ids}`;
      robotBtn.classList.add('has-selection');
    } else {
      robotLabel.textContent = '選択なし';
      robotBtn.classList.remove('has-selection');
    }

    // 位置選択表示
    if (this.modalPosition) {
      positionLabel.textContent = `(${this.modalPosition.x.toFixed(1)}, ${this.modalPosition.y.toFixed(1)})`;
      positionBtn.classList.add('has-selection');
    } else {
      positionLabel.textContent = '選択なし';
      positionBtn.classList.remove('has-selection');
    }
  }
}

// アプリ起動
const app = new CraneAnnotationApp();

console.log('Crane Annotation Tool loaded');
