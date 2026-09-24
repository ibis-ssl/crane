// 位置制御ゲインの遠隔調整パネル（ibis-ssl/crane#1442）
//
// ibis_sender は 1 秒ごとに position_control.* を読み直し、28 バイト（v2）の設定パケットと
// して CM4 へ送る。CM4 側で位置制御ループが閉じているので、ここでの ros2 param set 相当の
// 操作が、ロボットを再起動せずにゲインを変える唯一の経路。
//
// CM4 側の制御則は PID だが、ki / kd の既定は 0 で、そのときは従来の P 制御に
// 恒等的に縮退する。つまりこのパネルで ki / kd を上げるまで挙動は従来どおり。
//
// 押さえておくべき癖が 3 つある:
//   1. 反映は「送信のたび」ではなく ibis_sender の 1 秒周期。押してすぐには変わらない
//   2. CM4 は範囲外の値をクランプせずデータグラムごと捨てる。しかも検査はデータグラム
//      単位なので、ki だけが範囲外でも kp を含めて 1 つも適用されない。拒否理由は
//      CM4 のログにしか出ないので、範囲外は送る前に弾く（websocket_server 側も同じ範囲）
//      なお設定パケットに後方互換は無い。CM4 が古い機体では kp すら変わらない
//   3. 5 項目とも CM4 のゲインに効く。crane 側に位置制御ループは無い

const PARAMS = [
    {
        name: 'position_control.kp',
        label: '位置ゲイン',
        unit: '',
        min: 0, max: 20, step: 0.1,
        help: '目標位置へ向かう P ゲイン。大きいほど機敏だが振動しやすい',
    },
    {
        name: 'position_control.ki',
        label: '積分ゲイン',
        unit: '',
        min: 0, max: 20, step: 0.1,
        help: '定常偏差（詰めきれない残り誤差）を消す I ゲイン。0 で P 制御。大きいほど粘るが行き過ぎやすい',
    },
    {
        name: 'position_control.kd',
        label: '微分ゲイン',
        unit: '',
        min: 0, max: 5, step: 0.01,
        help: '実測速度を打ち消す D ゲイン（微分先行形）。0 で P 制御。行き過ぎと振動を抑える',
    },
    {
        name: 'position_control.deceleration',
        label: '減速度',
        unit: 'm/s²',
        min: 0, max: 20, step: 0.1,
        help: '停止時の制動エンベロープ。小さいほど手前から緩やかに減速する',
    },
    {
        name: 'position_control.tolerance',
        label: '許容誤差',
        unit: 'm',
        min: 0, max: 1, step: 0.005,
        help: '目標に到達したとみなす距離',
    },
];

// スライダを掴んでいる間は送らない。1 ドラッグで数十回 param set するのを避ける。
const APPLY_DEBOUNCE_MS = 500;
// ibis_sender の送信周期 1 秒ぶんの猶予を見込んだ、ロボットへの反映目安
const ROBOT_APPLY_HINT_MS = 1000;

export class PositionControlPanel {
    constructor(viewer, root) {
        this._viewer = viewer;
        this._root = root;
        this._rows = new Map();     // name -> {input, slider, status, spec}
        this._timers = new Map();   // name -> debounce timer id
        this._build();
    }

    _build() {
        this._root.innerHTML = '';

        this._banner = document.createElement('div');
        this._banner.className = 'pc-banner';
        this._root.appendChild(this._banner);
        this._setBanner('neutral', '読み込み中…');

        for (const spec of PARAMS) {
            this._root.appendChild(this._buildRow(spec));
        }

        const foot = document.createElement('div');
        foot.className = 'pc-foot';
        const reload = document.createElement('button');
        reload.type = 'button';
        reload.className = 'm3-btn m3-btn--text m3-btn--sm';
        reload.textContent = '再取得';
        reload.title = 'ibis_sender から現在値を読み直す';
        reload.addEventListener('click', () => this.requestConfig());
        foot.appendChild(reload);

        const note = document.createElement('span');
        note.className = 'pc-note';
        note.textContent = '反映は ibis_sender の送信周期（1 秒）ごと';
        foot.appendChild(note);
        this._root.appendChild(foot);
    }

    _buildRow(spec) {
        const row = document.createElement('div');
        row.className = 'pc-row';

        const head = document.createElement('div');
        head.className = 'pc-row__head';
        const label = document.createElement('span');
        label.className = 'pc-row__label';
        label.textContent = spec.label;
        const status = document.createElement('span');
        status.className = 'pc-row__status';
        head.append(label, status);

        const ctrl = document.createElement('div');
        ctrl.className = 'pc-row__ctrl';
        const slider = document.createElement('input');
        slider.type = 'range';
        slider.className = 'pc-row__slider';
        Object.assign(slider, { min: spec.min, max: spec.max, step: spec.step, value: spec.min });
        slider.disabled = true;

        const input = document.createElement('input');
        input.type = 'number';
        input.className = 'm3-text-input pc-row__number';
        Object.assign(input, { min: spec.min, max: spec.max, step: spec.step });
        input.disabled = true;

        const unit = document.createElement('span');
        unit.className = 'pc-row__unit';
        unit.textContent = spec.unit;
        ctrl.append(slider, input, unit);

        const help = document.createElement('div');
        help.className = 'pc-row__help';
        help.textContent = `${spec.help} / 範囲 ${spec.min}–${spec.max}`;

        row.append(head, ctrl, help);

        slider.addEventListener('input', () => {
            input.value = slider.value;
            this._onEdit(spec.name, Number(slider.value));
        });
        input.addEventListener('input', () => {
            const v = Number(input.value);
            if (Number.isFinite(v)) slider.value = String(v);
            this._onEdit(spec.name, v);
        });

        this._rows.set(spec.name, { spec, slider, input, status, help });
        return row;
    }

    // ===== 送受信 =====

    requestConfig() {
        this._setBanner('neutral', '読み込み中…');
        this._send({ type: 'get_position_control_config' });
    }

    _send(payload) {
        const ws = this._viewer.websocket;
        if (ws?.readyState !== WebSocket.OPEN) {
            this._setBanner('error', 'WebSocket が未接続です');
            return false;
        }
        ws.send(JSON.stringify(payload));
        return true;
    }

    _onEdit(name, value) {
        const row = this._rows.get(name);
        if (!row) return;
        const { spec } = row;
        if (!Number.isFinite(value) || value < spec.min || value > spec.max) {
            this._setStatus(name, 'error', `範囲外 (${spec.min}–${spec.max})`);
            clearTimeout(this._timers.get(name));
            return;
        }
        this._setStatus(name, 'pending', '未適用');
        clearTimeout(this._timers.get(name));
        this._timers.set(name, setTimeout(() => {
            this._setStatus(name, 'pending', '送信中…');
            if (this._send({ type: 'set_position_control_param', name, value })) {
                this._viewer.logPanel?.appendLog('info', 'PARAM', `${name} = ${value}`);
            }
        }, APPLY_DEBOUNCE_MS));
    }

    // websocket_server の position_control_config を受ける
    handleConfig(data) {
        if (!data.ready) {
            this._setBanner('error', data.message ?? 'ibis_sender が応答していません');
            for (const [name, row] of this._rows) {
                row.slider.disabled = true;
                row.input.disabled = true;
                this._setStatus(name, 'error', '—');
            }
            return;
        }

        const values = data.values ?? {};

        for (const [name, row] of this._rows) {
            // 範囲はサーバ（＝CM4 の受理範囲）を正とする。UI 側の定数とずれていたら合わせる。
            const limit = data.limits?.[name];
            if (limit) {
                row.spec.min = limit.min;
                row.spec.max = limit.max;
                Object.assign(row.slider, { min: limit.min, max: limit.max });
                Object.assign(row.input, { min: limit.min, max: limit.max });
                row.help.textContent = `${row.spec.help} / 範囲 ${limit.min}–${limit.max}`;
            }
            const v = values[name];
            if (typeof v === 'number') {
                row.slider.value = String(v);
                row.input.value = String(v);
                row.slider.disabled = false;
                row.input.disabled = false;
                this._setStatus(name, 'ok', '適用済み');
            } else {
                this._setStatus(name, 'error', '取得失敗');
            }
        }

        this._setBanner('ok',
            '1 秒ごとに CM4 へ設定パケット（UDP 12350）を送信中。5 項目とも実機のゲインに効きます。');
    }

    // websocket_server の set_position_control_param_result を受ける
    handleSetResult(data) {
        const name = data.name;
        if (!this._rows.has(name)) return;
        if (data.success) {
            // ロボットへ届くのは ibis_sender の次の送信。その間は「反映待ち」を出す
            this._setStatus(name, 'ok', '送信待ち…');
            setTimeout(() => this._setStatus(name, 'ok', '適用済み'), ROBOT_APPLY_HINT_MS);
        } else {
            this._setStatus(name, 'error', data.message ?? '失敗');
            this._viewer.logPanel?.appendLog('error', 'PARAM', `${name}: ${data.message ?? '失敗'}`);
        }
    }

    // ===== 表示 =====

    _setStatus(name, kind, text) {
        const row = this._rows.get(name);
        if (!row) return;
        row.status.textContent = text;
        row.status.dataset.kind = kind;
    }

    _setBanner(kind, text) {
        this._banner.dataset.kind = kind;
        this._banner.textContent = text;
    }
}
