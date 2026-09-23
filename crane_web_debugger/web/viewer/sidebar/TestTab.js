// フォーカスサイドバーの「テスト」タブ。
//
// 旧「ロボット移動モード」はここに「指令経路: プランナ経由 / 直接」として同居する。
// 2 系統は有効化も指令の宛先も別物なので、切り替えると何が変わるかをラベルで示す。
//
//   プランナ経由 : activate_robot_test  → セッション注入 ROBOT_TEST
//                  robot_test_target    → /robot_test/target（速度上限を毎回同梱）
//   直接         : activate_move_mode   → セッション注入 HALT
//                  move_robot           → /control_targets（プランナを迂回）
//
// Deactivate は HALT を注入してチーム全体が止まる。だから danger 系で出す。

import { TestSession, ROUTE_PLANNER, ROUTE_DIRECT, LIMITS } from '../state/TestSession.js';

export class TestTab {
    constructor(viewer) {
        this._v = viewer;
        this._root = null;
        this._id = null;
    }

    get label() { return 'テスト'; }

    mount(container) {
        this._root = document.createElement('div');
        this._root.className = 'rd-test';
        container.appendChild(this._root);
    }

    activate(id) {
        this._id = id;
        // フォーカスが変わったらセッションを作り直す。前のロボットの目標を
        // 引き継ぐと、Activate した瞬間に別の機体へ古い目標を投げてしまう
        if (!this._v.testSession || this._v.testSession.robotId !== id) {
            this._v.setTestSession(new TestSession(id));
        }
        this._render();
        this._unsub = this._v.testSession.onChange(() => this._syncValues());
    }

    deactivate() {
        this._unsub?.();
        this._unsub = null;
        this._root = null;
    }

    refresh() { this._syncValues(); }

    get _s() { return this._v.testSession; }

    _render() {
        const s = this._s;
        const active = this._v.modes.test;
        this._root.innerHTML = `
            <div class="rd-test__banner" data-active="${active}">
                ${active
                    ? `テストモード有効 — 操作対象 #${this._id}。フィールドのクリックが目標になります`
                    : 'テストモードは無効です。Activate で crane 全体をテストモードにします'}
            </div>

            <div class="rd-section-title">指令経路</div>
            <label class="rd-radio">
                <input type="radio" name="test-route" value="${ROUTE_PLANNER}"
                    ${s.route === ROUTE_PLANNER ? 'checked' : ''}>
                <span><b>プランナ経由</b><br>
                <span class="rd-hint">/robot_test/target に投げ、local_planner が経路を作る。
                セッション注入は ROBOT_TEST</span></span>
            </label>
            <label class="rd-radio">
                <input type="radio" name="test-route" value="${ROUTE_DIRECT}"
                    ${s.route === ROUTE_DIRECT ? 'checked' : ''}>
                <span><b>直接</b><br>
                <span class="rd-hint">/control_targets へ直接投げてプランナを迂回する。
                速度上限は効かない。セッション注入は HALT</span></span>
            </label>

            <div class="rd-section-title">Speed Limits</div>
            <div class="rd-test__limits" data-effective="${s.limitsEffective}">
                ${this._sliderHtml('maxVelocity', 'Max Vel')}
                ${this._sliderHtml('maxAcceleration', 'Max Acc')}
                ${this._sliderHtml('dampingGain', 'Damping')}
                <div class="rd-hint">上限は送信のたびに同梱される値です。
                動かしただけでは何も起きません（次の目標送信から効きます）。</div>
            </div>

            <div class="rd-section-title">目標</div>
            <div class="rd-test__target" id="test-target-readout">--</div>
            <label class="rd-check">
                <input type="checkbox" id="test-cursor-follow" ${s.cursorFollow ? 'checked' : ''}>
                カーソル追従（マウス位置をそのまま目標にする）
            </label>
            <div class="rd-hint">クリック = 目標 / ドラッグ = 姿勢 /
            Shift+ドラッグ = 速度上限 / 右クリック = 目標消去 / Esc = 解除</div>

            <div class="rd-test__actions">
                <button type="button" class="m3-btn m3-btn--sm m3-btn--filled" id="test-activate"
                    ${active ? 'disabled' : ''}>Activate</button>
                <button type="button" class="m3-btn m3-btn--sm gc-btn--halt" id="test-deactivate"
                    ${active ? '' : 'disabled'}>Deactivate (HALT)</button>
            </div>
        `;
        this._bind();
        this._syncValues();
    }

    _sliderHtml(key, label) {
        const lim = LIMITS[key];
        return `
            <div class="rd-test__row">
                <label>${label}</label>
                <input type="range" class="pc-row__slider" data-limit="${key}"
                    min="${lim.min}" max="${lim.max}" step="${lim.step}" value="${this._s[key]}">
                <span class="rd-test__val" data-limit-val="${key}">${this._s[key]}</span>
                <span class="rd-test__unit">${lim.unit}</span>
            </div>`;
    }

    _bind() {
        for (const r of this._root.querySelectorAll('input[name="test-route"]')) {
            r.addEventListener('change', () => {
                this._s.patch({ route: r.value });
                this._render();
            });
        }
        for (const sl of this._root.querySelectorAll('[data-limit]')) {
            sl.addEventListener('input', () => {
                this._s.patch({ [sl.dataset.limit]: parseFloat(sl.value) });
                if (sl.dataset.limit === 'dampingGain') this._v.sendPlannerDamping();
            });
        }
        this._root.querySelector('#test-cursor-follow')?.addEventListener('change', (e) => {
            this._s.patch({ cursorFollow: e.target.checked });
        });
        this._root.querySelector('#test-activate')?.addEventListener('click', () => {
            this._v.activateTest();
            this._render();
        });
        this._root.querySelector('#test-deactivate')?.addEventListener('click', () => {
            this._v.deactivateTest();
            this._render();
        });
    }

    _syncValues() {
        if (!this._root) return;
        const s = this._s;
        for (const el of this._root.querySelectorAll('[data-limit-val]')) {
            el.textContent = s[el.dataset.limitVal];
        }
        for (const sl of this._root.querySelectorAll('[data-limit]')) {
            sl.value = s[sl.dataset.limit];
        }
        const limits = this._root.querySelector('.rd-test__limits');
        if (limits) limits.dataset.effective = String(s.limitsEffective);
        const readout = this._root.querySelector('#test-target-readout');
        if (readout) {
            readout.textContent = s.targetPos
                ? `(${s.targetPos.x.toFixed(3)}, ${s.targetPos.y.toFixed(3)}) m  θ=${s.targetTheta.toFixed(3)} rad`
                : '未設定 — フィールドをクリックしてください';
        }
    }
}
