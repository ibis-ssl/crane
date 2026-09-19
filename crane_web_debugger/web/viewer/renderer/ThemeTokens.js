// フィールド描画用のデザイントークンを CSS custom property から読み取るクラス。
// prefers-color-scheme 変化と html[class]/[data-theme] 変化を監視して自動更新する。
//
// 【不変条件】このクラスは --crane-field-* だけを読む。クローム系トークン
// (--md-sys-color-*) を参照してはいけない。フィールドは常にダークで描かれるため、
// クロームをライトにした瞬間に HUD の文字が芝へ溶けて消える。
// フォールバック値は shared/theme/m3e-theme.css の定義と必ず一致させること
// (スタイルシート未読込時でも読める描画になるように)。
export class ThemeTokens {
    constructor() {
        this._tokens = null;
        this._onChange = null;
        this._mql = matchMedia('(prefers-color-scheme: dark)');
        this._observer = new MutationObserver(() => this._refresh());
    }

    mount(onChange) {
        this._onChange = onChange;
        this._mql.addEventListener('change', () => this._refresh());
        this._observer.observe(document.documentElement, {
            attributes: true, attributeFilter: ['class', 'data-theme']
        });
        window.addEventListener('load', () => this._refresh());
        this._refresh();
    }

    get() { return this._tokens; }

    _refresh() {
        const cs = getComputedStyle(document.documentElement);
        const r = (k, fallback) => cs.getPropertyValue(k).trim() || fallback;
        this._tokens = {
            // 面
            fieldTurf:    r('--crane-field-turf', '#0E3D10'),
            fieldGrid:    r('--crane-field-grid', '#1D5A22'),
            // テキスト
            ink:          r('--crane-field-ink', '#E2E4E8'),
            inkMuted:     r('--crane-field-ink-muted', '#8C929A'),
            // ロボット HUD
            hudAccent:    r('--crane-field-hud-accent', '#A0C4FF'),
            select:       r('--crane-field-select', '#A0C4FF'),
            selectDetail: r('--crane-field-select-2', '#7D5260'),
            // オーバーレイ
            overlayMove:  r('--crane-field-overlay-move', '#D0BCFF'),
            overlayBall:  r('--crane-field-overlay-ball', '#FFA726'),
            overlayRobot: r('--crane-field-overlay-robot', '#D0BCFF'),
            // 状態色（バッジ・レイテンシ）
            danger:       r('--crane-field-danger', '#B3261E'),
            onDanger:     r('--crane-field-on-danger', '#FFFFFF'),
            warn:         r('--crane-field-warn', '#F9A825'),
            onWarn:       r('--crane-field-on-warn', '#2A2000'),
            crit:         r('--crane-field-crit', '#6650A4'),
            onCrit:       r('--crane-field-on-crit', '#FFFFFF'),
            ok:           r('--crane-field-ok', '#F4DFF0'),
        };
        this._onChange?.(this._tokens);
    }
}
