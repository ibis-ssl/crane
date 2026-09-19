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
            fieldTurf:    r('--crane-field-turf', '#12291B'),
            fieldGrid:    r('--crane-field-grid', '#1B3A26'),
            // テキスト
            ink:          r('--crane-field-ink', '#FFFFFF'),
            inkMuted:     r('--crane-field-ink-muted', 'rgba(255, 255, 255, 0.62)'),
            // ロボット HUD
            hudAccent:    r('--crane-field-hud-accent', '#A0C4FF'),
            select:       r('--crane-field-select', '#7FE3FF'),
            // オーバーレイ
            overlayMove:  r('--crane-field-overlay-move', '#3DD68C'),
            overlayBall:  r('--crane-field-overlay-ball', '#FF7A1A'),
            overlayRobot: r('--crane-field-overlay-robot', '#7FE3FF'),
            // 状態色（バッジ・レイテンシ）
            danger:       r('--crane-field-danger', '#FF8A7A'),
            onDanger:     r('--crane-field-on-danger', '#2A0806'),
            warn:         r('--crane-field-warn', '#FFC44D'),
            onWarn:       r('--crane-field-on-warn', '#2A2000'),
            crit:         r('--crane-field-crit', '#C9A7FF'),
            onCrit:       r('--crane-field-on-crit', '#2A0A3A'),
            ok:           r('--crane-field-ok', 'rgba(255, 255, 255, 0.70)'),
        };
        this._onChange?.(this._tokens);
    }
}
