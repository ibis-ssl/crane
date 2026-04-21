// M3 デザイントークンを CSS custom property から Canvas 用に読み取るクラス。
// prefers-color-scheme 変化と html[class] 変化を監視して自動更新する。
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
        const r = k => cs.getPropertyValue(k).trim();
        this._tokens = {
            fieldTurf:       r('--md-sys-color-field-turf')        || '#0E3D10',
            fieldGrid:       r('--md-sys-color-field-grid')         || '#1D5A22',
            hudText:         r('--md-sys-color-on-surface')         || '#E2E4E8',
            hudTextMuted:    r('--md-sys-color-on-surface-variant') || '#8C929A',
            hudAccent:       r('--md-sys-color-primary')            || '#A0C4FF',
            selectionHalo:   r('--md-sys-color-primary')            || '#A0C4FF',
            moveOverlay:     r('--md-sys-color-tertiary')           || '#D0BCFF',
            simOverlayBall:  r('--md-sys-color-warning')            || '#FFA726',
            simOverlayRobot: r('--md-sys-color-tertiary')           || '#D0BCFF',
            noDataText:      r('--md-sys-color-on-surface-variant') || '#8C929A',
            bgDim:           r('--md-sys-color-surface-dim')        || '#0D1117',
        };
        this._onChange?.(this._tokens);
    }
}
