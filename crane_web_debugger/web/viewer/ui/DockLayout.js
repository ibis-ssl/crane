const STORAGE_KEY = 'cv.dockLayout.v1';

const DEFAULT_LAYOUT = {
    v: 1,
    sizes: { left: 300, right: 260, bottom: 0 },
    collapsed: { left: false, right: false, bottom: true },
};

export class DockLayout {
    constructor() {
        this._layout = this._load();
        this._applyToCSS();
        this._setupSplitters();
        this._setupToggleButtons();
    }

    _load() {
        try {
            const s = localStorage.getItem(STORAGE_KEY);
            if (s) {
                const p = JSON.parse(s);
                if (p.v === 1) return {
                    ...DEFAULT_LAYOUT,
                    sizes: { ...DEFAULT_LAYOUT.sizes, ...p.sizes },
                    collapsed: { ...DEFAULT_LAYOUT.collapsed, ...p.collapsed },
                };
            }
        } catch { /* ignore parse errors */ }
        return {
            ...DEFAULT_LAYOUT,
            sizes: { ...DEFAULT_LAYOUT.sizes },
            collapsed: { ...DEFAULT_LAYOUT.collapsed },
        };
    }

    _save() {
        try { localStorage.setItem(STORAGE_KEY, JSON.stringify(this._layout)); } catch { /* ignore */ }
    }

    _applyToCSS() {
        const rs = document.documentElement.style;
        const { sizes, collapsed } = this._layout;
        rs.setProperty('--dock-left-w', (collapsed.left ? 32 : sizes.left) + 'px');
        rs.setProperty('--dock-right-w', (collapsed.right ? 32 : sizes.right) + 'px');
        rs.setProperty('--dock-bottom-h', (collapsed.bottom ? 0 : sizes.bottom) + 'px');
        this._updateCollapseIcons();
    }

    _updateCollapseIcons() {
        for (const side of ['left', 'right', 'bottom']) {
            const btn = document.querySelector(`[data-dock-toggle="${side}"]`);
            if (!btn) continue;
            const icon = btn.querySelector('.material-symbols-outlined');
            if (!icon) continue;
            const isCol = this._layout.collapsed[side];
            if (side === 'left') icon.textContent = isCol ? 'chevron_right' : 'chevron_left';
            else if (side === 'right') icon.textContent = isCol ? 'chevron_left' : 'chevron_right';
            else icon.textContent = isCol ? 'expand_less' : 'expand_more';
        }
    }

    _setupSplitters() {
        document.querySelectorAll('.dock-splitter').forEach(spl => {
            const target = spl.dataset.target;
            if (!target) return;
            let startPos = 0, startSize = 0;

            spl.addEventListener('pointerdown', (e) => {
                e.preventDefault();
                spl.setPointerCapture(e.pointerId);
                startPos = e.clientX;
                startSize = this._layout.sizes[target];
                document.body.style.cursor = 'col-resize';
            });

            spl.addEventListener('pointermove', (e) => {
                if (!spl.hasPointerCapture(e.pointerId)) return;
                const delta = e.clientX - startPos;
                const sign = target === 'right' ? -1 : 1;
                const newSize = Math.max(200, Math.min(600, startSize + delta * sign));
                this._layout.sizes[target] = newSize;
                if (!this._layout.collapsed[target]) {
                    document.documentElement.style.setProperty(`--dock-${target}-w`, newSize + 'px');
                }
            });

            spl.addEventListener('pointerup', () => {
                document.body.style.cursor = '';
                this._save();
            });
        });
    }

    _setupToggleButtons() {
        document.addEventListener('click', (e) => {
            const btn = e.target.closest('[data-dock-toggle]');
            if (!btn) return;
            const target = btn.dataset.dockToggle;
            this._layout.collapsed[target] = !this._layout.collapsed[target];
            this._applyToCSS();
            this._save();
        });
    }
}
