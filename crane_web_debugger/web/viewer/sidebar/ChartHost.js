// Chart.js をサイドバーのタブに載せるための薄いラッパ。
//
// Chart.js には 3 つ踏みやすい落とし穴があり、そのすべてをここで潰す。
//
//  1. display:none の親で responsive:true の Chart を作ると 0×0 に潰れ、
//     可視化しても戻らない → 生成は「可視になってから requestAnimationFrame 1 フレーム後」
//  2. タブ切替で destroy() を忘れるとリスナが残りゾンビ更新になる
//     → FocusSidebar が deactivate() を必ず呼び、そこで destroy する
//  3. 受信のたびに update() を呼ぶと重い → flush() を rAF スロットルして 1 フレーム 1 回
//
// Chart.js 本体は /assets/chart.umd.min.js に同梱済み（CDN は使わない。会場オフライン前提）。
// 読み込みはテレメトリタブを最初に開いた時だけ。Viewer の初期表示に 200KB を負わせない。

const CHART_SRC = '/assets/chart.umd.min.js';
let loadPromise = null;

function loadChartJs() {
    if (window.Chart) return Promise.resolve(window.Chart);
    if (loadPromise) return loadPromise;
    loadPromise = new Promise((resolve, reject) => {
        const el = document.createElement('script');
        el.src = CHART_SRC;
        el.onload = () => resolve(window.Chart);
        el.onerror = () => {
            loadPromise = null;
            reject(new Error(`${CHART_SRC} を読み込めませんでした`));
        };
        document.head.appendChild(el);
    });
    return loadPromise;
}

export function makeDataset(label, color, dashed = false) {
    return {
        label,
        data: [],
        borderColor: color,
        backgroundColor: `${color}22`,
        borderWidth: dashed ? 1.5 : 2,
        borderDash: dashed ? [4, 3] : [],
        pointRadius: 0,
        tension: 0.2,
    };
}

export class ChartHost {
    constructor(canvas, datasets, options = {}) {
        this._canvas = canvas;
        this._datasets = datasets;
        this._yLabel = options.yLabel ?? '';
        this._maxPoints = options.maxPoints ?? 300;
        this._chart = null;
        this._rafId = null;
        this._dirty = false;
    }

    // 親が可視になってから 1 フレーム待って生成する（落とし穴 1）
    async create() {
        const Chart = await loadChartJs();
        await new Promise(r => requestAnimationFrame(r));
        if (!this._canvas.isConnected) return null;
        const cs = getComputedStyle(document.documentElement);
        const t = (k, fb) => cs.getPropertyValue(k).trim() || fb;
        const ink = t('--crane-chart-ink', '#555C6B');
        const grid = t('--crane-chart-grid', '#E3E6EC');

        this._chart = new Chart(this._canvas, {
            type: 'line',
            data: { labels: [], datasets: this._datasets },
            options: {
                responsive: true,
                maintainAspectRatio: false,
                animation: false,
                interaction: { intersect: false, mode: 'index' },
                scales: {
                    x: {
                        type: 'linear',
                        ticks: { color: ink, maxTicksLimit: 5, font: { size: 9 } },
                        grid: { color: grid },
                    },
                    y: {
                        title: { display: Boolean(this._yLabel), text: this._yLabel, color: ink, font: { size: 9 } },
                        ticks: { color: ink, maxTicksLimit: 5, font: { size: 9 } },
                        grid: { color: grid },
                    },
                },
                plugins: {
                    legend: { labels: { color: ink, boxWidth: 10, font: { size: 9 } } },
                    tooltip: { enabled: false },
                },
            },
        });
        return this._chart;
    }

    push(datasetIndex, x, y) {
        const ds = this._chart?.data.datasets[datasetIndex];
        if (!ds || y === null || y === undefined) return;
        ds.data.push({ x, y });
        if (ds.data.length > this._maxPoints) ds.data.shift();
        this._dirty = true;
    }

    // 受信のたびに呼んでよい。実際の update は 1 フレームに 1 回に間引かれる（落とし穴 3）
    flush() {
        if (!this._chart || !this._dirty || this._rafId !== null) return;
        this._rafId = requestAnimationFrame(() => {
            this._rafId = null;
            this._dirty = false;
            this._chart?.update('none');
        });
    }

    destroy() {
        if (this._rafId !== null) cancelAnimationFrame(this._rafId);
        this._rafId = null;
        this._chart?.destroy();
        this._chart = null;
    }
}
