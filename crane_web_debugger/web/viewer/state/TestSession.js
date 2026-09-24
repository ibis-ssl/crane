// テストモードで共有する状態。
//
// TestTab (UI) / TestLayer (描画) / PointerRouter (入力) の 3 者がこの 1 つを
// 共有する。分けると即座に二重管理になる（スライダを動かしたのに破線円が
// 動かない、という形で必ず出る）。
//
// フォーカスが変わったら破棄して作り直す。前のロボットの目標が残ると、
// Activate した瞬間に別の機体へ古い目標を投げることになる。
//
// 【重要】テストは「ロボット単位」ではなく「システム全体のモード」。
// #6 を選んでいても、有効化するのは crane 全体のテストモードで、
// 解除すると HALT が注入されてチーム全体が止まる。

export const ROUTE_PLANNER = 'planner';
export const ROUTE_DIRECT = 'direct';

export const LIMITS = {
    maxVelocity: { min: 0, max: 6, step: 0.1, def: 2.0, unit: 'm/s' },
    maxAcceleration: { min: 0, max: 8, step: 0.1, def: 2.5, unit: 'm/s²' },
    dampingGain: { min: 0, max: 2, step: 0.05, def: 0.5, unit: '' },
};

export class TestSession {
    constructor(robotId) {
        this.robotId = robotId;
        this.targetPos = null;          // { x, y } フィールド座標 [m]
        this.targetTheta = 0;           // [rad]
        this.maxVelocity = LIMITS.maxVelocity.def;
        this.maxAcceleration = LIMITS.maxAcceleration.def;
        this.dampingGain = LIMITS.dampingGain.def;
        this.cursorFollow = false;
        this.route = ROUTE_PLANNER;
        this._subs = new Set();
    }

    onChange(fn) {
        this._subs.add(fn);
        return () => this._subs.delete(fn);
    }

    // 変更は必ずここを通す。UI と描画が同じ通知で動くようにするため
    patch(props) {
        Object.assign(this, props);
        for (const fn of this._subs) fn(this);
    }

    // 「直接」は /control_targets へ素の速度指令を投げるだけの経路なので、
    // 速度上限とダンピングゲインは乗らない
    get limitsEffective() { return this.route === ROUTE_PLANNER; }
}
