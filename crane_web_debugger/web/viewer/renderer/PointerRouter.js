// canvas の入力を 1 箇所で捌く。
//
// canvas は 1 枚のままにしている。DOM のレイヤーを重ねると座標変換が二重になり、
// clientToFieldCoords が使えなくなるため。したがって「どのモードが今マウスを
// 受け取るか」はここの優先順位がすべてになる。
//
//   test（V6）> ボール配置 > sim 編集 > 移動モード > 通常（パン・選択）
//
// 優先順位を別の場所にも書かないこと。

const DRAG_THRESHOLD_PX = 5;
const ZOOM_MIN = 0.1;
const ZOOM_MAX = 5.0;
const ZOOM_STEP = 1.1;

export class PointerRouter {
    constructor(viewer, canvas) {
        this._v = viewer;
        this._canvas = canvas;
        this._dragStart = { x: 0, y: 0 };
        this._dragMoved = false;
        this._attach();
    }

    _attach() {
        const canvas = this._canvas;
        canvas.addEventListener('wheel', (e) => this._onWheel(e), { passive: false });
        canvas.addEventListener('mousedown', (e) => this._onMouseDown(e));
        canvas.addEventListener('mousemove', (e) => this._onMouseMove(e));
        canvas.addEventListener('mouseup', (e) => this._onMouseUp(e));
        canvas.addEventListener('dblclick', (e) => this._onDblClick(e));
        canvas.addEventListener('mouseleave', () => this._onMouseLeave());
        // テスト中は右クリックで目標を消す。ブラウザのメニューは出さない
        canvas.addEventListener('contextmenu', (e) => this._onContextMenu(e));
        canvas.style.cursor = 'grab';
    }

    _onWheel(e) {
        e.preventDefault();
        const v = this._v;
        const factor = e.deltaY < 0 ? ZOOM_STEP : 1 / ZOOM_STEP;
        // zoom-to-cursor: カーソル下のフィールド座標を固定点として拡縮する
        const fp = v.clientToFieldCoords(e.clientX, e.clientY);
        const newZoom = Math.min(Math.max(v.zoomLevel * factor, ZOOM_MIN), ZOOM_MAX);
        const ratio = newZoom / v.zoomLevel;
        const svgX = fp.x * 1000;
        const svgY = -fp.y * 1000;
        v.panOffset.x = svgX + (v.panOffset.x - svgX) * ratio;
        v.panOffset.y = svgY + (v.panOffset.y - svgY) * ratio;
        v.zoomLevel = newZoom;
        v.renderer?.invalidate();
    }

    _onMouseDown(e) {
        if (e.button !== 0) return;
        const v = this._v;
        this._dragStart = { x: e.clientX, y: e.clientY };
        this._dragMoved = false;

        // テストレイヤーが最優先。ここで return するので、下のレイヤーへは透過しない
        if (v.modes.test) {
            this._testDragKind = e.shiftKey ? 'velocity' : 'theta';
            this._testDragBase = { ...v.testSession };
            return;
        }
        if (e.ctrlKey) return;   // Ctrl+クリックは mouseup で複数選択として扱う

        if (e.shiftKey && v.modes.move) {
            const fp = v.clientToFieldCoords(e.clientX, e.clientY);
            const hitId = v.state.findRobotAtPosition(fp.x, fp.y);
            if (hitId !== null) {
                v.showRobotDetail(hitId);
            } else if (v.state.focusedRobotId !== null) {
                v.sendMoveCommand(v.state.focusedRobotId, fp.x, fp.y);
            }
            return;
        }

        v.isPanning = true;
        v.lastPanPoint = { x: e.clientX, y: e.clientY };
        if (!v.modes.simEdit && !v.modes.ballPlacement) this._canvas.style.cursor = 'grabbing';
    }

    _onMouseMove(e) {
        const v = this._v;
        const dx = e.clientX - this._dragStart.x;
        const dy = e.clientY - this._dragStart.y;
        if (Math.hypot(dx, dy) > DRAG_THRESHOLD_PX) this._dragMoved = true;

        v.lastMouseField = v.clientToFieldCoords(e.clientX, e.clientY);

        if (v.modes.test) {
            this._onTestMove(e);
            return;
        }
        v.updateHover(v.lastMouseField.x, v.lastMouseField.y);

        if (!v.isPanning) return;
        if (v.renderer) {
            const d = v.renderer.pixelDeltaToSvgDelta(
                e.clientX - v.lastPanPoint.x,
                e.clientY - v.lastPanPoint.y,
            );
            v.panOffset.x += d.dx;
            v.panOffset.y += d.dy;
        }
        v.lastPanPoint = { x: e.clientX, y: e.clientY };
        v.renderer?.invalidate();
    }

    _onTestMove(e) {
        const v = this._v;
        const s = v.testSession;
        if (!s) return;
        if (s.cursorFollow) {
            s.patch({ targetPos: { x: v.lastMouseField.x, y: v.lastMouseField.y } });
            v.sendTestTarget();
            return;
        }
        if (!this._testDragKind || !this._dragMoved) return;
        if (this._testDragKind === 'velocity') {
            // Shift+ドラッグ: 横移動量を速度上限に写す（スライダと双方向に同期する）
            const dx = e.clientX - this._dragStart.x;
            const next = Math.min(6, Math.max(0, (this._testDragBase.maxVelocity ?? 2) + dx / 60));
            s.patch({ maxVelocity: Math.round(next * 10) / 10 });
            return;
        }
        // ドラッグ: 目標位置からの向きを目標姿勢にする
        if (!s.targetPos) return;
        const theta = Math.atan2(v.lastMouseField.y - s.targetPos.y, v.lastMouseField.x - s.targetPos.x);
        s.patch({ targetTheta: theta });
    }

    _onContextMenu(e) {
        if (!this._v.modes.test) return;
        e.preventDefault();
        this._v.testSession?.patch({ targetPos: null });
    }

    _onMouseUp(e) {
        const v = this._v;
        if (v.modes.test) {
            const kind = this._testDragKind;
            this._testDragKind = null;
            const fp = v.clientToFieldCoords(e.clientX, e.clientY);
            if (!this._dragMoved && kind !== 'velocity') {
                v.testSession?.patch({ targetPos: { x: fp.x, y: fp.y } });
            }
            v.sendTestTarget();
            return;
        }
        v.isPanning = false;
        if (!v.modes.simEdit && !v.modes.ballPlacement) this._canvas.style.cursor = 'grab';
        if (this._dragMoved) return;

        const fp = v.clientToFieldCoords(e.clientX, e.clientY);

        if (e.ctrlKey) {
            const hitId = v.state.findRobotAtPosition(fp.x, fp.y);
            if (hitId !== null) {
                const sel = v.state.multiSelect;
                if (sel.has(hitId)) sel.delete(hitId);
                else sel.add(hitId);
                v.renderer?.invalidate();
            }
            return;
        }
        if (v.modes.ballPlacement) {
            const team = v.modes.ballPlacement;
            v.modes.exitBallPlacement();
            v.gcClient.setBallPlacementPos(fp.x, fp.y);
            v.gcClient.newCommand('BALL_PLACEMENT', team);
            return;
        }
        if (v.modes.simEdit) {
            v.simSelectAt(fp.x, fp.y);
            return;
        }
        if (!e.shiftKey && !v.modes.move) {
            // 通常クリック: ロボットで Detail トグル、空所で閉じる
            const hitId = v.state.findRobotAtPosition(fp.x, fp.y);
            if (hitId !== null) v.toggleRobotDetail(hitId);
            else v.closeRobotDetail();
        }
    }

    _onDblClick(e) {
        const v = this._v;
        if (!v.modes.simEdit || !v.modes.simSelectedObj) return;
        if (this._dragMoved) return;
        const fp = v.clientToFieldCoords(e.clientX, e.clientY);
        v.simTeleportTo(fp.x, fp.y);
    }

    _onMouseLeave() {
        const v = this._v;
        v.isPanning = false;
        if (!v.modes.simEdit && !v.modes.ballPlacement) this._canvas.style.cursor = 'default';
        v.state.hoveredRobotId = null;
        v.updateTooltip(null);
        v.renderer?.invalidate();
    }
}
