export class GameControlClient {
    constructor() {
        this.ws = null;
        this.state = null;
        this.onStateChange = null;
        this._host = null;
    }

    connect(host) {
        this._host = host;
        const url = `ws://${host}:8081/api/control`;
        try { this.ws = new WebSocket(url); } catch { this._reconnect(); return; }
        this.ws.onopen = () => this._setIndicator(true);
        this.ws.onmessage = (e) => {
            try {
                const msg = JSON.parse(e.data);
                if (msg.matchState !== undefined) {
                    this.state = msg.matchState;
                    this.onStateChange?.(this.state);
                }
            } catch { /* ignore parse errors */ }
        };
        this.ws.onclose = () => { this._setIndicator(false); this._reconnect(); };
        this.ws.onerror = () => this._setIndicator(false);
    }

    _reconnect() { setTimeout(() => this.connect(this._host), 5000); }

    _setIndicator(ok) {
        document.getElementById('gc-connection-dot')?.classList.toggle('connected', ok);
    }

    _sendChange(change) {
        if (this.ws?.readyState === WebSocket.OPEN)
            this.ws.send(JSON.stringify({ change }));
    }

    _sendInput(input) {
        if (this.ws?.readyState === WebSocket.OPEN)
            this.ws.send(JSON.stringify(input));
    }

    newCommand(type, forTeam = 'UNKNOWN') {
        this._sendChange({ newCommandChange: { command: { type, forTeam } } });
    }

    setBallPlacementPos(x, y) {
        this._sendChange({ setBallPlacementPosChange: { pos: { x, y } } });
    }

    addYellowCard(team) { this._sendChange({ addYellowCardChange: { forTeam: team } }); }
    addRedCard(team) { this._sendChange({ addRedCardChange: { forTeam: team } }); }

    updateGoals(team, delta) {
        const current = this.state?.teamState?.[team]?.goals ?? 0;
        this._sendChange({
            updateTeamStateChange: { forTeam: team, goals: Math.max(0, current + delta) }
        });
    }

    nextStage() {
        const ca = this.state?.continueActions?.[0];
        if (ca) this._sendInput({ continueAction: ca });
    }
}
