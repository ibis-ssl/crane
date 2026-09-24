// 8091 への唯一の WebSocket。
//
// タブやパネルが個別に new WebSocket すると、タブを往復するたびに購読が増えて
// 同じメッセージが何度も処理される。ソケットをここ 1 本に固定し、購読者には
// subscribe(type, fn) -> unsubscribe だけを渡すことで、その事故を構造的に防ぐ。

const RECONNECT_DELAY_MS = 3000;

export class WsHub {
    constructor(url) {
        this._url = url;
        this._subs = new Map();      // type -> Set<fn>
        this._statusSubs = new Set();
        this._logSubs = new Set();
        this.socket = null;
    }

    connect() {
        this.socket = new WebSocket(this._url);
        this.socket.onopen = () => {
            this._emitStatus(true);
            this._log('info', 'WS', `接続 ${this._url}`);
        };
        this.socket.onmessage = (event) => {
            let data;
            try {
                data = JSON.parse(event.data);
            } catch (e) {
                console.error('メッセージ解析エラー:', e);
                this._log('error', 'WS', `メッセージ解析エラー: ${e.message}`);
                return;
            }
            // 1 人の購読者が投げた例外で他の購読者を巻き添えにしない
            for (const fn of this._subs.get(data.type) ?? []) {
                try {
                    fn(data);
                } catch (e) {
                    console.error(`購読者エラー (${data.type}):`, e);
                }
            }
        };
        this.socket.onclose = () => {
            this._emitStatus(false);
            this._log('warn', 'WS', `切断 — ${RECONNECT_DELAY_MS / 1000}秒後に再接続`);
            setTimeout(() => {
                if (!this.socket || this.socket.readyState === WebSocket.CLOSED) this.connect();
            }, RECONNECT_DELAY_MS);
        };
        this.socket.onerror = () => {
            this._emitStatus(false);
            this._log('error', 'WS', '接続エラー');
        };
    }

    subscribe(type, fn) {
        if (!this._subs.has(type)) this._subs.set(type, new Set());
        this._subs.get(type).add(fn);
        return () => this._subs.get(type)?.delete(fn);
    }

    onStatus(fn) {
        this._statusSubs.add(fn);
        return () => this._statusSubs.delete(fn);
    }

    onLog(fn) {
        this._logSubs.add(fn);
        return () => this._logSubs.delete(fn);
    }

    get isOpen() { return this.socket?.readyState === WebSocket.OPEN; }

    send(payload) {
        if (!this.isOpen) return false;
        this.socket.send(JSON.stringify(payload));
        return true;
    }

    _emitStatus(connected) {
        for (const fn of this._statusSubs) fn(connected);
    }

    _log(level, tag, message) {
        for (const fn of this._logSubs) fn(level, tag, message);
    }
}
