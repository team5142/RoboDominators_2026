// NT4 Client - RoboDominators 5142
// Implements WPILib NT4 over WebSocket with MessagePack binary frames.
// Prefix subscriptions only (no per-topic subscribing needed from the dashboard).

// Minimal MessagePack decoder - handles the types NT4 actually sends.
const MsgPack = (() => {
    function decode(buf) {
        const ab = buf instanceof ArrayBuffer ? buf : buf.buffer;
        const off = buf.byteOffset || 0;
        const view = new DataView(ab);
        const [val] = decodeAt(view, off);
        return val;
    }

    function decodeAt(view, off) {
        const b = view.getUint8(off);
        if (b <= 0x7f) return [b, off + 1];
        if ((b & 0xf0) === 0x80) return decodeMap(view, off + 1, b & 0x0f);
        if ((b & 0xf0) === 0x90) return decodeArray(view, off + 1, b & 0x0f);
        if ((b & 0xe0) === 0xa0) return decodeStr(view, off + 1, b & 0x1f);
        if ((b & 0xe0) === 0xe0) return [b - 256, off + 1];
        switch (b) {
            case 0xc0: return [null, off + 1];
            case 0xc2: return [false, off + 1];
            case 0xc3: return [true, off + 1];
            case 0xc4: { const n = view.getUint8(off+1); return [new Uint8Array(view.buffer, view.byteOffset+off+2, n), off+2+n]; }
            case 0xca: return [view.getFloat32(off+1, false), off+5];
            case 0xcb: return [view.getFloat64(off+1, false), off+9];
            case 0xcc: return [view.getUint8(off+1), off+2];
            case 0xcd: return [view.getUint16(off+1, false), off+3];
            case 0xce: return [view.getUint32(off+1, false), off+5];
            case 0xcf: return [Number(view.getBigUint64(off+1, false)), off+9];
            case 0xd0: return [view.getInt8(off+1), off+2];
            case 0xd1: return [view.getInt16(off+1, false), off+3];
            case 0xd2: return [view.getInt32(off+1, false), off+5];
            case 0xd3: return [Number(view.getBigInt64(off+1, false)), off+9];
            case 0xd9: { const n = view.getUint8(off+1); return decodeStr(view, off+2, n); }
            case 0xda: { const n = view.getUint16(off+1, false); return decodeStr(view, off+3, n); }
            case 0xdb: { const n = view.getUint32(off+1, false); return decodeStr(view, off+5, n); }
            case 0xdc: { const n = view.getUint16(off+1, false); return decodeArray(view, off+3, n); }
            case 0xdd: { const n = view.getUint32(off+1, false); return decodeArray(view, off+5, n); }
            case 0xde: { const n = view.getUint16(off+1, false); return decodeMap(view, off+3, n); }
            case 0xdf: { const n = view.getUint32(off+1, false); return decodeMap(view, off+5, n); }
            default: throw new Error('MsgPack: unknown byte 0x' + b.toString(16) + ' at ' + off);
        }
    }

    function decodeStr(view, off, len) {
        return [new TextDecoder().decode(new Uint8Array(view.buffer, view.byteOffset+off, len)), off+len];
    }

    function decodeArray(view, off, count) {
        const arr = [];
        for (let i = 0; i < count; i++) { const [v, o] = decodeAt(view, off); arr.push(v); off = o; }
        return [arr, off];
    }

    function decodeMap(view, off, count) {
        const obj = {};
        for (let i = 0; i < count; i++) {
            const [k, o1] = decodeAt(view, off);
            const [v, o2] = decodeAt(view, o1);
            obj[k] = v; off = o2;
        }
        return [obj, off];
    }

    return { decode };
})();

class NT4Client {
    constructor(serverAddr, appName = 'FRC_Dashboard') {
        this.serverAddr = serverAddr;
        this.appName = appName;
        this.ws = null;
        this.connected = false;
        this.reconnectTimer = null;
        this.subUid = 0;
        this.topicsById = new Map();   // id -> { name, type }
        this.values = new Map();       // name -> last value
        this.prefixes = [];
        this.onConnect = null;
        this.onDisconnect = null;
        this.onChange = null;
    }

    connect() {
        if (this.ws) { try { this.ws.close(); } catch (_) {} }
        const url = `ws://${this.serverAddr}/nt/${this.appName}`;
        console.log('[NT4] Connecting to', url);
        
        this.ws = new WebSocket(url, ['networktables.first.wpi.edu']);
        this.ws.binaryType = 'arraybuffer';

        this.ws.onopen = () => {
            console.log('[NT4] Connected');
            this.connected = true;
            clearInterval(this.reconnectTimer);
            this.reconnectTimer = null;
            this.topicsById.clear();
            if (this.prefixes.length > 0) this._sendSubscriptions();
            if (this.onConnect) this.onConnect();
        };

        this.ws.onmessage = (evt) => {
            if (typeof evt.data === 'string') {
                this._handleJson(evt.data);
            } else {
                this._handleBinary(evt.data);
            }
        };

        this.ws.onerror = (e) => console.warn('[NT4] WS error', e);

        this.ws.onclose = () => {
            console.log('[NT4] Disconnected');
            this.connected = false;
            this.topicsById.clear();
            if (this.onDisconnect) this.onDisconnect();
            if (!this.reconnectTimer) {
                this.reconnectTimer = setInterval(() => this.connect(), 2000);
            }
        };
    }

    // Call with the same wildcard list dashboard.js already uses - strips trailing /*
    subscribe(prefixList) {
        this.prefixes = prefixList.map(p => p.endsWith('/*') ? p.slice(0, -2) : p);
        if (this.connected) this._sendSubscriptions();
    }

    getValue(topic, defaultValue = null) {
        return this.values.has(topic) ? this.values.get(topic) : defaultValue;
    }

    _sendSubscriptions() {
        const msgs = [];
        this.prefixes.forEach(prefix => {
            msgs.push({
                method: 'subscribe',
                params: {
                    topics: [prefix],
                    subuid: ++this.subUid,
                    options: { prefix: true, periodic: 0.1, all: false, sendAll: true, immediate: true }
                }
            });
        });
        this._sendJson(msgs);
        console.log('[NT4] Subscribed to prefixes:', this.prefixes);
    }

    _handleJson(raw) {
        let msgs;
        try { msgs = JSON.parse(raw); } catch (e) { return; }
        if (!Array.isArray(msgs)) msgs = [msgs];
        msgs.forEach(msg => {
            if (msg.method === 'announce') {
                const { name, id, type } = msg.params;
                this.topicsById.set(id, { name, type });
            } else if (msg.method === 'unannounce') {
                this.topicsById.delete(msg.params.id);
            }
        });
    }

    _handleBinary(buf) {
        // NT4 binary frames are MessagePack arrays: [topicId, timestamp_us, typeId, value]
        let frame;
        try {
            frame = MsgPack.decode(new DataView(buf));
        } catch (e) {
            console.warn('[NT4] MsgPack decode error:', e);
            return;
        }
        if (!Array.isArray(frame) || frame.length < 4) return;

        const [topicId, , , value] = frame;
        if (topicId < 0) return; // timestamp sync

        const topic = this.topicsById.get(topicId);
        if (!topic) return;

        this.values.set(topic.name, value);
        if (this.onChange) this.onChange(topic.name, value);
    }

    _handleJson(raw) {
        let msgs;
        try { msgs = JSON.parse(raw); } catch (e) { return; }
        if (!Array.isArray(msgs)) msgs = [msgs];
        msgs.forEach(msg => {
            if (msg.method === 'announce') {
                const { name, id, type } = msg.params;
                this.topicsById.set(id, { name, type });
            } else if (msg.method === 'unannounce') {
                this.topicsById.delete(msg.params.id);
            }
        });
    }

    _handleBinary(buf) {
        // NT4 binary frames are MessagePack arrays: [topicId, timestamp_us, typeId, value]
        let frame;
        try {
            frame = MsgPack.decode(new DataView(buf));
        } catch (e) {
            console.warn('[NT4] MsgPack decode error:', e);
            return;
        }
        if (!Array.isArray(frame) || frame.length < 4) return;

        const [topicId, , , value] = frame;
        if (topicId < 0) return; // timestamp sync

        const topic = this.topicsById.get(topicId);
        if (!topic) return;

        this.values.set(topic.name, value);
        if (this.onChange) this.onChange(topic.name, value);
    }

    _sendJson(data) {
        if (this.ws && this.ws.readyState === WebSocket.OPEN) {
            this.ws.send(JSON.stringify(data));
        }
    }

    disconnect() {
        clearInterval(this.reconnectTimer);
        this.reconnectTimer = null;
        if (this.ws) { this.ws.close(); this.ws = null; }
    }
}

if (typeof module !== 'undefined' && module.exports) {
    module.exports = NT4Client;
}
