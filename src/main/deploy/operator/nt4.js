// NT4 client for FRC web dashboards.
// Implements NT4 v4.0/v4.1 over WebSocket with binary MessagePack value publishing.
// Based on: https://github.com/wpilibsuite/allwpilib/blob/main/ntcore/doc/networktables4.adoc

'use strict';

// Minimal MessagePack encoder/decoder - only the types needed for NT4.
// Published values are sent as 4-element arrays: [pubuid, timestamp_us, type_id, value]
var MsgPack = (function () {
  function encodeUint(val, buf) {
    if (val >= 0 && val <= 0x7f) {
      buf.push(val);
    } else if (val <= 0xff) {
      buf.push(0xcc, val);
    } else if (val <= 0xffff) {
      buf.push(0xcd, (val >> 8) & 0xff, val & 0xff);
    } else if (val <= 0xffffffff) {
      buf.push(0xce,
        (val >>> 24) & 0xff, (val >>> 16) & 0xff,
        (val >>> 8)  & 0xff,  val         & 0xff);
    } else {
      var hi = Math.floor(val / 0x100000000);
      var lo = val >>> 0;
      buf.push(0xcf,
        (hi >>> 24) & 0xff, (hi >>> 16) & 0xff,
        (hi >>> 8)  & 0xff,  hi         & 0xff,
        (lo >>> 24) & 0xff, (lo >>> 16) & 0xff,
        (lo >>> 8)  & 0xff,  lo         & 0xff);
    }
  }

  function encodeInt(val, buf) {
    if (val >= 0) {
      encodeUint(val, buf);
    } else if (val >= -32) {
      buf.push(val & 0xff);
    } else if (val >= -128) {
      buf.push(0xd0, val & 0xff);
    } else if (val >= -32768) {
      buf.push(0xd1, (val >> 8) & 0xff, val & 0xff);
    } else {
      buf.push(0xd2,
        (val >> 24) & 0xff, (val >> 16) & 0xff,
        (val >> 8)  & 0xff,  val        & 0xff);
    }
  }

  function encodeDouble(val, buf) {
    var tmp = new ArrayBuffer(8);
    new DataView(tmp).setFloat64(0, val, false); // big-endian IEEE754
    buf.push(0xcb);
    var bytes = new Uint8Array(tmp);
    for (var i = 0; i < 8; i++) buf.push(bytes[i]);
  }

  function encodeString(val, buf) {
    var encoded = unescape(encodeURIComponent(val)); // UTF-8 bytes as latin1 string
    var len = encoded.length;
    if (len <= 31) {
      buf.push(0xa0 | len);
    } else if (len <= 0xff) {
      buf.push(0xd9, len);
    } else {
      buf.push(0xda, (len >> 8) & 0xff, len & 0xff);
    }
    for (var i = 0; i < len; i++) buf.push(encoded.charCodeAt(i));
  }

  // Encode a 4-element NT4 frame for publishing: [pubuid, timestamp_us, type_id, value]
  function encodeNT4Value(pubuid, timestampUs, typeId, value) {
    var buf = [0x94]; // fixarray of 4
    encodeInt(pubuid, buf);
    encodeUint(timestampUs, buf);
    encodeUint(typeId, buf);
    if (typeId === 1)      encodeDouble(value, buf);  // double
    else if (typeId === 0) buf.push(value ? 0xc3 : 0xc2); // boolean
    else if (typeId === 2) encodeInt(value, buf);     // int
    else if (typeId === 4) encodeString(value, buf);  // string
    else                   encodeDouble(value, buf);  // fallback
    return new Uint8Array(buf);
  }

  function decodeValue(view, offset) {
    var b = view.getUint8(offset++);
    if (b <= 0x7f)       return { val: b,     next: offset };
    if (b >= 0xe0)       return { val: b-256, next: offset };
    if ((b & 0xe0) === 0xa0) { // fixstr
      var len = b & 0x1f, s = '';
      for (var i = 0; i < len; i++) s += String.fromCharCode(view.getUint8(offset + i));
      return { val: s, next: offset + len };
    }
    switch (b) {
      case 0xc2: return { val: false, next: offset };
      case 0xc3: return { val: true,  next: offset };
      case 0xca: return { val: view.getFloat32(offset, false), next: offset + 4 };
      case 0xcb: return { val: view.getFloat64(offset, false), next: offset + 8 };
      case 0xcc: return { val: view.getUint8(offset),          next: offset + 1 };
      case 0xcd: return { val: view.getUint16(offset, false),  next: offset + 2 };
      case 0xce: return { val: view.getUint32(offset, false),  next: offset + 4 };
      case 0xd0: return { val: view.getInt8(offset),           next: offset + 1 };
      case 0xd1: return { val: view.getInt16(offset, false),   next: offset + 2 };
      case 0xd2: return { val: view.getInt32(offset, false),   next: offset + 4 };
      case 0xd9: { var len = view.getUint8(offset); offset++; var s = ''; for (var i=0;i<len;i++) s+=String.fromCharCode(view.getUint8(offset+i)); return {val:s, next:offset+len}; }
      case 0xda: { var len = view.getUint16(offset,false); offset+=2; var s=''; for(var i=0;i<len;i++) s+=String.fromCharCode(view.getUint8(offset+i)); return {val:s,next:offset+len}; }
      default:   return { val: null, next: offset };
    }
  }

  // Decode one NT4 binary frame starting at offset. Returns { topicId, timestampUs, typeId, value, next }.
  function decodeNT4Message(view, offset) {
    if (offset >= view.byteLength) return null;
    var b = view.getUint8(offset);
    if ((b & 0xf0) !== 0x90 || (b & 0x0f) < 4) return null; // must be fixarray of >=4
    offset++;
    var r0 = decodeValue(view, offset); offset = r0.next;
    var r1 = decodeValue(view, offset); offset = r1.next;
    var r2 = decodeValue(view, offset); offset = r2.next;
    var r3 = decodeValue(view, offset); offset = r3.next;
    return { topicId: r0.val, timestampUs: r1.val, typeId: r2.val, value: r3.val, next: offset };
  }

  return { encodeNT4Value: encodeNT4Value, decodeNT4Message: decodeNT4Message };
})();

var NT4Types = { boolean: 0, double: 1, int: 2, float: 3, string: 4 };

class NT4Client {
    constructor(serverAddr, appName = 'FRC_Dashboard') {
        this.serverAddr = serverAddr;
        this.appName = appName;
        this.ws = null;
        this._serverTimeOffsetUs = 0;
        this._lastPingTime = 0;
        this._subscriptions = new Map(); // subuid -> { topics, options, callback }
        this._publishers = new Map();    // topicName -> { pubuid, typeId, typeName, announced }
        this._topicIds = new Map();      // topicName -> numeric server id (from announce)
        this.subUid = 0;
        this.pubUid = 0;
        this.connected = false;
        this.reconnectInterval = null;
        this.onConnect = null;
        this.onDisconnect = null;
    }

    connect() {
        if (this.ws) { try { this.ws.close(); } catch(e) {} this.ws = null; }

        var url = 'ws://' + this.serverAddr + '/nt/' + this.appName;
        console.log('[NT4] Connecting to ' + url);
        this.ws = new WebSocket(url, ['networktables.first.wpi.edu', 'v4.1.networktables.first.wpi.edu']);
        this.ws.binaryType = 'arraybuffer';

        this.ws.onopen = () => {
            console.log('[NT4] Connected');
            this.connected = true;
            if (this.reconnectInterval) { clearInterval(this.reconnectInterval); this.reconnectInterval = null; }

            // Re-announce all publishers after reconnect
            var pubs = [];
            this._publishers.forEach((pub, name) => {
                pub.announced = false;
                pubs.push({ method: 'publish', params: { name: name, pubuid: pub.pubuid, type: pub.typeName, properties: {} } });
            });
            if (pubs.length > 0) this.sendJson(pubs);

            // Re-subscribe
            var subs = [];
            this._subscriptions.forEach((sub, uid) => {
                subs.push({ method: 'subscribe', params: { topics: sub.topics, subuid: uid, options: sub.options } });
            });
            if (subs.length > 0) this.sendJson(subs);

            this._syncTime();
            if (this.onConnect) this.onConnect();
        };

        this.ws.onmessage = (event) => this._handleMessage(event.data);
        this.ws.onerror = (e) => console.warn('[NT4] WebSocket error', e);

        this.ws.onclose = () => {
            console.log('[NT4] Disconnected');
            this.connected = false;
            this._topicIds.clear();
            this._publishers.forEach(pub => { pub.announced = false; });
            if (this.onDisconnect) this.onDisconnect();
            if (!this.reconnectInterval) {
                this.reconnectInterval = setInterval(() => this.connect(), 1000);
            }
        };
    }

    // Subscribe to one or more topics. callback(topicName, timestampUs, value) called on update.
    // topics can be an array or single string; use prefix=true to match all sub-topics.
    subscribe(topics, callback, options = {}) {
        var subId = ++this.subUid;
        var topicArr = Array.isArray(topics) ? topics : [topics];
        this._subscriptions.set(subId, { topics: topicArr, options: options, callback: callback });
        if (this.connected) {
            this.sendJson([{ method: 'subscribe', params: { topics: topicArr, subuid: subId, options: options } }]);
        }
        return subId;
    }

    unsubscribe(subId) {
        this._subscriptions.delete(subId);
        if (this.connected) this.sendJson([{ method: 'unsubscribe', params: { subuid: subId } }]);
    }

    // Publish a value via binary MessagePack frame. type = 'double'|'boolean'|'string'|'int'.
    // type is inferred from the JS value type if omitted.
    putValue(topicName, value, type) {
        if (!this.connected) return;
        type = type || (typeof value === 'boolean' ? 'boolean' : typeof value === 'string' ? 'string' : 'double');
        var typeId = NT4Types[type] !== undefined ? NT4Types[type] : NT4Types.double;

        var pub = this._publishers.get(topicName);
        if (!pub) {
            pub = { pubuid: ++this.pubUid, typeId: typeId, typeName: type, announced: false };
            this._publishers.set(topicName, pub);
        }

        if (!pub.announced) {
            this.sendJson([{ method: 'publish', params: { name: topicName, pubuid: pub.pubuid, type: pub.typeName, properties: {} } }]);
            pub.announced = true;
        }

        var ts = this._getServerTimeUs();
        var encoded = MsgPack.encodeNT4Value(pub.pubuid, ts, pub.typeId, value);
        if (this.ws && this.ws.readyState === WebSocket.OPEN) {
            this.ws.send(encoded.buffer);
        }
    }

    // Legacy alias used by older dashboard code
    publish(topic, type, value) { this.putValue(topic, value, type); }

    sendJson(data) {
        if (this.ws && this.ws.readyState === WebSocket.OPEN) {
            this.ws.send(JSON.stringify(data));
        }
    }

    getServerTime() { return this._getServerTimeUs(); }

    disconnect() {
        if (this.reconnectInterval) { clearInterval(this.reconnectInterval); this.reconnectInterval = null; }
        if (this.ws) { try { this.ws.close(); } catch(e) {} this.ws = null; }
        this.connected = false;
    }

    // ---- internal ----

    _getClientTimeUs() { return Math.round(Date.now() * 1000); }
    _getServerTimeUs() { return this._getClientTimeUs() + this._serverTimeOffsetUs; }

    // RTT timestamp sync: send [-1, 0, int, clientTimeUs], server echoes with its time.
    _syncTime() {
        this._lastPingTime = this._getClientTimeUs();
        var encoded = MsgPack.encodeNT4Value(-1, 0, NT4Types.int, this._lastPingTime);
        if (this.ws && this.ws.readyState === WebSocket.OPEN) this.ws.send(encoded.buffer);
    }

    _handleMessage(data) {
        if (typeof data === 'string') {
            var messages = JSON.parse(data);
            if (Array.isArray(messages)) messages.forEach(m => this._handleJsonMsg(m));
        } else {
            this._handleBinaryMsg(new DataView(data));
        }
    }

    _handleJsonMsg(msg) {
        if (!msg || !msg.method || !msg.params) return;
        var params = msg.params;

        if (msg.method === 'announce') {
            this._topicIds.set(params.name, params.id);
        } else if (msg.method === 'unannounce') {
            this._topicIds.delete(params.name);
        }
        // 'properties' messages are ignored
    }

    _handleBinaryMsg(view) {
        var offset = 0;
        while (offset < view.byteLength) {
            var msg = MsgPack.decodeNT4Message(view, offset);
            if (!msg) break;
            offset = msg.next;

            // Topic id -1 (or 0xff byte) = timestamp sync response
            if (msg.topicId === -1 || msg.topicId === 0xffffffff) {
                if (this._lastPingTime && typeof msg.value === 'number') {
                    var rxTime = this._getClientTimeUs();
                    var rtt = rxTime - msg.value;
                    this._serverTimeOffsetUs = msg.timestampUs + Math.round(rtt / 2) - rxTime;
                }
                continue;
            }

            // Resolve numeric id to topic name
            var topicName = null;
            this._topicIds.forEach((id, name) => { if (id === msg.topicId) topicName = name; });
            if (!topicName) continue;

            // Dispatch to matching subscriptions
            this._subscriptions.forEach((sub) => {
                var matched = sub.topics.some(t =>
                    sub.options.prefix ? topicName.startsWith(t) : topicName === t
                );
                if (matched && sub.callback) {
                    try { sub.callback(topicName, msg.timestampUs, msg.value); } catch(e) {}
                }
            });
        }
    }
}

if (typeof module !== 'undefined' && module.exports) {
    module.exports = NT4Client;
}
