// NetworkTables 4.0 Client Implementation
// Based on the WPILib NT4 protocol specification

class NT4Client {
    constructor(serverAddr, appName = 'FRC_Dashboard') {
        this.serverAddr = serverAddr;
        this.appName = appName;
        this.ws = null;
        this.serverTimeOffset = 0;
        this.subscriptions = new Map();
        this.topics = new Map();
        this.values = new Map();
        this.publishedTopics = new Map();
        this.subUid = 0;
        this.pubUid = 0;
        this.connected = false;
        this.reconnectInterval = null;
        this.onConnect = null;
        this.onDisconnect = null;
        this.onChange = null;
    }

    connect() {
        if (this.ws) {
            this.ws.close();
        }

        const wsAddr = `ws://${this.serverAddr}/nt/${this.appName}`;
        console.log(`Connecting to ${wsAddr}`);
        
        this.ws = new WebSocket(wsAddr, ['networktables.first.wpi.edu']);
        
        this.ws.binaryType = 'arraybuffer';
        
        this.ws.onopen = () => {
            console.log('WebSocket connected');
            this.connected = true;
            if (this.onConnect) this.onConnect();
            
            // Clear reconnect interval
            if (this.reconnectInterval) {
                clearInterval(this.reconnectInterval);
                this.reconnectInterval = null;
            }
        };
        
        this.ws.onmessage = (event) => {
            this.handleMessage(event.data);
        };
        
        this.ws.onerror = (error) => {
            console.error('WebSocket error:', error);
        };
        
        this.ws.onclose = () => {
            console.log('WebSocket disconnected');
            this.connected = false;
            if (this.onDisconnect) this.onDisconnect();
            
            // Auto-reconnect
            if (!this.reconnectInterval) {
                this.reconnectInterval = setInterval(() => {
                    console.log('Attempting reconnect...');
                    this.connect();
                }, 1000);
            }
        };
    }

    handleMessage(data) {
        if (typeof data === 'string') {
            // JSON message
            const messages = JSON.parse(data);
            messages.forEach(msg => this.processJsonMessage(msg));
        } else {
            // Binary message
            this.processBinaryMessage(new DataView(data));
        }
    }

    processJsonMessage(msg) {
        const method = msg.method;
        const params = msg.params;

        switch (method) {
            case 'announce':
                this.handleAnnounce(params);
                break;
            case 'unannounce':
                this.handleUnannounce(params);
                break;
            case 'properties':
                this.handleProperties(params);
                break;
            default:
                console.warn('Unknown method:', method);
        }
    }

    handleAnnounce(params) {
        const { name, id, type, properties } = params;
        this.topics.set(id, { name, type, properties });
        
        // Check if we have subscriptions for this topic
        this.subscriptions.forEach((sub, subId) => {
            if (this.topicMatchesPattern(name, sub.topics)) {
                // Send subscription request
                this.sendJson([{
                    method: 'subscribe',
                    params: {
                        topics: [name],
                        subuid: subId,
                        options: sub.options || {}
                    }
                }]);
            }
        });
    }

    handleUnannounce(params) {
        const { name, id } = params;
        this.topics.delete(id);
    }

    handleProperties(params) {
        const { name, ack } = params;
        // Handle property updates if needed
    }

    processBinaryMessage(view) {
        let offset = 0;
        
        while (offset < view.byteLength) {
            // Read topic ID (int)
            const topicId = view.getInt32(offset, true);
            offset += 4;
            
            // Read timestamp (int64)
            const timestamp = view.getBigInt64(offset, true);
            offset += 8;
            
            // Read type info
            const typeInfo = view.getInt32(offset, true);
            offset += 4;
            
            const topic = this.topics.get(topicId);
            if (!topic) continue;
            
            // Decode value based on type
            const { value, bytesRead } = this.decodeValue(view, offset, topic.type);
            offset += bytesRead;
            
            // Store value
            this.values.set(topic.name, value);
            
            // Notify subscribers
            if (this.onChange) {
                this.onChange(topic.name, value, timestamp);
            }
        }
    }

    decodeValue(view, offset, type) {
        let value, bytesRead;
        
        switch (type) {
            case 'boolean':
                value = view.getUint8(offset) !== 0;
                bytesRead = 1;
                break;
            case 'double':
                value = view.getFloat64(offset, true);
                bytesRead = 8;
                break;
            case 'float':
                value = view.getFloat32(offset, true);
                bytesRead = 4;
                break;
            case 'int':
                value = view.getInt32(offset, true);
                bytesRead = 4;
                break;
            case 'string':
                const length = view.getInt32(offset, true);
                offset += 4;
                const bytes = new Uint8Array(view.buffer, view.byteOffset + offset, length);
                value = new TextDecoder().decode(bytes);
                bytesRead = 4 + length;
                break;
            default:
                // For arrays and other complex types
                value = null;
                bytesRead = 0;
        }
        
        return { value, bytesRead };
    }

    subscribe(topics, callback, options = {}) {
        const subId = ++this.subUid;
        
        this.subscriptions.set(subId, {
            topics: Array.isArray(topics) ? topics : [topics],
            callback,
            options
        });
        
        if (this.connected) {
            this.sendJson([{
                method: 'subscribe',
                params: {
                    topics: Array.isArray(topics) ? topics : [topics],
                    subuid: subId,
                    options
                }
            }]);
        }
        
        return subId;
    }

    unsubscribe(subId) {
        this.subscriptions.delete(subId);
        
        if (this.connected) {
            this.sendJson([{
                method: 'unsubscribe',
                params: {
                    subuid: subId
                }
            }]);
        }
    }

    getValue(topic, defaultValue = null) {
        return this.values.has(topic) ? this.values.get(topic) : defaultValue;
    }

    sendJson(data) {
        if (this.ws && this.ws.readyState === WebSocket.OPEN) {
            this.ws.send(JSON.stringify(data));
        }
    }

    topicMatchesPattern(topic, patterns) {
        return patterns.some(pattern => {
            const regex = new RegExp('^' + pattern.replace(/\*/g, '.*') + '$');
            return regex.test(topic);
        });
    }

    getServerTime() {
        return Date.now() * 1000 + this.serverTimeOffset;
    }

    publish(topic, type, value) {
        let pubInfo = this.publishedTopics.get(topic);

        if (!pubInfo) {
            const pubId = ++this.pubUid;
            pubInfo = { id: pubId, type, announced: false, topicId: null };
            this.publishedTopics.set(topic, pubInfo);
        }

        if (!this.connected) {
            return;
        }

        if (!pubInfo.announced) {
            this.sendJson([{
                method: 'publish',
                params: {
                    name: topic,
                    pubuid: pubInfo.id,
                    type: type,
                    properties: {}
                }
            }]);
            pubInfo.announced = true;

            // Topic ID comes from server announce; we will send values once it is known.
            const found = Array.from(this.topics.entries()).find(([, t]) => t.name === topic);
            if (found) {
                pubInfo.topicId = found[0];
            }
        }

        // Refresh topicId if we did not have it yet
        if (pubInfo.topicId == null) {
            const found = Array.from(this.topics.entries()).find(([, t]) => t.name === topic);
            if (found) {
                pubInfo.topicId = found[0];
            } else {
                return;
            }
        }

        this.sendBinaryValue(pubInfo.topicId, type, value);
    }

    sendBinaryValue(topicId, type, value) {
        if (!this.ws || this.ws.readyState !== WebSocket.OPEN) {
            return;
        }

        // Frame format: int32 topicId, int64 timestamp, int32 typeInfo, value bytes
        // typeInfo: currently unused by this client, send 0
        const timestamp = BigInt(this.getServerTime());

        let valueBytes;
        let valueLen = 0;

        if (type === 'boolean') {
            valueLen = 1;
            valueBytes = new Uint8Array(valueLen);
            valueBytes[0] = value ? 1 : 0;
        } else if (type === 'double') {
            valueLen = 8;
            valueBytes = new Uint8Array(valueLen);
            new DataView(valueBytes.buffer).setFloat64(0, Number(value), true);
        } else if (type === 'int') {
            valueLen = 4;
            valueBytes = new Uint8Array(valueLen);
            new DataView(valueBytes.buffer).setInt32(0, Number(value) | 0, true);
        } else if (type === 'string') {
            const enc = new TextEncoder();
            const strBytes = enc.encode(String(value));
            valueLen = 4 + strBytes.length;
            valueBytes = new Uint8Array(valueLen);
            const dv = new DataView(valueBytes.buffer);
            dv.setInt32(0, strBytes.length, true);
            valueBytes.set(strBytes, 4);
        } else {
            // Unsupported type
            return;
        }

        const buf = new ArrayBuffer(4 + 8 + 4 + valueLen);
        const dv = new DataView(buf);
        let off = 0;

        dv.setInt32(off, topicId, true);
        off += 4;
        dv.setBigInt64(off, timestamp, true);
        off += 8;
        dv.setInt32(off, 0, true);
        off += 4;

        new Uint8Array(buf, off).set(valueBytes);

        this.ws.send(buf);
    }
    
    disconnect() {
        if (this.ws) {
            this.ws.close();
            this.ws = null;
        }
        if (this.reconnectInterval) {
            clearInterval(this.reconnectInterval);
            this.reconnectInterval = null;
        }
    }
}

// Export for use in dashboard
if (typeof module !== 'undefined' && module.exports) {
    module.exports = NT4Client;
}
