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
        // Get or create publisher info
        let pubInfo = this.publishedTopics.get(topic);
        
        if (!pubInfo) {
            const pubId = ++this.pubUid;
            pubInfo = { id: pubId, type, announced: false };
            this.publishedTopics.set(topic, pubInfo);
        }
        
        if (!this.connected) {
            return; // Can't send if not connected
        }
        
        // First time publishing this topic - announce it
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
        }
        
        // Send value update using JSON (MessagePack compatible format)
        // The server expects: [topicId, timestamp, typeCode, value]
        // But we'll use JSON messages instead of binary
        const timestamp = this.getServerTime();
        
        // Use a simple timestamped update message
        this.sendJson([{
            method: 'setproperties',
            params: {
                name: topic,
                update: {
                    '.value': value,
                    '.timestamp': timestamp
                }
            }
        }]);
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
