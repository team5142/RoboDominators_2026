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
        console.log('[NT4] Announce:', name, 'ID:', id, 'Type:', type);
        this.topics.set(id, { name, type, properties });
        
        // Update any pending published topics with this ID
        const pubInfo = this.publishedTopics.get(name);
        if (pubInfo && pubInfo.topicId == null) {
            pubInfo.topicId = id;
            console.log('[NT4] Resolved topic ID for', name, ':', id);
        }
        
        // Check if we have subscriptions for this topic
        this.subscriptions.forEach((sub, subId) => {
            if (this.topicMatchesPattern(name, sub.topics)) {
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
        // We only publish via JSON setproperties, we do not consume binary updates
        // Skip processing to avoid decode errors
        console.log('[NT4] Ignoring binary message (not needed for publish-only client)');
        return;
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
            console.warn('[NT4] Publish failed: not connected');
            return;
        }

        if (!pubInfo.announced) {
            console.log('[NT4] Announcing topic:', topic);
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

        // Use setproperties to publish value (JSON method, not binary)
        console.log('[NT4] Publishing via setproperties:', topic, 'value:', value);
        this.sendJson([{
            method: 'setproperties',
            params: {
                name: topic,
                update: {
                    value: value
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
