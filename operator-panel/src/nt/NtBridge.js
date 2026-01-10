const { NetworkTables } = require('ntcore-ts-client');

class NtBridge {
  constructor({ team, host, port }) {
    this.team = team;
    this.host = host;
    this.port = port;

    this._onConnection = () => {};
    this._onUiModelJson = () => {};
    this._onAckSeq = () => {};

    this._nt = null;

    this._topicReqName = null;
    this._topicReqSeq = null;
    this._topicReqArgsJson = null;

    this._topicUiModelJson = null;
    this._topicAckSeq = null;

    this._connected = false;
  }

  onConnection(cb) {
    this._onConnection = typeof cb === 'function' ? cb : () => {};
  }

  onUiModelJson(cb) {
    this._onUiModelJson = typeof cb === 'function' ? cb : () => {};
  }

  onAckSeq(cb) {
    this._onAckSeq = typeof cb === 'function' ? cb : () => {};
  }

  _setConnected(connected) {
    const c = !!connected;
    if (c === this._connected) return;
    this._connected = c;
    this._onConnection(c);
  }

  connect() {
    if (this._nt) return;

    const host = String(this.host || '').trim();
    const portNum = Number(this.port);

    if (!host) throw new Error('Missing host');
    if (!Number.isFinite(portNum) || portNum <= 0) throw new Error('Invalid port');

    const baseUrl = `ws://${host}:${portNum}`;

    // ntcore-ts-client appears to append its own path. Give it a valid base URL + ports.
    const opts = {
      appName: 'operator-panel',
      server: baseUrl,
      uri: baseUrl,
      serverUri: baseUrl,
      port: portNum,
      wsPort: portNum,
    };

    // Do not pass team for now. Team mode is producing an invalid URL in this version.
    this._nt = new NetworkTables(opts);

    // Topic objects
    this._topicReqName = this._nt.topic('/OperatorUI/request/name');
    this._topicReqSeq = this._nt.topic('/OperatorUI/request/seq');
    this._topicReqArgsJson = this._nt.topic('/OperatorUI/request/argsJson');

    this._topicUiModelJson = this._nt.topic('/OperatorUI/uiModelJson');
    this._topicAckSeq = this._nt.topic('/OperatorUI/ack/seq');

    // Subscribe to robot -> UI
    this._topicUiModelJson.subscribe();
    this._topicAckSeq.subscribe();

    // Forward updates
    this._topicUiModelJson.addListener((value) => {
      if (typeof value !== 'string') return;
      this._onUiModelJson(value);
    });

    this._topicAckSeq.addListener((value) => {
      if (typeof value !== 'number') return;
      this._onAckSeq(value);
    });

    // Connection tracking
    if (typeof this._nt.addRobotConnectionListener === 'function') {
      this._nt.addRobotConnectionListener((connected) => this._setConnected(connected));
    } else if (typeof this._nt.addListener === 'function') {
      this._nt.addListener('connected', () => this._setConnected(true));
      this._nt.addListener('disconnected', () => this._setConnected(false));
    } else {
      this._setConnected(false);
    }

    this._nt.connect();
  }

  sendRequest(name, seq, argsJson) {
    if (!this._nt) return;

    this._topicReqName.publish();
    this._topicReqSeq.publish();
    this._topicReqArgsJson.publish();

    this._topicReqName.setValue(String(name));
    this._topicReqSeq.setValue(Number(seq) | 0);
    this._topicReqArgsJson.setValue(argsJson ? String(argsJson) : '');
  }
}

module.exports = { NtBridge };
