(function () {
  'use strict';

  const CLICK_DEBOUNCE_MS = 200;

  const els = {
    connectionLight: document.getElementById('connection-light'),
    enabledLight: document.getElementById('enabled-light'),
    messageBox: document.getElementById('message-box'),

    orientField: document.getElementById('orient-field'),
    setReef17: document.getElementById('set-reef-17'),
    processor: document.getElementById('processor'),
    coralStation: document.getElementById('coral-station'),

    reefClickZones: Array.prototype.slice.call(document.querySelectorAll('.reef-click-zone')),
    fullscreenBtn: document.getElementById('fullscreen-btn'),
  };

  const state = {
    lastClickMs: 0,
  };

  function nowMs() {
    return Date.now();
  }

  function setMessage(text) {
    if (!els.messageBox) return;
    els.messageBox.textContent = text || '';
  }

  function setLightConnected(lightEl, on) {
    if (!lightEl) return;
    if (on) lightEl.classList.add('connected');
    else lightEl.classList.remove('connected');
  }

  function canClick() {
    const t = nowMs();
    if (t - state.lastClickMs < CLICK_DEBOUNCE_MS) return false;
    state.lastClickMs = t;
    return true;
  }

  function reefIdFromPosition(pos) {
    if (typeof pos !== 'string') return '';
    const m = /_TAG_(\d+)$/.exec(pos);
    return m ? m[1] : '';
  }

  function formatRequestLabel(name, args) {
    if (!args || typeof args !== 'object') return name;

    if (typeof args.tag === 'number') return `${name} (ID ${args.tag})`;

    if (typeof args.position === 'string') {
      const id = reefIdFromPosition(args.position);
      if (id) return `${name} (ID ${id})`;
      return `${name} (${args.position})`;
    }

    return name;
  }

  async function sendRequest(name, args) {
    if (!window.opPanel || typeof window.opPanel.sendRequest !== 'function') {
      setMessage('Bridge not ready');
      return;
    }
    if (!canClick()) return;

    try {
      const res = await window.opPanel.sendRequest(name, args || null);
      if (!res || !res.ok) {
        setMessage(res && res.error ? String(res.error) : 'Request failed');
        return;
      }
      setMessage('Sent: ' + formatRequestLabel(name, args));
    } catch {
      setMessage('Send failed');
    }
  }

  function bindClick(el, name, argsFactory) {
    if (!el) return;
    el.addEventListener('click', function () {
      const args = typeof argsFactory === 'function' ? argsFactory(el) : null;
      sendRequest(name, args);
    });
  }

  function safeParseJson(s) {
    if (!s || typeof s !== 'string') return null;
    try {
      return JSON.parse(s);
    } catch {
      return null;
    }
  }

  function setElementVisible(el, visible) {
    if (!el) return;
    el.style.display = visible ? '' : 'none';
  }

  function setElementEnabled(el, enabled) {
    if (!el) return;
    if ('disabled' in el) el.disabled = !enabled;
    if (enabled) el.classList.remove('disabled');
    else el.classList.add('disabled');
  }

  function applyUiModel(model) {
    if (!model || typeof model !== 'object') return;

    if (typeof model.enabled === 'boolean') setLightConnected(els.enabledLight, model.enabled);

    const controls = model.controls && typeof model.controls === 'object' ? model.controls : null;
    if (!controls) return;

    const ids = Object.keys(controls);
    for (let i = 0; i < ids.length; i += 1) {
      const id = ids[i];
      const c = controls[id];
      if (!c || typeof c !== 'object') continue;

      const el = document.getElementById(id);
      if (!el) continue;

      if (typeof c.visible === 'boolean') setElementVisible(el, c.visible);
      if (typeof c.enabled === 'boolean') setElementEnabled(el, c.enabled);
      if (typeof c.label === 'string') el.textContent = c.label;
    }
  }

  function bindBridge() {
    if (!window.opPanel) return;

    if (typeof window.opPanel.onConnection === 'function') {
      window.opPanel.onConnection(function (p) {
        setLightConnected(els.connectionLight, !!(p && p.connected));
      });
    }

    if (typeof window.opPanel.onUiModelUpdate === 'function') {
      window.opPanel.onUiModelUpdate(function (p) {
        const model = safeParseJson(p && p.json ? p.json : '');
        if (model) applyUiModel(model);
      });
    }

    if (typeof window.opPanel.onAck === 'function') {
      window.opPanel.onAck(function (p) {
        if (!p) return;
        if (typeof p.seq === 'number') setMessage('Ack: ' + String(p.seq));
      });
    }
  }

  function bindUi() {
    bindClick(els.orientField, 'orientToField', null);

    bindClick(els.setReef17, 'setReefStart', function () {
      return { tag: 17 };
    });

    bindClick(els.processor, 'goToProcessor', function (el) {
      return { position: el.getAttribute('data-position') || '' };
    });

    bindClick(els.coralStation, 'goToCoralStation', function (el) {
      return { position: el.getAttribute('data-position') || '' };
    });

    for (let i = 0; i < els.reefClickZones.length; i += 1) {
      const zone = els.reefClickZones[i];
      bindClick(zone, 'selectReef', function (el) {
        return { position: el.getAttribute('data-position') || '' };
      });
    }

    if (els.fullscreenBtn) {
      els.fullscreenBtn.addEventListener('click', function () {
        if (!document.fullscreenElement) document.documentElement.requestFullscreen().catch(function () {});
        else document.exitFullscreen().catch(function () {});
      });
    }
  }

  function init() {
    setMessage('UI loaded');
    bindUi();
    bindBridge();
  }

  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', init);
  else init();
})();
