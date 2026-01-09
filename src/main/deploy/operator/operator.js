(function () {
  'use strict';

  const ROBOT_ADDR = '10.51.42.2:5810';
  const ROBOT_HTTP = 'http://10.51.42.2:5805';
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
    nt: null,
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

  function publishCommand(position) {
    console.log('[HTTP] Sending command:', position);
    fetch(ROBOT_HTTP + '/command', {
      method: 'POST',
      body: position
    })
    .then(res => {
      if (res.ok) {
        console.log('[HTTP] Command sent successfully');
      } else {
        console.warn('[HTTP] Command failed:', res.status);
      }
    })
    .catch(err => {
      console.error('[HTTP] Request failed:', err);
    });
  }

  function sendReefRequest(position) {
    if (!canClick()) return;
    publishCommand(position);
    setMessage('Sent: ' + position);
  }

  function sendProcessorRequest(position) {
    if (!canClick()) return;
    publishCommand(position);
    setMessage('Sent: Processor');
  }

  function sendCoralStationRequest(position) {
    if (!canClick()) return;
    publishCommand(position);
    setMessage('Sent: Coral Station');
  }

  function bindClick(el, handler) {
    if (!el) return;
    el.addEventListener('click', handler);
  }

  function bindUi() {
    bindClick(els.orientField, function () {
      setMessage('Orient to field - not implemented via NT yet');
    });

    bindClick(els.setReef17, function () {
      setMessage('Set Reef 17 start - not implemented via NT yet');
    });

    bindClick(els.processor, function () {
      const pos = els.processor.getAttribute('data-position') || '';
      if (pos) sendProcessorRequest(pos);
    });

    bindClick(els.coralStation, function () {
      const pos = els.coralStation.getAttribute('data-position') || '';
      if (pos) sendCoralStationRequest(pos);
    });

    for (let i = 0; i < els.reefClickZones.length; i += 1) {
      const zone = els.reefClickZones[i];
      bindClick(zone, function () {
        const pos = zone.getAttribute('data-position') || '';
        if (pos) sendReefRequest(pos);
      });
    }

    if (els.fullscreenBtn) {
      els.fullscreenBtn.addEventListener('click', function () {
        if (!document.fullscreenElement) document.documentElement.requestFullscreen().catch(function () {});
        else document.exitFullscreen().catch(function () {});
      });
    }
  }

  function setupNt() {
    state.nt = new NT4Client(ROBOT_ADDR, 'operator-panel');

    state.nt.onConnect = function () {
      setLightConnected(els.connectionLight, true);
      setMessage('Connected');
    };

    state.nt.onDisconnect = function () {
      setLightConnected(els.connectionLight, false);
      setMessage('Disconnected - retrying...');
    };

    state.nt.connect();
  }

  function init() {
    setMessage('Connecting...');
    bindUi();
    setupNt();
  }

  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', init);
  else init();
})();