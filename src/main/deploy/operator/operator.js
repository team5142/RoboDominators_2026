(function () {
  'use strict';

  // -----------------------------------------------------------------------
  // NT connection
  // -----------------------------------------------------------------------
  const ROBOT_ADDR = '10.51.42.2:5810';

  // -----------------------------------------------------------------------
  // Field-to-image coordinate mapping.
  //
  // The field images show the Blue alliance zone + a bit of the bump area.
  // Field origin (0,0) is the Blue alliance wall corner (standard WPILib).
  //
  // These values describe what region of the field the image covers.
  // Adjust FIELD_IMG_X_MAX if the image extends further than the alliance zone.
  // Y runs bottom-to-top in field coords but top-to-bottom in image pixels,
  // so the mapping inverts Y.
  //
  // CALIBRATION: The red box on screen shows exactly where these bounds land.
  // Increase X_MAX to push the right edge rightward; increase Y_MAX to push
  // the top edge upward. Match the box to the actual field boundary in the image.
  // -----------------------------------------------------------------------
  const FIELD_IMG_X_MIN = 0.0;          // meters - left edge of image (Blue wall)
  const FIELD_IMG_X_MAX = 4.5;          // meters - right edge of image (adjust to match image)
  const FIELD_IMG_Y_MIN = 0.0;          // meters - bottom of image
  const FIELD_IMG_Y_MAX = 8.05;         // meters - top of image (adjust to match image)

  // Safety padding - targets are clamped inside these bounds so a pass target
  // can never be set too close to a wall or the bump line.
  // The yellow dashed box shows this region.
  const PAD_X_MIN = 0.5;   // meters from Blue wall (don't aim right at the wall)
  const PAD_X_MAX = 4.0;   // meters - no further than alliance zone depth
  const PAD_Y_MIN = 0.5;   // meters from bottom wall
  const PAD_Y_MAX = 7.5;   // meters from bottom (field width ~8.05m)

  // Live tuning handle - accessible from Chrome DevTools console as window.fieldTuning
  // Image axes: left=high Y, right=low Y, top=high X, bottom=low X
  //
  // QUICKEST WAY TO CALIBRATE - use percent (0-100) of image dimensions:
  //   fieldTuning.setRedPct(left, top, right, bottom)   move the red box edges
  //   fieldTuning.setYellowPct(left, top, right, bottom) move the yellow box edges
  //   fieldTuning.current()                              print current values
  //
  // Example: red box left edge at 5% from left, top at 8%, right at 95%, bottom at 90%:
  //   fieldTuning.setRedPct(5, 8, 95, 90)
  window.fieldTuning = {
    // Percent-based overlay tuning (easiest for visual calibration)
    setRedPct: function(left, top, right, bottom) {
      overlayPct.red = { left: left, top: top, right: right, bottom: bottom };
      positionOverlays();
      console.log('Red box pct: left=' + left + ' top=' + top + ' right=' + right + ' bottom=' + bottom);
    },
    setYellowPct: function(left, top, right, bottom) {
      overlayPct.yellow = { left: left, top: top, right: right, bottom: bottom };
      positionOverlays();
      console.log('Yellow box pct: left=' + left + ' top=' + top + ' right=' + right + ' bottom=' + bottom);
    },
    current: function() {
      console.log('Red pct:    ', JSON.stringify(overlayPct.red));
      console.log('Yellow pct: ', JSON.stringify(overlayPct.yellow));
    }
  };

  // Percent-based overlay positions (0-100). Calibrated to match the field image.
  var overlayPct = {
    red:    { left: 4,  top: 14, right: 96, bottom: 93 },  // alliance zone boundary
    yellow: { left: 8,  top: 15, right: 92, bottom: 92 },  // safe click region (avoids walls/bump)
  };

  // Live-tunable copies (start equal to the constants above; DevTools writes to these)
  var FIELD_IMG_X_MAX_live = FIELD_IMG_X_MAX;
  var FIELD_IMG_Y_MAX_live = FIELD_IMG_Y_MAX;
  var PAD_X_MIN_live = PAD_X_MIN, PAD_X_MAX_live = PAD_X_MAX;
  var PAD_Y_MIN_live = PAD_Y_MIN, PAD_Y_MAX_live = PAD_Y_MAX;

  // Default pass targets - must match Constants.PassTargets.BLUE_PASS_TARGET_LEFT/RIGHT
  // Left: (3.46, 5.83), Right: (3.46, 2.21) from Constants.java
  const DEFAULTS = {
    left:  { x: 3.46, y: 5.83 },
    right: { x: 3.46, y: 2.21 },
  };

  // -----------------------------------------------------------------------
  // State
  // -----------------------------------------------------------------------
  const state = {
    nt: null,
    isRed: false,
    activeMode: null,           // 'left' | 'right' | null
    pending: null,              // { side, x, y } staged but not confirmed
    confirmed: {
      left:  { x: DEFAULTS.left.x,  y: DEFAULTS.left.y  },
      right: { x: DEFAULTS.right.x, y: DEFAULTS.right.y },
    },
  };

  // -----------------------------------------------------------------------
  // DOM refs
  // -----------------------------------------------------------------------
  var el = {
    fieldContainer: document.getElementById('field-container'),
    fieldImage:     document.getElementById('field-image'),
    pendingDot:     document.getElementById('pending-dot'),
    markerLeft:     document.getElementById('marker-left'),
    markerRight:    document.getElementById('marker-right'),
    debugFieldBounds: document.getElementById('debug-field-bounds'),
    debugSafeBounds:  document.getElementById('debug-safe-bounds'),
    connectionLight: document.getElementById('connection-light'),
    enabledLight:    document.getElementById('enabled-light'),
    allianceBadge:   document.getElementById('alliance-badge'),
    messageBox:      document.getElementById('message-box'),
    btnSetLeft:    document.getElementById('btn-set-left'),
    btnSetRight:   document.getElementById('btn-set-right'),
    btnResetLeft:  document.getElementById('btn-reset-left'),
    btnResetRight: document.getElementById('btn-reset-right'),
    fullscreenBtn: document.getElementById('fullscreen-btn'),
  };

  // -----------------------------------------------------------------------
  // Utilities
  // -----------------------------------------------------------------------
  function setMessage(text) {
    if (el.messageBox) el.messageBox.textContent = text || '';
  }

  function setLight(lightEl, on) {
    if (!lightEl) return;
    lightEl.classList.toggle('connected', !!on);
  }

  // Convert pixel offset within the field image element to field meters (Blue-origin).
  // Image is landscape, top-down (bird's eye) view of the Blue alliance zone:
  //   image left  = left wall  = high field Y
  //   image right = right wall = low field Y
  //   image top   = bump/neutral zone = high field X
  //   image bottom = driver station   = low field X
  // So: pixelX maps to Y (inverted), pixelY maps to X (inverted).
  function pixelToField(pixelX, pixelY) {
    var rect = el.fieldImage.getBoundingClientRect();
    var fracX = pixelX / rect.width;   // 0=left wall, 1=right wall
    var fracY = pixelY / rect.height;  // 0=bump, 1=driver station
    // Left edge = Y_MAX, right edge = Y_MIN (image left = high Y)
    var fieldY = FIELD_IMG_Y_MAX_live - fracX * (FIELD_IMG_Y_MAX_live - FIELD_IMG_Y_MIN);
    // Top edge = X_MAX, bottom edge = X_MIN (image top = high X)
    var fieldX = FIELD_IMG_X_MAX_live - fracY * (FIELD_IMG_X_MAX_live - FIELD_IMG_X_MIN);
    return { x: Math.round(fieldX * 100) / 100, y: Math.round(fieldY * 100) / 100 };
  }

  // Convert field meters to pixel offset within the field image element.
  function fieldToPixel(fieldX, fieldY) {
    var rect = el.fieldImage.getBoundingClientRect();
    // Y maps to pixelX (inverted: high Y = left = low pixelX)
    var fracX = (FIELD_IMG_Y_MAX_live - fieldY) / (FIELD_IMG_Y_MAX_live - FIELD_IMG_Y_MIN);
    // X maps to pixelY (inverted: high X = top = low pixelY)
    var fracY = (FIELD_IMG_X_MAX_live - fieldX) / (FIELD_IMG_X_MAX_live - FIELD_IMG_X_MIN);
    return {
      px: Math.round(fracX * rect.width),
      py: Math.round(fracY * rect.height),
    };
  }

  // Place an element (marker/dot) at a given field coordinate.
  function placeMarkerAtField(markerEl, fieldX, fieldY) {
    var pos = fieldToPixel(fieldX, fieldY);
    markerEl.style.left = pos.px + 'px';
    markerEl.style.top  = pos.py + 'px';
    markerEl.style.display = 'flex';
  }

  // Position the debug overlay boxes using percent values from overlayPct.
  // Call this after the image loads and on any resize.
  function positionOverlays() {
    var rect = el.fieldImage.getBoundingClientRect();
    if (!rect.width) return;

    // Red box: marks the alliance zone boundary on the image
    if (el.debugFieldBounds) {
      var r = overlayPct.red;
      el.debugFieldBounds.style.left   = r.left + '%';
      el.debugFieldBounds.style.top    = r.top + '%';
      el.debugFieldBounds.style.width  = (r.right - r.left) + '%';
      el.debugFieldBounds.style.height = (r.bottom - r.top) + '%';
    }

    // Yellow dashed box: safe click region inside the alliance zone
    if (el.debugSafeBounds) {
      var y = overlayPct.yellow;
      el.debugSafeBounds.style.left   = y.left + '%';
      el.debugSafeBounds.style.top    = y.top + '%';
      el.debugSafeBounds.style.width  = (y.right - y.left) + '%';
      el.debugSafeBounds.style.height = (y.bottom - y.top) + '%';
    }
  }

  // Mirror X for Red alliance (Red X = FIELD_LENGTH - Blue X)
  var FIELD_LENGTH = 16.51; // meters (matches Constants.java)
  function toNetworkX(fieldX) {
    return state.isRed ? (FIELD_LENGTH - fieldX) : fieldX;
  }

  // -----------------------------------------------------------------------
  // UI update
  // -----------------------------------------------------------------------
  function updateAllianceDisplay() {
    el.fieldImage.src = state.isRed ? 'field_red.jpg' : 'field_blue.jpg';
    el.allianceBadge.textContent = state.isRed ? 'RED' : 'BLUE';
    el.allianceBadge.className = 'alliance-badge ' + (state.isRed ? 'red' : 'blue');
    // Overlays and markers are redrawn once the new image finishes loading
  }

  function refreshMarkers() {
    if (state.confirmed.left) {
      placeMarkerAtField(el.markerLeft, state.confirmed.left.x, state.confirmed.left.y);
    }
    if (state.confirmed.right) {
      placeMarkerAtField(el.markerRight, state.confirmed.right.x, state.confirmed.right.y);
    }
  }

  function setActiveMode(side) {
    state.activeMode = side;
    state.pending = null;
    el.pendingDot.style.display = 'none';

    el.btnSetLeft.classList.toggle('active', side === 'left');
    el.btnSetRight.classList.toggle('active', side === 'right');

    if (side) {
      setMessage('Tap the field to set the ' + side.toUpperCase() + ' pass target');
      el.fieldContainer.classList.add('selecting');
    } else {
      setMessage('Ready');
      el.fieldContainer.classList.remove('selecting');
    }
  }

  // -----------------------------------------------------------------------
  // Field click handler
  // -----------------------------------------------------------------------
  function onFieldClick(e) {
    console.log('fieldClick - activeMode=' + state.activeMode);
    if (!state.activeMode) return;

    var rect = el.fieldImage.getBoundingClientRect();
    var rawX = (e.clientX !== undefined ? e.clientX : e.pageX) - rect.left;
    var rawY = (e.clientY !== undefined ? e.clientY : e.pageY) - rect.top;

    // Clamp to image bounds
    rawX = Math.max(0, Math.min(rawX, rect.width));
    rawY = Math.max(0, Math.min(rawY, rect.height));

    // Place the marker directly at the raw pixel position (no coordinate math yet)
    var side = state.activeMode;
    var marker = side === 'left' ? el.markerLeft : el.markerRight;
    marker.style.left    = rawX + 'px';
    marker.style.top     = rawY + 'px';
    marker.style.display = 'flex';

    // Store raw pixel coords for later use
    state.pending = { side: side, x: rawX, y: rawY };
    state.confirmed[side] = { x: rawX, y: rawY };

    setMessage(
      cap(side) + ' target set - pixel x=' + Math.round(rawX) + ' y=' + Math.round(rawY) +
      '  (coordinate math not yet calibrated)'
    );
    setActiveMode(null);
  }

  // -----------------------------------------------------------------------
  // NT publishing  (DISABLED for offline UI testing - re-enable before deploy)
  // -----------------------------------------------------------------------
  function publishTarget(_side, _x, _y) {
    // no-op while NT is disabled
  }

  function cap(s) {
    return s.charAt(0).toUpperCase() + s.slice(1);
  }

  // -----------------------------------------------------------------------
  // Button handlers
  // -----------------------------------------------------------------------
  function bindUi() {
    el.btnSetLeft.addEventListener('click', function () {
      setActiveMode(state.activeMode === 'left' ? null : 'left');
    });

    el.btnSetRight.addEventListener('click', function () {
      setActiveMode(state.activeMode === 'right' ? null : 'right');
    });

    el.btnResetLeft.addEventListener('click', function () {
      state.confirmed.left = { x: DEFAULTS.left.x, y: DEFAULTS.left.y };
      publishTarget('left', state.confirmed.left.x, state.confirmed.left.y);
      placeMarkerAtField(el.markerLeft, state.confirmed.left.x, state.confirmed.left.y);
      if (state.activeMode === 'left') setActiveMode(null);
      setMessage('LEFT target reset to default');
    });

    el.btnResetRight.addEventListener('click', function () {
      state.confirmed.right = { x: DEFAULTS.right.x, y: DEFAULTS.right.y };
      publishTarget('right', state.confirmed.right.x, state.confirmed.right.y);
      placeMarkerAtField(el.markerRight, state.confirmed.right.x, state.confirmed.right.y);
      if (state.activeMode === 'right') setActiveMode(null);
      setMessage('RIGHT target reset to default');
    });

    // Field image click and touch
    // Listen on the container div so the whole field area is clickable
    el.fieldContainer.addEventListener('click', onFieldClick);
    el.fieldContainer.addEventListener('touchend', function (e) {
      e.preventDefault();
      onFieldClick(e.changedTouches[0]);
    });

    el.fullscreenBtn.addEventListener('click', function () {
      if (!document.fullscreenElement) document.documentElement.requestFullscreen().catch(function () {});
      else document.exitFullscreen().catch(function () {});
    });
  }

  // -----------------------------------------------------------------------
  // NT setup  (DISABLED for offline UI testing - re-enable before deploy)
  // -----------------------------------------------------------------------
  function setupNt() {
    setLight(el.connectionLight, false);
    setMessage('[OFFLINE TEST] NT disabled - clicks go straight to markers');
  }

  // -----------------------------------------------------------------------
  // Init
  // -----------------------------------------------------------------------
  function init() {
    setMessage('Connecting...');
    bindUi();
    setupNt();
    // Draw default marker positions and overlays after the image loads
    el.fieldImage.addEventListener('load', function () {
      positionOverlays();
      refreshMarkers();
    });
    // If image already loaded (cached), draw immediately
    if (el.fieldImage.complete) {
      positionOverlays();
      refreshMarkers();
    }
  }

  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', init);
  else init();
})();