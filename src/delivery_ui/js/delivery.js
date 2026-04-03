/* ==========================================================================
   delivery.js — C4i4 Delivery UI
   Depends on: app.js (SERVER_URL, showToast), localization.js (poseHasBeenSet)
   ========================================================================== */

var _deliveryPollTimer  = null;
var _deliveryRunning    = false;
var _deliveryModeActive = false;

/* ─────────────────────────────────────────────
   LOCALIZATION GATE — checks poseHasBeenSet
   from localization.js before allowing start
───────────────────────────────────────────── */
function _isLocalized() {
  return (typeof poseHasBeenSet !== 'undefined' && poseHasBeenSet === true);
}

function _showLocGate(visible) {
  var gate = document.getElementById('deliveryLocGate');
  if (gate) gate.style.display = visible ? 'block' : 'none';
}

/* ─────────────────────────────────────────────
   DELIVERY MODE TOGGLE
───────────────────────────────────────────── */
function onDeliveryModeToggle(checked) {
  _deliveryModeActive = checked;

  /* Update toggle knob colour */
  var slider = document.getElementById('deliveryModeSlider');
  var knob   = document.getElementById('deliveryModeKnob');
  if (slider) slider.style.background = checked ? '#10b981' : '#374151';
  if (knob)   knob.style.transform    = checked ? 'translateX(20px)' : 'translateX(0)';

  if (checked) {
    if (!_isLocalized()) {
      _showLocGate(true);
      /* Revert toggle off */
      var chk = document.getElementById('deliveryModeToggle');
      if (chk) chk.checked = false;
      if (slider) slider.style.background = '#374151';
      if (knob)   knob.style.transform    = 'translateX(0)';
      _deliveryModeActive = false;
      showToast('⚠ Complete localization before enabling Delivery Mode', 'error');
      return;
    }
    _showLocGate(false);
    _setDeliveryButtons(false); /* enable Start, keep Stop disabled */
    var startBtn = document.getElementById('deliveryStartBtn');
    if (startBtn) startBtn.disabled = false;
    showToast('✅ Delivery Mode enabled', 'success');
  } else {
    _showLocGate(false);
    /* If delivery is running, stop it */
    if (_deliveryRunning) stopDelivery();
    var startBtn = document.getElementById('deliveryStartBtn');
    if (startBtn) startBtn.disabled = true;
    showToast('⏸ Delivery Mode disabled', 'info');
  }
}

/* ─────────────────────────────────────────────
   START DELIVERY — runs ros2 launch load_cell_pkg delivery_v2.launch.py
───────────────────────────────────────────── */
function startDelivery() {
  if (!_isLocalized()) {
    _showLocGate(true);
    showToast('⚠ Localization required before starting delivery', 'error');
    return;
  }

  fetch(SERVER_URL + '/delivery/start', { method: 'POST' })
    .then(function(r) { return r.json(); })
    .then(function(d) {
      if (d.status === 'already_running') {
        showToast('⚠ Delivery already running', 'info');
      } else {
        showToast('✅ Delivery started', 'success');
        _deliveryRunning = true;
        _startDeliveryPoll();
        _setDeliveryButtons(true);
        /* Update state badge */
        var stateEl = document.getElementById('deliveryState');
        if (stateEl) { stateEl.textContent = '🚀 Running…'; stateEl.style.color = '#10b981'; }
      }
    })
    .catch(function() { showToast('⚠ Failed to start delivery', 'error'); });
}

/* ─────────────────────────────────────────────
   STOP DELIVERY — kills launch file and sends robot home
───────────────────────────────────────────── */
function stopDelivery() {
  fetch(SERVER_URL + '/delivery/stop', { method: 'POST' })
    .then(function(r) { return r.json(); })
    .then(function() {
      showToast('⏹ Delivery stopped — robot returning home', 'info');
      _deliveryRunning = false;
      _stopDeliveryPoll();
      _setDeliveryButtons(false);
      _resetDeliveryUI();
      /* Re-enable Start if mode is still on */
      if (_deliveryModeActive) {
        var startBtn = document.getElementById('deliveryStartBtn');
        if (startBtn) startBtn.disabled = false;
      }
    })
    .catch(function() { showToast('⚠ Failed to stop delivery', 'error'); });
}

/* ─────────────────────────────────────────────
   STATUS POLLING — subscribes weight from /load_cell_data
───────────────────────────────────────────── */
function _startDeliveryPoll() {
  if (_deliveryPollTimer) return;
  _deliveryPollTimer = setInterval(_pollDeliveryStatus, 800);
}

function _stopDeliveryPoll() {
  if (_deliveryPollTimer) { clearInterval(_deliveryPollTimer); _deliveryPollTimer = null; }
}

function _pollDeliveryStatus() {
  fetch(SERVER_URL + '/delivery/status')
    .then(function(r) {
      if (r.status === 404 || r.status === 204) {
        _stopDeliveryPoll();
        _resetDeliveryUI();
        return null;
      }
      return r.json();
    })
    .then(function(d) { if (d) _updateDeliveryUI(d); })
    .catch(function() {});
}

/* ─────────────────────────────────────────────
   UI UPDATE — weight from /load_cell_data (Float32)
───────────────────────────────────────────── */
var _STATE_LABELS = {
  'IDLE':                 { label: '💤 Idle — waiting for load',          color: '#6b7280' },
  'DOCK_LOAD':            { label: '🎯 Docking at loading station',        color: '#8b5cf6' },
  'WAIT_BEFORE_NAV':      { label: '⏳ Loaded — moving to delivery soon',  color: '#f59e0b' },
  'NAVIGATE_TO_DELIVERY': { label: '🚗 Navigating to delivery',            color: '#3b82f6' },
  'DOCK_DELIVERY':        { label: '🎯 Docking at delivery station',        color: '#8b5cf6' },
  'WAIT_FOR_UNLOAD':      { label: '📦 At delivery — please unload',       color: '#10b981' },
  'RETURN_WAIT':          { label: '⏳ Unloaded — returning home soon',    color: '#f59e0b' },
  'NAVIGATE_TO_LOADING':  { label: '🏠 Navigating to loading station',     color: '#3b82f6' },
  'DOCK_LOAD_RETURN':     { label: '🎯 Docking back at loading station',   color: '#8b5cf6' },
};

function _updateDeliveryUI(d) {
  var stateEl  = document.getElementById('deliveryState');
  var weightEl = document.getElementById('deliveryWeight');
  var barEl    = document.getElementById('deliveryWeightBar');

  if (!stateEl) return;

  /* State label */
  var info = _STATE_LABELS[d.state] || { label: d.state, color: '#6b7280' };
  stateEl.textContent = info.label;
  stateEl.style.color = info.color;

  /* Weight — from /load_cell_data Float32 */
  var kg        = (d.weight || 0).toFixed(2);
  var threshold = (d.threshold || 2.0);
  weightEl.textContent = kg + ' kg';
  weightEl.style.color = d.weight >= threshold ? '#10b981' : '#9ca3af';

  /* Also update topbar weight widget */
  var topbarW = document.getElementById('topbar-weight-val');
  if (topbarW) { topbarW.textContent = kg + ' kg'; topbarW.style.color = d.weight >= threshold ? '#10b981' : 'var(--accent)'; }

  /* Weight bar */
  var pct = Math.min(100, (d.weight / threshold) * 100);
  barEl.style.width      = pct + '%';
  barEl.style.background = d.weight >= threshold ? '#10b981' : '#3b82f6';

  /* Update bottom-bar badges */
  var stateBadge  = document.getElementById('del-state-badge');
  var weightBadge = document.getElementById('del-weight-badge');
  if (stateBadge)  { stateBadge.textContent  = d.state || 'IDLE'; stateBadge.style.color = info.color; }
  if (weightBadge) { weightBadge.textContent = kg + ' kg'; }
}

function _resetDeliveryUI() {
  var stateEl  = document.getElementById('deliveryState');
  var weightEl = document.getElementById('deliveryWeight');
  var barEl    = document.getElementById('deliveryWeightBar');
  if (!stateEl) return;
  stateEl.textContent  = 'Node not running';
  stateEl.style.color  = '#6b7280';
  weightEl.textContent = '— kg';
  barEl.style.width    = '0%';

  var stateBadge  = document.getElementById('del-state-badge');
  var weightBadge = document.getElementById('del-weight-badge');
  if (stateBadge)  stateBadge.textContent  = 'IDLE';
  if (weightBadge) weightBadge.textContent = '— kg';
}

function _setDeliveryButtons(running) {
  var startBtn = document.getElementById('deliveryStartBtn');
  var stopBtn  = document.getElementById('deliveryStopBtn');
  if (!startBtn) return;
  startBtn.disabled = running;
  stopBtn.disabled  = !running;
}

/* ─────────────────────────────────────────────
   INIT — called by switchView('delivery')
───────────────────────────────────────────── */
function initDeliveryView() {
  /* Check localization state */
  var localized = _isLocalized();
  _showLocGate(!localized && _deliveryModeActive);

  /* Restore toggle visual state */
  var slider = document.getElementById('deliveryModeSlider');
  var knob   = document.getElementById('deliveryModeKnob');
  var chk    = document.getElementById('deliveryModeToggle');
  if (chk)    chk.checked             = _deliveryModeActive;
  if (slider) slider.style.background = _deliveryModeActive ? '#10b981' : '#374151';
  if (knob)   knob.style.transform    = _deliveryModeActive ? 'translateX(20px)' : 'translateX(0)';

  /* Check if delivery is already running */
  fetch(SERVER_URL + '/delivery/status')
    .then(function(r) {
      if (r.status === 404) return null;
      if (r.status !== 204) {
        _deliveryRunning = true;
        _setDeliveryButtons(true);
        _startDeliveryPoll();
      } else {
        /* Not running — enable Start only if mode is on and localized */
        var startBtn = document.getElementById('deliveryStartBtn');
        if (startBtn) startBtn.disabled = !(_deliveryModeActive && localized);
      }
      return null;
    })
    .catch(function() {});
}

/* ─────────────────────────────────────────────
   ALWAYS-ON WEIGHT POLL
   Runs from page load — independent of delivery
   state. Updates sidebar weight + topbar widget.
───────────────────────────────────────────── */
(function startWeightPoll() {
  function poll() {
    fetch(SERVER_URL + '/weight/status')
      .then(function(r) { if (r.ok) return r.json(); })
      .then(function(d) {
        if (!d && d !== 0) return;
        var kg        = (d.weight || 0).toFixed(2);
        var threshold = 2.0;

        /* Sidebar weight */
        var weightEl = document.getElementById('deliveryWeight');
        if (weightEl) {
          weightEl.textContent = kg + ' kg';
          weightEl.style.color = d.weight >= threshold ? '#10b981' : '#9ca3af';
        }
        /* Weight bar */
        var barEl = document.getElementById('deliveryWeightBar');
        if (barEl) {
          var pct = Math.min(100, (d.weight / threshold) * 100);
          barEl.style.width      = pct + '%';
          barEl.style.background = d.weight >= threshold ? '#10b981' : '#3b82f6';
        }
        /* Topbar weight */
        var topbarW = document.getElementById('topbar-weight-val');
        if (topbarW) {
          topbarW.textContent = kg + ' kg';
          topbarW.style.color = d.weight >= threshold ? '#10b981' : 'var(--accent)';
        }
        /* Bottom bar badge */
        var badge = document.getElementById('del-weight-badge');
        if (badge) badge.textContent = kg + ' kg';
      })
      .catch(function() {});
  }
  poll();
  setInterval(poll, 1000);
})();