/* ==========================================================================
   mission.js — Waypoint mission system
   Depends on: app.js (ros, SERVER_URL, stage, rootObject, waypointLayer,
               poseHasBeenSet, showToast)
   ========================================================================== */


/* ---------- STATE ---------- */

var waypoints      = [];
var waypointMode   = false;
var missionRunning = false;


/* ---------- WAYPOINT ARROW (orange) ---------- */

var wpArrowContainer = new createjs.Container();
wpArrowContainer.visible = false;
rootObject.addChild(wpArrowContainer);

(function buildWpArrow() {
  var shaft = new createjs.Shape();
  shaft.graphics.beginFill("#f97316").drawRect(0, -0.06, 0.65, 0.12);
  var head = new createjs.Shape();
  head.graphics.beginFill("#f97316")
    .moveTo(1.0, 0).lineTo(0.65, -0.22).lineTo(0.65, 0.22).closePath();
  wpArrowContainer.addChild(shaft);
  wpArrowContainer.addChild(head);
})();


/* ---------- ENABLE WAYPOINT PLACEMENT ---------- */

function enableWaypointMode() {
  if (!poseHasBeenSet) { showToast("⚠ Set 2D Pose Estimate first", "error"); return; }
  waypointMode     = true;
  poseEstimateMode = false;
  goalPoseMode     = false;
  var mapEl = document.getElementById("map");
  var banner = document.getElementById("pose-banner");
  var coords = document.getElementById("pose-coords");
  var addWp  = document.getElementById("btn-add-wp");
  if (mapEl)  mapEl.style.cursor          = "crosshair";
  if (banner) banner.style.display        = "flex";
  if (coords) coords.textContent          = "Click on map to place waypoint…";
  if (addWp)  addWp.classList.add("btn-wp-active");
  showToast("📍 Click and drag to place waypoint", "info");
}


/* ---------- REDRAW MARKERS ON MAP ---------- */

function redrawWaypointMarkers() {
  waypointLayer.removeAllChildren();
  var scaleX_fix = (typeof FLIP_X !== "undefined" && FLIP_X < 0) ? -1 : 1;

  waypoints.forEach(function (wp, idx) {
    /* Small filled circle — pin dot */
    var dot = new createjs.Shape();
    dot.graphics
      .beginFill("#f97316")
      .drawCircle(0, 0, 0.10);
    dot.x = wp.x;
    dot.y = wp.y;

    /* Direction tick showing heading */
    var tick = new createjs.Shape();
    tick.graphics
      .setStrokeStyle(0.035)
      .beginStroke("#f97316")
      .moveTo(0, 0)
      .lineTo(0.18, 0);
    tick.x        = wp.x;
    tick.y        = wp.y;
    tick.rotation = wp.yaw * (180 / Math.PI);

    /* Number label only — clean and minimal */
    var numLabel = new createjs.Text(
      String(idx + 1),
      "bold 0.14px Arial",
      "#ffffff"
    );
    numLabel.textAlign    = "center";
    numLabel.textBaseline = "middle";
    numLabel.x      = wp.x;
    numLabel.y      = wp.y;
    numLabel.scaleY = -1;
    numLabel.scaleX = scaleX_fix;

    waypointLayer.addChild(dot);
    waypointLayer.addChild(tick);
    waypointLayer.addChild(numLabel);
  });
  stage.update();
}


/* ---------- RENDER WAYPOINT LIST (sidebar) ---------- */

function renderWpList() {
  var el = document.getElementById("wp-list");
  if (waypoints.length === 0) {
    el.innerHTML = "<div style='color:var(--muted);font-size:13px;padding:8px 0;'>No waypoints yet</div>";
    var btnStart = document.getElementById("btn-start-mission");
    if (btnStart) btnStart.disabled = true;
    return;
    return;
  }

  el.innerHTML = waypoints.map(function (wp, i) {
    var delay = wp.delay !== undefined ? wp.delay : 0;
    return (
      "<div class='wp-item' draggable='true' data-idx='" + i + "'" +
      " ondragstart='wpDragStart(event," + i + ")'" +
      " ondragover='wpDragOver(event)'" +
      " ondrop='wpDrop(event," + i + ")'" +
      " ondragend='wpDragEnd(event)'>" +
      "<span class='wp-drag'>⠿</span>" +
      "<span class='wp-name'>" + (i + 1) + ". " + wp.name + "</span>" +
      "<div class='wp-delay-wrap'>" +
        "<input class='wp-delay-input' type='number' min='0' step='1' value='" + delay + "'" +
        " title='Delay after arriving (s)'" +
        " onchange='setWpDelay(" + i + ", this.value)'" +
        " onclick='event.stopPropagation()'>" +
      "</div>" +
      "<button class='wp-del' onclick='removeWaypoint(" + i + ")'>✕</button>" +
      "</div>"
    );
  }).join("");

  var _bs = document.getElementById("btn-start-mission");
  if (_bs) _bs.disabled = false;
}


/* ---------- DRAG-TO-REORDER ---------- */

var _dragIdx = null;

function wpDragStart(e, idx) {
  _dragIdx = idx;
  e.currentTarget.classList.add("wp-dragging");
  e.dataTransfer.effectAllowed = "move";
}
function wpDragOver(e) {
  e.preventDefault();
  e.dataTransfer.dropEffect = "move";
  document.querySelectorAll(".wp-item").forEach(function (el) {
    el.classList.remove("wp-drag-over");
  });
  e.currentTarget.classList.add("wp-drag-over");
}
function wpDrop(e, toIdx) {
  e.preventDefault();
  if (_dragIdx === null || _dragIdx === toIdx) return;
  var moved = waypoints.splice(_dragIdx, 1)[0];
  waypoints.splice(toIdx, 0, moved);
  renderWpList();
  redrawWaypointMarkers();
  saveMissionFile();
}
function wpDragEnd() {
  _dragIdx = null;
  document.querySelectorAll(".wp-item").forEach(function (el) {
    el.classList.remove("wp-dragging");
    el.classList.remove("wp-drag-over");
  });
}


/* ---------- WAYPOINT HELPERS ---------- */

function setWpDelay(idx, val) {
  waypoints[idx].delay = Math.max(0, parseFloat(val) || 0);
  saveMissionFile();
}
function removeWaypoint(idx) {
  waypoints.splice(idx, 1);
  renderWpList();
  redrawWaypointMarkers();
  saveMissionFile();
}
function clearWaypoints() {
  waypoints = [];
  renderWpList();
  redrawWaypointMarkers();
  showToast("🗑 Waypoints cleared", "info");
}


/* ---------- SAVE MISSION FILE ---------- */

function saveMissionFile() {
  fetch(SERVER_URL + "/mission/save", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ waypoints: waypoints })
  }).catch(() => showToast("⚠ Failed to save mission", "error"));
}


/* ---------- START MISSION ---------- */

function startMission() {
  if (waypoints.length === 0) { showToast("⚠ Add waypoints first", "error"); return; }
  fetch(SERVER_URL + "/mission/save", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ waypoints: waypoints })
  })
    .then(r => r.json())
    .then(function () { return fetch(SERVER_URL + "/mission/start", { method: "POST" }); })
    .then(r => r.json())
    .then(function () {
      missionRunning = true;
      var bs = document.getElementById("btn-start-mission");
      var bst = document.getElementById("btn-stop-mission");
      if (bs)  { bs.textContent = "🚀 Running…"; bs.disabled = true; bs.style.display = "none"; }
      if (bst) { bst.style.display = "block"; }
      showToast("🚀 Mission started — " + waypoints.length + " waypoints", "success");
      pollMissionStatus();
    })
    .catch(() => showToast("⚠ Failed to start mission", "error"));
}


/* ---------- STOP MISSION ---------- */

function stopMission() {
  fetch(SERVER_URL + "/mission/stop", { method: "POST" })
    .then(r => r.json())
    .then(function () {
      missionRunning = false;
      var bs  = document.getElementById("btn-start-mission");
      var bst = document.getElementById("btn-stop-mission");
      if (bs)  { bs.textContent = "▶ Run Mission"; bs.disabled = (waypoints.length === 0); bs.style.display = "block"; }
      if (bst) { bst.style.display = "none"; }
      showToast("⛔ Mission stopped", "info");
    })
    .catch(() => showToast("⚠ Failed to stop mission", "error"));
}


/* ---------- POLL MISSION STATUS ---------- */

function pollMissionStatus() {
  if (!missionRunning) return;
  fetch(SERVER_URL + "/mission/status")
    .then(r => r.json())
    .then(function (data) {
      if (!data.running) {
        missionRunning = false;
        var bs  = document.getElementById("btn-start-mission");
        var bst = document.getElementById("btn-stop-mission");
        if (bs)  { bs.textContent = "▶ Run Mission"; bs.disabled = (waypoints.length === 0); bs.style.display = "block"; }
        if (bst) { bst.style.display = "none"; }
        showToast("✅ Mission complete", "success");
      } else {
        setTimeout(pollMissionStatus, 3000);
      }
    });
}


/* ---------- ENABLE MISSION BUTTONS (called after pose set) ---------- */

function enableMissionButtons() {
  var el = document.getElementById("btn-add-wp");
  if (el) el.disabled = false;
}


/* ---------- LOAD EXISTING MISSION ON PAGE LOAD ---------- */

fetch(SERVER_URL + "/mission")
  .then(r => r.json())
  .then(function (data) {
    if (data.waypoints && data.waypoints.length > 0) {
      waypoints = data.waypoints;
      renderWpList();
    }
  })
  .catch(function () {});   /* silent — server may not be up */