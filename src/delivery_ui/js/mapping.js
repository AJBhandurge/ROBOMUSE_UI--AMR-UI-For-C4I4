/* ==========================================================================
   mapping.js — SLAM mapping, map save, map list, preview
   Depends on: app.js (SERVER_URL, showToast, resetPoseState,
               lockLocalization, unlockLocalization, mapMode)
   ========================================================================== */


/* ---------- MAP LIST ---------- */

function loadMapList() {
  fetch(SERVER_URL + "/maps")
    .then(r => r.json())
    .then(function (data) {
      var sel = document.getElementById("mapSelect");
      sel.innerHTML = "";
      if (!data.length) { sel.innerHTML = "<option>No maps found</option>"; return; }
      data.forEach(function (m) {
        var o = document.createElement("option");
        o.value = o.textContent = m;
        sel.appendChild(o);
      });
    })
    .catch(() => showToast("⚠ Could not reach server", "error"));
}
loadMapList();


/* ---------- PREVIEW MAP ---------- */

function selectMap() {
  var map = document.getElementById("mapSelect").value;
  if (!map) { showToast("⚠ Select a map first", "error"); return; }

  /* Use the dedicated preview image (always inside a visible container) */
  var img = document.getElementById("mapPreviewImage");
  if (!img) { showToast("⚠ Preview element not found", "error"); return; }

  /* Strip .yaml extension — server expects .pgm or image filename */
  var imgName = map.replace(/\.yaml$/, ".pgm");

  img.onload = function () {
    img.style.display = "block";
    var emptyMsg = document.getElementById("loc-empty-msg");
    if (emptyMsg) emptyMsg.style.display = "none";
    showToast("🗺 Preview: " + imgName, "info");
  };
  img.onerror = function () {
    /* Try .png fallback */
    var pngName = map.replace(/\.(pgm|yaml)$/, ".png");
    if (img.src.indexOf(".png") === -1) {
      img.src = SERVER_URL + "/map_image/" + pngName;
    } else {
      showToast("⚠ Could not load map image — check maps folder", "error");
    }
  };
  img.src = SERVER_URL + "/map_image/" + imgName;
}


/* ---------- RESET POSE STATE ---------- */

function resetPoseState() {
  poseHasBeenSet     = false;
  freezeOdom         = true;
  robotLayer.visible = false;
  laserLayer.visible = false; if(typeof clearScan==="function") clearScan();
  laserLayer.removeAllChildren();
  var btnHome = document.getElementById("btn-home");
  if (btnHome) btnHome.disabled = true;
}


/* ---------- START MAPPING ---------- */

function startMapping() {
  mapMode = "mapping";
  resetPoseState();
  lockLocalization();
  if (typeof resetMapFit === "function") resetMapFit();   /* clears mapBitmap + resets seq */
  /* Also clear costmap overlay from previous localization session */
  if (typeof clearCostmap === "function") clearCostmap();
  if (typeof costmapVisible !== "undefined") costmapVisible = false;
  var costToggle = document.getElementById("toggle-costmap");
  if (costToggle) costToggle.checked = false;
  document.getElementById("mapImage").style.display = "none";
  document.getElementById("map").style.display      = "block";
  showToast("⏳ Starting mapping...", "info");
  fetch(SERVER_URL + "/start_mapping", { method: "POST" })
    .then(r => r.json())
    .then(function() {
      /* Show robot at origin immediately — odom will update it */
      robotLayer.x        = 0;
      robotLayer.y        = 0;
      robotLayer.rotation = 0;
      robotLayer.visible  = true;
      laserLayer.visible  = true;
      poseHasBeenSet      = true;
      stage.update();
      /* Draw robot on overlay immediately (before first scan arrives) */
      if (typeof redrawOverlay === "function") redrawOverlay();
      showToast("✅ Mapping started — drive the robot", "success");
    })
    .catch(() => showToast("⚠ Failed to start mapping", "error"));
}


/* ---------- STOP MAPPING ---------- */

function stopMapping() {
  fetch(SERVER_URL + "/stop_mapping", { method: "POST" })
    .then(r => r.json())
    .then(function () {
      unlockLocalization();
      stopMapPolling();          /* stop live polling — map is frozen */
      poseHasBeenSet = false;    /* hide robot + scan until next session */
      robotLayer.visible = false;
      if (typeof clearScan === "function") clearScan();
      mapMode = "idle";
      showToast("⏹ Mapping stopped — save or discard the map", "info");
    })
    .catch(() => showToast("⚠ Failed to stop mapping", "error"));
}


/* ---------- SAVE / DISCARD MAP ---------- */

function _resetToIdle() {
  robotLayer.visible = false;
  laserLayer.visible = false; if (typeof clearScan === "function") clearScan();
  mapMode = "idle";
  if (typeof mapBitmap !== "undefined" && mapBitmap) {
    rootObject.removeChild(mapBitmap); mapBitmap = null;
  }
  if (typeof stopMapPolling === "function") stopMapPolling();
  if (typeof resetMapFit    === "function") resetMapFit();
  /* Unlock localization so user can immediately switch to localization */
  if (typeof unlockLocalization === "function") unlockLocalization();
  /* Reset UI buttons — restore to "ready to start new mapping" state */
  var saveBlock  = document.getElementById("save-map-block");
  var startBlock = document.getElementById("btn-start-mapping");
  var stopBlock  = document.getElementById("btn-stop-mapping");
  var ph         = document.getElementById("map-placeholder");
  var mc         = document.getElementById("mapContainer");
  if (saveBlock)  saveBlock.style.display  = "none";
  if (startBlock) startBlock.style.display = "block";
  if (stopBlock)  stopBlock.style.display  = "none";
  if (mc)         mc.style.display         = "none";
  if (ph)         ph.style.display         = "flex";
  loadMapList();
}

function discardMapAndReturn() {
  _resetToIdle();
  showToast("🗑 Map discarded", "info");
  setTimeout(function() { switchView("home"); }, 400);
}

var _savingMap = false;
function saveMapAndReturn() {
  if (_savingMap) return;
  var name = document.getElementById("mapname").value.trim();
  if (!name) { showToast("⚠ Enter a map name first", "error"); return; }
  _savingMap = true;

  /* Show saving indicator */
  var saveBtn = document.querySelector("#save-map-block .btn-success");
  if (saveBtn) { saveBtn.textContent = "⏳ Saving…"; saveBtn.disabled = true; }

  fetch(SERVER_URL + "/save_map", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ name: name })
  })
    .then(r => r.json())
    .then(function(data) {
      _savingMap = false;
      if (saveBtn) { saveBtn.textContent = "💾 Save Map"; saveBtn.disabled = false; }

      if (data.status === "error") {
        showToast("⚠ Save failed: " + (data.message || "unknown"), "error");
        return;
      }

      showToast("✅ Map saved: " + name, "success");
      _resetToIdle();
      setTimeout(function() { switchView("home"); }, 400);
    })
    .catch(function(err) {
      _savingMap = false;
      if (saveBtn) { saveBtn.textContent = "💾 Save Map"; saveBtn.disabled = false; }
      showToast("⚠ Network error saving map", "error");
    });
}

function saveMap() {
  var name = document.getElementById("mapname").value.trim();
  if (!name) { showToast("⚠ Enter a map name first", "error"); return; }
  fetch(SERVER_URL + "/save_map", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ name: name })
  })
    .then(r => r.json())
    .then(() => {
      showToast("✅ Map saved: " + name, "success");
      setTimeout(loadMapList, 2000);
    })
    .catch(() => { _savingMap = false; showToast("⚠ Failed to save map", "error"); });
}


/* ---------- ODOM SUBSCRIBER (mapping mode only) ---------- */

var odomTopic = new ROSLIB.Topic({
  ros          : ros,
  name         : RobotConfig.topics.odom,
  messageType  : "nav_msgs/Odometry",
  compression  : "none",
  throttle_rate: RobotConfig.throttle.odom
});
odomTopic.subscribe(function (msg) {
  if (mapMode !== "mapping") return;
  var p   = msg.pose.pose.position;
  var q   = msg.pose.pose.orientation;
  var yaw = Math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z));
  _robotYawRad        = yaw;           /* store for laser scan alignment */
  robotLayer.x        =  p.x;
  robotLayer.y        = -p.y;
  robotLayer.rotation = -yaw * (180 / Math.PI);

  /* Ensure robot is visible — odom arriving means SLAM is running */
  if (!robotLayer.visible && poseHasBeenSet) {
    robotLayer.visible = true;
    laserLayer.visible = true;
    /* First odom: center view on robot */
    if (typeof centerOnRobot === "function") centerOnRobot();
  }

  /* Update position HUD */
  if (typeof updateRobotPosHUD === "function") updateRobotPosHUD(p.x, p.y);

  /* Redraw robot + scan at new position */
  if (typeof redrawOverlay === "function") redrawOverlay();
  else stage.update();
});