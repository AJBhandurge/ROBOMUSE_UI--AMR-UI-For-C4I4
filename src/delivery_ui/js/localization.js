/* ==========================================================================
   localization.js — AMCL localization, 2D Pose Estimate, Goal Pose, Home
   Depends on: app.js (ros, SERVER_URL, viewer, stage, mapCanvas,
               robotLayer, laserLayer, poseArrowContainer, goalArrowContainer,
               poseHasBeenSet, poseEstimateMode, goalPoseMode, mapMode,
               canvasToRos, drawScan, showToast, lockMapping)
              mission.js (enableMissionButtons)
   ========================================================================== */


/* ---------- AMCL POSE SUBSCRIBER ---------- */

var amclPoseTopic = new ROSLIB.Topic({
  ros          : ros,
  name         : RobotConfig.topics.amcl_pose,
  messageType  : "geometry_msgs/PoseWithCovarianceStamped",
  compression  : "none",
  throttle_rate: RobotConfig.throttle.amcl
});
amclPoseTopic.subscribe(function (msg) {
  if (mapMode !== "localization") return;
  var p   = msg.pose.pose.position;
  var q   = msg.pose.pose.orientation;
  var yaw = Math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z));
  _robotYawRad        = yaw;           /* store for laser scan alignment */
  robotLayer.x        =  p.x;
  robotLayer.y        = -p.y;
  robotLayer.rotation = -yaw * (180 / Math.PI);

  /* AMCL gives better estimate — update robot, redraw */
  if (!poseHasBeenSet) {
    poseHasBeenSet     = true;
    robotLayer.visible = true;
    laserLayer.visible = true;
    if (typeof centerOnRobot === "function") setTimeout(centerOnRobot, 100);
  }

  /* Update position HUD and Get Location display */
  if (typeof updateRobotPosHUD === "function") updateRobotPosHUD(p.x, p.y);
  _updateGetLocHUD(p.x, p.y);

  /* Redraw robot + scan at new AMCL position */
  if (typeof redrawOverlay === "function") redrawOverlay();
  else stage.update();
});


/* ---------- /initialpose TOPIC ---------- */

var initialPoseTopic = new ROSLIB.Topic({
  ros         : ros,
  name        : RobotConfig.topics.initial_pose,
  messageType : "geometry_msgs/PoseWithCovarianceStamped",
  compression : "none"
});

function publishInitialPose(x, y, yaw) {
  var qz = Math.sin(yaw / 2);
  var qw = Math.cos(yaw / 2);
  initialPoseTopic.publish(new ROSLIB.Message({
    header: { frame_id: RobotConfig.frames.map },
    pose: {
      pose: {
        position:    { x: x, y: y, z: 0 },
        orientation: { x: 0, y: 0, z: qz, w: qw }
      },
      covariance: [
        0.25,0,0,0,0,0,  0,0.25,0,0,0,0,
        0,0,0,0,0,0,     0,0,0,0,0,0,
        0,0,0,0,0,0,     0,0,0,0,0,0.068
      ]
    }
  }));
}


/* ---------- START LOCALIZATION ---------- */

function startLocalization() {
  mapMode = "localization";
  resetPoseState();
  lockMapping();
  if (typeof resetMapFit === "function") resetMapFit();   /* auto-fit on new map */

  var map  = document.getElementById("mapSelect").value;
  var yaml = map.replace(".pgm", ".yaml");
  document.getElementById("mapImage").style.display = "none";
  document.getElementById("map").style.display      = "block";
  showToast("⏳ Starting localization...", "info");

  fetch(SERVER_URL + "/start_localization", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ map: yaml })
  })
    .then(r => r.json())
    .then(function () {
      /* Navigation stack launches in background (takes ~10s).
         Show info toast — user can set pose estimate while Nav2 warms up. */
      showToast("⏳ Localization starting — Nav2 warming up (~10s)...", "info");

      /* Auto-spawn robot at map origin (0, 0, 0) and publish to AMCL.
         User can refine with 2D Pose Estimate if needed. */
      robotLayer.x        = 0;
      robotLayer.y        = 0;
      robotLayer.rotation = 0;
      _robotYawRad        = 0;
      robotLayer.visible  = true;
      laserLayer.visible  = true;
      poseHasBeenSet      = true;

      /* Publish initial pose at (0,0,0) to seed AMCL */
      publishInitialPose(0, 0, 0);

      /* Update HUD */
      if (typeof updateRobotPosHUD === "function") updateRobotPosHUD(0, 0);

      /* Draw robot and center view on it */
      if (typeof redrawOverlay === "function") redrawOverlay();
      if (typeof centerOnRobot === "function") setTimeout(centerOnRobot, 50);

      /* Enable all buttons */
      var btnPose = document.getElementById("btn-pose");
      if (btnPose) { btnPose.disabled = false; btnPose.classList.add("btn-pose-ready"); }
      var btnGoal = document.getElementById("btn-goal");
      if (btnGoal) { btnGoal.disabled = false; btnGoal.classList.add("btn-goal-ready"); }
      var btnHome = document.getElementById("btn-home");
      if (btnHome) btnHome.disabled = false;
      var btnGetLoc = document.getElementById("btn-get-location");
      if (btnGetLoc) btnGetLoc.disabled = false;
      if (typeof enableMissionButtons === "function") enableMissionButtons();

      showToast("🤖 Robot placed at (0, 0) — adjust with 🎯 Pose Estimate if needed", "info");
    })
    .catch(() => showToast("⚠ Failed to start localization", "error"));
}


/* ---------- ENABLE 2D POSE ESTIMATE MODE ---------- */

function enablePoseEstimate() {
  if (mapMode !== "localization") { showToast("⚠ Start localization first", "error"); return; }
  poseEstimateMode = true;
  goalPoseMode     = false;
  freezeOdom       = true;
  document.getElementById("map").style.cursor          = "crosshair";
  document.getElementById("pose-banner").style.display = "flex";
  document.getElementById("pose-coords").textContent   = "Click on map to place robot…";
  showToast("🎯 Click on map and drag to set heading", "info");
}


/* ---------- ENABLE GOAL POSE MODE ---------- */

function enableCancelGoal() {
  var btn = document.getElementById("btn-cancel-goal");
  if (btn) btn.disabled = false;
}

/* ---------- GET CURRENT LOCATION AS WAYPOINT ---------- */

function _updateGetLocHUD(x, y) {
  var ex = document.getElementById("get-loc-x");
  var ey = document.getElementById("get-loc-y");
  if (ex) ex.textContent = x.toFixed(3);
  if (ey) ey.textContent = y.toFixed(3);
  var btn = document.getElementById("btn-get-location");
  if (btn && poseHasBeenSet) btn.disabled = false;
}

function getLocationAsWaypoint() {
  if (!poseHasBeenSet) {
    showToast("⚠ Set pose estimate first", "error");
    return;
  }
  /* Get current robot position from stored ROS coords */
  var rx  = robotLayer.x;          /* already ros_x (no FLIP_X needed) */
  var ry  = -robotLayer.y;         /* ros_y */
  var yaw = _robotYawRad || 0;

  /* Build waypoint name */
  var nameInput = document.getElementById("get-loc-name");
  var name = (nameInput && nameInput.value.trim()) ||
             ("WP" + (waypoints.length + 1) + " (" + rx.toFixed(1) + "," + ry.toFixed(1) + ")");

  /* Add to waypoints list */
  waypoints.push({ name: name, x: rx, y: ry, yaw: yaw });
  if (nameInput) nameInput.value = "";

  /* Refresh UI */
  if (typeof renderWpList          === "function") renderWpList();
  if (typeof redrawWaypointMarkers === "function") redrawWaypointMarkers();
  if (typeof saveMissionFile       === "function") saveMissionFile();

  showToast("📍 Added waypoint: " + name, "success");
}

function enableGoalPose() {
  if (!poseHasBeenSet) { showToast("⚠ Set pose estimate first", "error"); return; }
  goalPoseMode     = true;
  poseEstimateMode = false;
  document.getElementById("map").style.cursor          = "crosshair";
  document.getElementById("pose-banner").style.display = "flex";
  document.getElementById("pose-coords").textContent   = "Click on map to set navigation goal…";
  document.getElementById("btn-goal").classList.add("btn-goal-active");
  showToast("🟢 Click on map and drag to set goal heading", "info");
}


/* ---------- PUBLISH GOAL POSE (via Flask → Nav2) ---------- */

function publishGoalPose(x, y, yaw) {
  fetch(SERVER_URL + "/navigate_to_pose", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ x: x, y: y, yaw: yaw })
  })
    .then(r => r.json())
    .then(() => showToast(
      "🟢 Goal sent — X: " + x.toFixed(2) + "  Y: " + y.toFixed(2) +
      "  θ: " + (yaw * 180 / Math.PI).toFixed(1) + "°", "success"
    ))
    .catch(() => showToast("⚠ Failed to send goal", "error"));
}


/* ---------- GO HOME ---------- */

function goHome() {
  if (!poseHasBeenSet) { showToast("⚠ Start localization first", "error"); return; }
  fetch(SERVER_URL + "/navigate_to_pose", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ x: 0, y: 0, yaw: 0 })
  })
    .then(r => r.json())
    .then(() => showToast("🏠 Going home (0, 0)", "success"))
    .catch(() => showToast("⚠ Failed to send home goal", "error"));
}


/* ---------- MOUSE DRAG — POSE / GOAL / WAYPOINT ---------- */

var poseStart = null;
var dragging  = false;

/* mousedown */
mapCanvas.addEventListener("mousedown", function (e) {
  if (!poseEstimateMode && !goalPoseMode && !waypointMode) return;
  dragging  = true;
  poseStart = canvasToRos(e.clientX, e.clientY);

  if (waypointMode) {
    wpArrowContainer.x        =  poseStart.x;
    wpArrowContainer.y        =  poseStart.y;
    wpArrowContainer.rotation = 0;
    wpArrowContainer.visible  = true;
  } else if (poseEstimateMode) {
    poseArrowContainer.x        =  poseStart.x;
    poseArrowContainer.y        =  poseStart.y;
    poseArrowContainer.rotation = 0;
    poseArrowContainer.visible  = true;
    laserLayer.visible = true;
  } else {
    goalArrowContainer.x        =  poseStart.x;
    goalArrowContainer.y        =  poseStart.y;
    goalArrowContainer.rotation = 0;
    goalArrowContainer.visible  = true;
  }

  stage.update();
  document.getElementById("pose-coords").textContent =
    "X: " + poseStart.x.toFixed(2) + "  Y: " + poseStart.y.toFixed(2) + "  — drag to set heading";
});

/* mousemove */
mapCanvas.addEventListener("mousemove", function (e) {
  if (!dragging) return;
  var rosPos = canvasToRos(e.clientX, e.clientY);
  /* yaw from ROS-space delta — scaleX<0 in scene already handles flip */
  var yaw    = Math.atan2(rosPos.y - poseStart.y, rosPos.x - poseStart.x);

  /* EaselJS rotation formula (proven for scaleX>0 or scaleX<0, scaleY always <0):
     rotation = +yaw_deg  puts arrow tip in correct screen direction.
     scene.scaleY<0 means +sin(R) → downward, so +yaw makes +Y face upward ✓ */
  var rotDeg = yaw * (180 / Math.PI);
  if (waypointMode) {
    wpArrowContainer.rotation = rotDeg;
  } else if (poseEstimateMode) {
    poseArrowContainer.rotation = rotDeg;
    /* Preview robot AND scan at the estimated pose with the dragged heading.
     * Without this, the robot stays at its old AMCL position/yaw while the
     * arrow shows the new heading → they appear to rotate in opposite directions. */
    if (typeof drawScanWithRobot === "function") {
      drawScanWithRobot(
        (typeof lastScan !== "undefined" ? lastScan : null),
        poseStart.x, poseStart.y, yaw
      );
    } else if (typeof lastScan !== "undefined" && lastScan) {
      drawScan(lastScan, poseStart.x, poseStart.y, yaw);
    }
  } else if (goalPoseMode) {
    goalArrowContainer.rotation = rotDeg;
  }

  stage.update();
  document.getElementById("pose-coords").textContent =
    "X: " + poseStart.x.toFixed(2) +
    "  Y: " + poseStart.y.toFixed(2) +
    "  θ: " + (yaw * 180 / Math.PI).toFixed(1) + "°";
});

/* mouseup */
mapCanvas.addEventListener("mouseup", function (e) {
  if (!dragging) return;
  dragging = false;

  var rosPos = canvasToRos(e.clientX, e.clientY);
  /* yaw from ROS-space delta — scaleX<0 naturally gives correct heading */
  var yaw    = Math.atan2(rosPos.y - poseStart.y, rosPos.x - poseStart.x);

  document.getElementById("pose-banner").style.display = "none";
  document.getElementById("map").style.cursor          = "default";

  /* --- WAYPOINT branch --- */
  if (waypointMode) {
    waypointMode             = false;
    wpArrowContainer.visible = false;

    var name = document.getElementById("wp-name").value.trim() ||
               ("Waypoint " + (waypoints.length + 1));
    waypoints.push({ name: name, x: poseStart.x, y: poseStart.y, yaw: yaw });
    document.getElementById("wp-name").value = "";
    document.getElementById("btn-add-wp").classList.remove("btn-wp-active");
    renderWpList();
    redrawWaypointMarkers();
    saveMissionFile();
    showToast("📍 Added: " + name, "success");

  /* --- POSE ESTIMATE branch --- */
  } else if (poseEstimateMode) {
    poseEstimateMode           = false;
    poseArrowContainer.visible = false;
    document.getElementById("btn-pose").classList.remove("btn-pose-ready");

    publishInitialPose(poseStart.x, poseStart.y, yaw);

    robotLayer.x        =  poseStart.x;
    robotLayer.y        = -poseStart.y;
    robotLayer.rotation = -yaw * (180 / Math.PI);
    _robotYawRad        = yaw;
    robotLayer.visible  = true;
    laserLayer.visible  = true;
    poseHasBeenSet      = true;

    if (typeof updateRobotPosHUD === "function") updateRobotPosHUD(poseStart.x, poseStart.y);
    if (typeof centerOnRobot === "function") centerOnRobot();
    if (typeof redrawOverlay === "function") setTimeout(redrawOverlay, 50);

    var btnGoal = document.getElementById("btn-goal");
    btnGoal.disabled = false;
    btnGoal.classList.add("btn-goal-ready");
    document.getElementById("btn-home").disabled = false;
    enableMissionButtons();

    showToast(
      "📍 Pose set — X: " + poseStart.x.toFixed(2) +
      "  Y: " + poseStart.y.toFixed(2) +
      "  θ: " + (yaw * 180 / Math.PI).toFixed(1) + "°", "success"
    );

  /* --- GOAL POSE branch --- */
  } else if (goalPoseMode) {
    goalPoseMode               = false;
    goalArrowContainer.visible = false;
    document.getElementById("btn-goal").classList.remove("btn-goal-active");
    publishGoalPose(poseStart.x, poseStart.y, yaw);
    enableCancelGoal();
    showToast("🟢 Navigating to goal…", "info");
  }

  stage.update();
});


/* ---------- NAV PATH VISUALIZATION ---------- */

/* ── NAV PATH — drawn on a DOM overlay canvas for pixel-accurate line width ── */
var pathVisible  = true;
var _lastPath    = null;   /* cached for redraw on zoom/pan */

var _pathCanvas = (function() {
  var c = document.createElement("canvas");
  c.style.cssText = "position:absolute;top:0;left:0;pointer-events:none;z-index:3;";
  setTimeout(function() {
    var mc = document.getElementById("map");
    if (mc) mc.appendChild(c);
  }, 200);
  return c;
})();
var _pathCtx = _pathCanvas.getContext("2d");

function _syncPathCanvas() {
  var sc = typeof stage !== "undefined" ? stage.canvas : null;
  if (!sc) return;
  if (_pathCanvas.width  !== sc.width)  _pathCanvas.width  = sc.width;
  if (_pathCanvas.height !== sc.height) _pathCanvas.height = sc.height;
}

function _drawPathOnCanvas(poses) {
  _syncPathCanvas();
  _pathCtx.clearRect(0, 0, _pathCanvas.width, _pathCanvas.height);
  if (!pathVisible || !poses || poses.length < 2) return;
  if (Math.abs(viewer.scene.scaleX) < 2) return;   /* map not loaded yet */

  _pathCtx.strokeStyle = "#22c55e";   /* green like RViz planned path */
  _pathCtx.lineWidth   = 2;           /* thinner — cleaner look */
  _pathCtx.lineJoin    = "round";
  _pathCtx.lineCap     = "round";
  _pathCtx.setLineDash([]);

  var s = viewer.scene;
  _pathCtx.beginPath();
  var first = poses[0].pose.position;
  var fp    = { px: s.x + first.x * s.scaleX, py: s.y + first.y * s.scaleY };
  _pathCtx.moveTo(fp.px, fp.py);
  for (var i = 1; i < poses.length; i++) {
    var pt = poses[i].pose.position;
    _pathCtx.lineTo(s.x + pt.x * s.scaleX, s.y + pt.y * s.scaleY);
  }
  _pathCtx.stroke();
}

function redrawPath() {
  if (_lastPath) _drawPathOnCanvas(_lastPath);
  else { _syncPathCanvas(); _pathCtx.clearRect(0, 0, _pathCanvas.width, _pathCanvas.height); }
}

var planTopic = new ROSLIB.Topic({
  ros          : ros,
  name         : RobotConfig.topics.nav_plan,
  messageType  : "nav_msgs/Path",
  compression  : "none",
  throttle_rate: RobotConfig.throttle.plan
});
planTopic.subscribe(function (msg) {
  _lastPath = msg.poses;
  _drawPathOnCanvas(_lastPath);
});

function togglePath(on) {
  pathVisible = on;
  _pathCanvas.style.display = on ? "block" : "none";
  if (!on) {
    _pathCtx.clearRect(0, 0, _pathCanvas.width, _pathCanvas.height);
  } else if (_lastPath) {
    _drawPathOnCanvas(_lastPath);
  }
}


/* ---------- GLOBAL COSTMAP OVERLAY ---------- */
/* Uses a DOM canvas overlay (same approach as scan) — no EaselJS path bugs */

var costmapVisible = false;
var costmapClient  = null;

var _costmapCanvas = (function() {
  var c = document.createElement("canvas");
  c.style.cssText = "position:absolute;top:0;left:0;pointer-events:none;z-index:4;";
  setTimeout(function() {
    var mc = document.getElementById("map");
    if (mc) mc.appendChild(c);
  }, 150);
  return c;
})();
var _costmapCtx = _costmapCanvas.getContext("2d");

function _syncCostmapCanvas() {
  var sc = stage.canvas;
  if (_costmapCanvas.width  !== sc.width)  _costmapCanvas.width  = sc.width;
  if (_costmapCanvas.height !== sc.height) _costmapCanvas.height = sc.height;
}

function clearCostmap() {
  _costmapCtx.clearRect(0, 0, _costmapCanvas.width, _costmapCanvas.height);
  _lastCostmapGrid = null;
}

function toggleCostmap(on) {
  costmapVisible = on;
  if (on) {
    _costmapCanvas.style.display = "block";
    if (!costmapClient) {
      costmapClient = new ROSLIB.Topic({
        ros: ros, name: RobotConfig.topics.global_costmap,
        messageType: "nav_msgs/OccupancyGrid",
        throttle_rate: RobotConfig.throttle.costmap
      });
      costmapClient.subscribe(function (grid) {
        _lastCostmapGrid = grid;   /* cache for redraw on zoom/pan */
        if (!costmapVisible) return;
        drawCostmap(grid);
      });
    }
  } else {
    _costmapCanvas.style.display = "none";
    clearCostmap();
  }
}

function drawCostmap(grid) {
  _syncCostmapCanvas();
  _costmapCtx.clearRect(0, 0, _costmapCanvas.width, _costmapCanvas.height);
  if (Math.abs(viewer.scene.scaleX) < 2) return;   /* map not loaded yet */

  var info  = grid.info;
  var res   = info.resolution;
  var ox    = info.origin.position.x;
  var oy    = info.origin.position.y;
  var w     = info.width;
  var data  = grid.data;
  var scene = viewer.scene;
  /* pixel size of one costmap cell */
  var cellPx = Math.max(1, Math.ceil(Math.abs(res * scene.scaleX)));  /* ceil = sharp, no gaps */

  for (var i = 0; i < data.length; i++) {
    var cost = data[i];
    /* Skip free (0) and unknown (-1 / 255) */
    if (cost <= 0 || cost === 255) continue;

    var col = i % w;
    var row = Math.floor(i / w);
    var wx  = ox + (col + 0.5) * res;
    var wy  = oy + (row + 0.5) * res;
    var px  = scene.x + wx * scene.scaleX;
    var py  = scene.y + wy * scene.scaleY;
    if (px < -cellPx || px > _costmapCanvas.width  + cellPx) continue;
    if (py < -cellPx || py > _costmapCanvas.height + cellPx) continue;

    /* RViz-style costmap colours (soft, matching reference image):
       254 = lethal obstacle   → white (sharp, like RViz obstacle outline)
       253 = inscribed radius  → magenta (bright, hard boundary)
       128-252 = near-lethal   → red→orange (fading with cost)
       1-127   = inflation     → cyan/teal gradient (like RViz global costmap)
       Low inflation (<30)     → very light, almost transparent */
    /* Gray-scale costmap scheme:
       254 = lethal      → near-black (dark gray, sharply visible)
       253 = inscribed   → dark gray
       128-252 = near-lethal → medium dark gray gradient
       1-127   = inflation   → light gray gradient (barely visible at low cost)
       Inspired by RViz "map" color scheme — neutral grays, no colors */
    if (cost === 254) {
      _costmapCtx.fillStyle = "rgba(50,50,50,0.92)";      /* lethal: dark gray */
    } else if (cost === 253) {
      _costmapCtx.fillStyle = "rgba(80,80,80,0.85)";      /* inscribed: dark gray */
    } else if (cost >= 128) {
      /* near-lethal: medium gray, darker with higher cost */
      var t1  = (cost - 128) / 124;
      var gv1 = Math.round(140 - t1 * 60);                /* 140 → 80 */
      var a1  = 0.50 + t1 * 0.30;
      _costmapCtx.fillStyle = "rgba(" + gv1 + "," + gv1 + "," + gv1 + "," + a1.toFixed(2) + ")";
    } else if (cost >= 20) {
      /* inflation: light gray, fading out at lower cost */
      var t2  = (cost - 20) / 107;
      var gv2 = Math.round(180 + t2 * 0);                 /* constant light gray 180 */
      var a2  = 0.10 + t2 * 0.30;                         /* 0.10 → 0.40 opacity */
      _costmapCtx.fillStyle = "rgba(" + gv2 + "," + gv2 + "," + gv2 + "," + a2.toFixed(2) + ")";
    } else {
      _costmapCtx.fillStyle = "rgba(200,200,200,0.06)";
    }
    _costmapCtx.fillRect(px - cellPx/2, py - cellPx/2, cellPx, cellPx);
  }
}