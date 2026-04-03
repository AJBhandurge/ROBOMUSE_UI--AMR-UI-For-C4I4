/* ==========================================================================
   app.js — Main entry point
   Initialises ROS, viewer, shared layers, resize, toast.
   All feature modules (mapping, localization, teleop…) are loaded AFTER
   this file via <script> tags in index.html.
   ========================================================================== */


/* ---------- HOST / URL GLOBALS ---------- */

var SERVER_HOST = window.location.hostname || "localhost";
var SERVER_URL  = "http://" + SERVER_HOST + ":5000";
var ROS_WS_URL  = "ws://"  + SERVER_HOST + ":9090";

/* Camera feeds set by login controller in index.html */


/* ---------- ROS CONNECTION ---------- */

var ros    = new ROSLIB.Ros({ url: ROS_WS_URL });
var cmdVel = null;

ros.on("connection", function () {
  document.getElementById("ros-status").textContent = "● CONNECTED";
  document.getElementById("ros-status").style.color = "#22c55e";
  cmdVel = new ROSLIB.Topic({
    ros: ros, name: RobotConfig.topics.cmd_vel,
    messageType: "geometry_msgs/Twist"
  });
});
ros.on("error", function () {
  document.getElementById("ros-status").textContent = "● ERROR";
  document.getElementById("ros-status").style.color = "#ef4444";
});
ros.on("close", function () {
  document.getElementById("ros-status").textContent = "● DISCONNECTED";
  document.getElementById("ros-status").style.color = "#f59e0b";
});


/* ---------- SHARED STATE ---------- */

var mapMode          = "idle";       // "idle" | "mapping" | "localization"
var poseEstimateMode = false;        // user is placing 2D pose estimate
var goalPoseMode     = false;        // user is placing nav goal
var poseHasBeenSet   = false;        // robot/scan hidden until true
var freezeOdom       = true;         // pause odom during drag

/* ── MAP X-FLIP ──────────────────────────────────────────────────────────────
 * When flip_map_x = true, the entire display is mirrored around the Y axis:
 *   • scene.scaleX is set NEGATIVE so +X goes leftward on screen
 *   • canvasToRos negates X  (canvas left = ROS +X)
 *   • robot, scan, costmap, arrows all use the same negated X
 * This matches RViz orientation when your map appears left/right mirrored.
 * Change RobotConfig.display.flip_map_x in config/robot_config.js.
 * ─────────────────────────────────────────────────────────────────────────── */
var FLIP_X = (RobotConfig.display && RobotConfig.display.flip_map_x) ? -1 : 1;
/* FLIP_X = -1 → mirror X;  FLIP_X = +1 → normal */

var lastScan     = null;             // cached LaserScan message
var _robotYawRad = 0;                // true ROS yaw in radians


/* ---------- VIEWER ---------- */

var mapDiv       = document.getElementById("map");
var mapContainer = document.getElementById("mapContainer");

function getMapSize() {
  return {
    w: mapContainer.clientWidth  || 800,
    h: mapContainer.clientHeight || 500
  };
}

var sz         = getMapSize();
var viewer     = new ROS2D.Viewer({ divID: "map", width: sz.w, height: sz.h });
var rootObject = viewer.scene;
var stage      = viewer.scene.getStage();
var mapCanvas  = stage.canvas;
stage.canvas.style.background = "#282c34";   /* dark bg — shows before map loads */


/* ---------- RESIZE VIEWER ---------- */

function resizeViewer() {
  var s = getMapSize();
  if (s.w < 10 || s.h < 10) return;   /* container not visible yet */
  stage.canvas.width  = s.w;
  stage.canvas.height = s.h;
  /* Force canvas CSS size to match attribute size (no stretching) */
  stage.canvas.style.width  = s.w + "px";
  stage.canvas.style.height = s.h + "px";
  _syncScanCanvas();
  if (gridClient &&
      gridClient.currentGrid &&
      gridClient.currentGrid.pose &&
      gridClient.currentGrid.pose.position) {
    viewer.scaleToDimensions(
      gridClient.currentGrid.width,
      gridClient.currentGrid.height
    );
    viewer.shift(
      gridClient.currentGrid.pose.position.x,
      gridClient.currentGrid.pose.position.y
    );
    _applyFlipX();   /* restore X-mirror after re-fit */
  }
  if (typeof redrawPath === "function") redrawPath();
  stage.update();
}

window.addEventListener("resize", function() {
  resizeViewer();
  redrawOverlay();
});
if (window.ResizeObserver) {
  new ResizeObserver(resizeViewer).observe(mapContainer);
}


/* ---------- MOUSE WHEEL ZOOM ---------- */

mapContainer.addEventListener("wheel", function (e) {
  e.preventDefault();
  var factor = e.deltaY < 0 ? 1.1 : 0.91;
  var rect   = stage.canvas.getBoundingClientRect();
  var mouseX = (e.clientX - rect.left) * (stage.canvas.width  / rect.width);
  var mouseY = (e.clientY - rect.top)  * (stage.canvas.height / rect.height);
  var scene  = viewer.scene;
  scene.x    = mouseX + (scene.x - mouseX) * factor;
  scene.y    = mouseY + (scene.y - mouseY) * factor;
  scene.scaleX *= factor;
  scene.scaleY *= factor;
  /* Redraw robot+scan at new zoom */
  redrawOverlay();
}, { passive: false });

/* ── MAP PAN (mouse drag) ─────────────────────────────────────────────── */
var _isDragging   = false;
var _dragStartX   = 0;
var _dragStartY   = 0;
var _sceneDragX   = 0;
var _sceneDragY   = 0;

mapContainer.addEventListener("mousedown", function (e) {
  if (e.button !== 0) return;
  /* Don't hijack clicks when in pose/goal mode */
  if (typeof poseEstimateMode !== "undefined" && (poseEstimateMode || goalPoseMode || waypointMode)) return;
  _isDragging  = true;
  _dragStartX  = e.clientX;
  _dragStartY  = e.clientY;
  _sceneDragX  = viewer.scene.x;
  _sceneDragY  = viewer.scene.y;
  mapContainer.style.cursor = "grabbing";
});

window.addEventListener("mousemove", function (e) {
  if (!_isDragging) return;
  var dx = e.clientX - _dragStartX;
  var dy = e.clientY - _dragStartY;
  viewer.scene.x = _sceneDragX + dx;
  viewer.scene.y = _sceneDragY + dy;
  /* Redraw robot+scan at new pan */
  redrawOverlay();
});

window.addEventListener("mouseup", function (e) {
  if (!_isDragging) return;
  _isDragging = false;
  mapContainer.style.cursor = "";
});


/* ---------- MAP CLIENT (OccupancyGrid) ---------- */

/*
 * compression: "png" — rosbridge encodes the data array as a PNG image
 * which avoids the broken JSON serialization of large int8 arrays
 * that produces empty commas like [-1,-1,,1,-1] in the output.
 */
/* ── HTTP-polled map renderer (bypasses WebSocket fragmentation) ── */
/* ── MAP RENDERING ─────────────────────────────────────────────────────────
 * Two-step polling:
 *  1. /map_meta  (1 Hz, ~200 bytes) — metadata + seq number
 *  2. /map_tile  (on seq change)    — PNG image of the map
 *
 * This avoids sending 250k integers per poll (was causing "stuck" map).
 * The PNG is decoded by the browser and placed as an EaselJS Bitmap.
 * ──────────────────────────────────────────────────────────────────────── */

var gridClient  = { currentGrid: null };   /* stub so resizeViewer works */
var mapBitmap   = null;
var _mapFitted  = false;
var _mapPollTimer = null;
var _lastMapSeq   = -1;
var _lastMeta     = null;   /* last received map_meta */

function resetMapFit() {
  _mapFitted  = false;
  _lastMapSeq = -1;   /* force re-fetch even if seq happens to match old value */
  if (mapBitmap) { rootObject.removeChild(mapBitmap); mapBitmap = null; }
  stage.update();
}

function _applyMapMeta(meta) {
  /* Update gridClient stub so resizeViewer / fitMap work */
  var w   = meta.width;
  var h   = meta.height;
  var res = meta.resolution;
  var ox  = meta.origin.x;
  var oy  = meta.origin.y;
  gridClient.currentGrid = {
    width:  w * res,
    height: h * res,
    pose:   { position: { x: ox, y: oy } }
  };
  _lastMeta = meta;
}

function _placeBitmap(img, meta) {
  var w   = meta.width;
  var h   = meta.height;
  var res = meta.resolution;
  var ox  = meta.origin.x;
  var oy  = meta.origin.y;

  if (mapBitmap) rootObject.removeChild(mapBitmap);
  var bmp    = new createjs.Bitmap(img);
  bmp.x      = ox;
  bmp.y      = oy;
  bmp.scaleX = res;
  bmp.scaleY = res;
  rootObject.addChildAt(bmp, 0);
  mapBitmap  = bmp;
  _applyMapMeta(meta);
  stage.canvas.style.background = "#1a1a2e";

  if (!_mapFitted) {
    _mapFitted = true;
    viewer.scaleToDimensions(w * res, h * res);
    viewer.shift(ox, oy);
    /* Apply X-flip AFTER scaleToDimensions sets scaleX to positive value.
     * Negating scaleX mirrors the entire scene: +X goes leftward on screen.
     * This makes robotLayer, arrows, scan all auto-flip consistently. */
    _applyFlipX();
    /* After first fit: redraw, then center on robot if visible */
    setTimeout(function() {
      if (typeof redrawOverlay === "function") redrawOverlay();
      if (poseHasBeenSet && typeof centerOnRobot === "function") {
        setTimeout(centerOnRobot, 50);
      }
    }, 80);
  } else {
    /* Map content updated — keep zoom/pan, just redraw overlay */
    if (typeof redrawOverlay === "function") redrawOverlay();
  }
  stage.update();
}

function _fetchMapTile(meta) {
  var img    = new Image();
  img.onload = function() { _placeBitmap(img, meta); };
  img.onerror = function() { console.warn("[map] tile load failed"); };
  img.src = SERVER_URL + "/map_tile?t=" + Date.now();   /* cache-bust */
}

function startMapPolling() {
  if (_mapPollTimer) return;
  _mapPollTimer = setInterval(function() {
    fetch(SERVER_URL + "/map_meta")
      .then(function(r) { return r.status === 204 ? null : r.json(); })
      .then(function(meta) {
        if (!meta || meta.status === "no_map") return;
        if (meta.seq !== _lastMapSeq || mapBitmap === null) {
          _lastMapSeq = meta.seq;
          _fetchMapTile(meta);
        }
      })
      .catch(function() {});
  }, RobotConfig.throttle.map_poll);
}

function stopMapPolling() {
  if (_mapPollTimer) { clearInterval(_mapPollTimer); _mapPollTimer = null; }
}

/* ── MAP VIEWPORT CONTROLS ──────────────────────────────────────────────── */

function _applyFlipX() {
  /* After scaleToDimensions: enforce uniform scale (same px/m for X and Y)
   * so the map looks like a true 2D orthographic view (like RViz).
   * Then optionally negate scaleX to mirror the X axis. */
  var s = viewer.scene;
  var g = gridClient.currentGrid;
  if (!g || !g.pose) return;

  var cw  = stage.canvas.width;
  var ch  = stage.canvas.height;
  var gw  = g.width;    /* map width  in metres */
  var gh  = g.height;   /* map height in metres */
  var ox  = g.pose.position.x;
  var oy  = g.pose.position.y;

  /* Uniform scale: fit the whole map in the canvas, same px/m both axes */
  var scaleAbs = Math.min(cw / gw, ch / gh) * 0.92;   /* 0.92 = 8% margin */

  /* Centre the map in the canvas */
  var mapPixW = gw * scaleAbs;
  var mapPixH = gh * scaleAbs;
  var offX    = (cw - mapPixW) / 2;
  var offY    = (ch - mapPixH) / 2;

  /* Scene placement formula (derived from: worldToPixel(ox,oy) = map bottom corner):
   *   canvas_px = scene.x + ox * scaleX = offX (or offX+mapPixW when flipped)
   *   canvas_py = scene.y + oy * scaleY = offY + mapPixH  (bottom of map = bottom of viewport)
   * Solving: scene.y = offY + mapPixH + oy*scaleAbs  (same for both flip cases)
   *          scene.x = offX - ox*scaleAbs            (no-flip)
   *          scene.x = offX + mapPixW + ox*scaleAbs  (flip: ROS left = screen right)
   */
  var sy_base = offY + mapPixH + oy * scaleAbs;

  if (FLIP_X < 0) {
    s.scaleX = -scaleAbs;
    s.scaleY = -scaleAbs;
    s.x      = offX + mapPixW + ox * scaleAbs;
    s.y      = sy_base;
  } else {
    s.scaleX =  scaleAbs;
    s.scaleY = -scaleAbs;
    s.x      = offX - ox * scaleAbs;
    s.y      = sy_base;
  }
}

function fitMap() {
  if (!gridClient.currentGrid || !gridClient.currentGrid.pose) return;
  var g = gridClient.currentGrid;
  viewer.scaleToDimensions(g.width, g.height);
  viewer.shift(g.pose.position.x, g.pose.position.y);
  _applyFlipX();
  redrawOverlay();
  stage.update();
}

function zoomMapIn() {
  var scene = viewer.scene;
  var cx = stage.canvas.width  / 2;
  var cy = stage.canvas.height / 2;
  var f  = 1.25;
  scene.x    = cx + (scene.x - cx) * f;
  scene.y    = cy + (scene.y - cy) * f;
  scene.scaleX *= f;
  scene.scaleY *= f;
  stage.update();
  redrawOverlay();
}

function zoomMapOut() {
  var scene = viewer.scene;
  var cx = stage.canvas.width  / 2;
  var cy = stage.canvas.height / 2;
  var f  = 0.8;
  scene.x    = cx + (scene.x - cx) * f;
  scene.y    = cy + (scene.y - cy) * f;
  scene.scaleX *= f;
  scene.scaleY *= f;
  stage.update();
  redrawOverlay();
}

function centerOnRobot() {
  /* Pan so robot is centred in the canvas — keeps current zoom */
  if (!poseHasBeenSet) return;
  var scene = viewer.scene;
  var cx = stage.canvas.width  / 2;
  var cy = stage.canvas.height / 2;
  /* worldToPixel: px = scene.x + rx*scaleX, py = scene.y + ry*scaleY
     We want px=cx, py=cy, so:
       scene.x = cx - robotLayer.x * scaleX
       scene.y = cy - robotLayer.y * scaleY   (robotLayer.y is already -ros_y) */
  var rx = robotLayer.x;    /* ROS X in metres */
  var ry = -robotLayer.y;   /* ROS Y in metres (robotLayer.y = -rosY) */
  scene.x = cx - rx * scene.scaleX;
  scene.y = cy - ry * scene.scaleY;
  stage.update();
  redrawOverlay();
}

/* ── ROBOT POSITION HUD ─────────────────────────────────────────────────── */

function updateRobotPosHUD(x, y) {
  var ex = document.getElementById("robot-pos-x");
  var ey = document.getElementById("robot-pos-y");
  if (ex) ex.textContent = x.toFixed(2);
  if (ey) ey.textContent = y.toFixed(2);
}

/* Map polling starts ONLY when user clicks Start Mapping or Confirm Localization */
/* Do NOT auto-start polling on ROS connection */


/* ---------- SHARED CANVAS LAYERS (order = draw order) ---------- */

var laserLayer = new createjs.Container();   // red scan dots
laserLayer.visible = false; if(typeof clearScan==="function") clearScan();
rootObject.addChild(laserLayer);

var robotLayer = new createjs.Container();   // blue robot box
robotLayer.visible = false;
rootObject.addChild(robotLayer);

/* Robot is drawn on the scan overlay canvas (same coordinate system as scan dots).
 * This guarantees pixel-perfect alignment and a minimum visible size. */
function _drawRobot(ctx, px, py, yaw, scaleX, forceVisible) {
  if (!robotLayer.visible && !forceVisible) return;

  /* Scale with map zoom — robot stays proportional to map (like waypoint circles).
   * ROBOT_W/ROBOT_H are in metres. viewer.scene.scaleX gives px/m at current zoom.
   * Min clamp ensures robot is always at least visible at high zoom-out. */
  var zoom  = Math.abs(viewer.scene.scaleX) || 40;
  var scale = Math.max(zoom, 12);   /* at least 12 px/m so robot is always legible */
  var L  = ROBOT_W * scale * 1.15;  /* +15% so robot looks slightly bigger */
  var W  = ROBOT_H * scale * 1.15;
  var hl = L / 2;
  var hw = W / 2;

  ctx.save();
  ctx.translate(px, py);
  /* Universal canvas rotation: derived from worldToPixel direction vector.
   * canvasAngle = atan2(sin(yaw)*scaleY, cos(yaw)*scaleX)
   * Works correctly for any sign of scaleX (flipped or not). */
  var _s = viewer.scene;
  var canvasAngle = Math.atan2(Math.sin(yaw) * _s.scaleY, Math.cos(yaw) * _s.scaleX);
  ctx.rotate(canvasAngle);

  /* Body — RViz-style green robot rectangle */
  var bodyColor = RobotConfig.robot.color || "#009900";
  ctx.fillStyle   = bodyColor;
  ctx.strokeStyle = "#ffffff";
  ctx.lineWidth   = 2;
  ctx.beginPath();
  ctx.rect(-hl, -hw, L, W);
  ctx.fill();
  ctx.stroke();

  /* Forward direction indicator — white triangle pointing forward (+X) */
  ctx.fillStyle = "rgba(255,255,255,0.95)";
  ctx.beginPath();
  ctx.moveTo( hl * 0.7,  0);
  ctx.lineTo(-hl * 0.1, -hw * 0.5);
  ctx.lineTo(-hl * 0.1,  hw * 0.5);
  ctx.closePath();
  ctx.fill();

  /* Front edge highlight — bright white line like RViz */
  ctx.strokeStyle = "#ffffff";
  ctx.lineWidth   = 3;
  ctx.beginPath();
  ctx.moveTo(hl, -hw);
  ctx.lineTo(hl,  hw);
  ctx.stroke();

  ctx.restore();
}

var poseArrowContainer = new createjs.Container();   // red 2D-pose arrow
poseArrowContainer.visible = false;
rootObject.addChild(poseArrowContainer);

(function buildPoseArrow() {
  var shaft = new createjs.Shape();
  shaft.graphics.beginFill("#dc2626").drawRect(0, -0.06, 0.65, 0.12);
  var head = new createjs.Shape();
  head.graphics.beginFill("#dc2626")
    .moveTo(1.0, 0).lineTo(0.65, -0.22).lineTo(0.65, 0.22).closePath();
  poseArrowContainer.addChild(shaft);
  poseArrowContainer.addChild(head);
})();

var goalArrowContainer = new createjs.Container();   // green goal arrow
goalArrowContainer.visible = false;
rootObject.addChild(goalArrowContainer);

(function buildGoalArrow() {
  var shaft = new createjs.Shape();
  shaft.graphics.beginFill("#16a34a").drawRect(0, -0.06, 0.65, 0.12);
  var head = new createjs.Shape();
  head.graphics.beginFill("#16a34a")
    .moveTo(1.0, 0).lineTo(0.65, -0.22).lineTo(0.65, 0.22).closePath();
  goalArrowContainer.addChild(shaft);
  goalArrowContainer.addChild(head);
})();

var waypointLayer = new createjs.Container();        // orange waypoint markers
rootObject.addChild(waypointLayer);


/* ---------- CANVAS → ROS COORDINATES ---------- */

function canvasToRos(clientX, clientY) {
  /* Convert canvas click → ROS world coordinates.
   *
   * scene.scaleX is NEGATIVE when FLIP_X=-1 (map mirror enabled).
   * localX = (cx - scene.x) / scene.scaleX  already gives correct ROS x:
   *   Example: scene.x=400, scaleX=-40, click at canvas_px=280 (left of center)
   *   localX = (280-400)/(-40) = +3  ← positive ROS x (robot is at +3 on flipped map) ✓
   * NO additional FLIP_X multiplication needed — scaleX handles it.
   *
   * scene.scaleY is always NEGATIVE (ROS2D Y-flip):
   *   localY = (cy-scene.y)/scaleY  gives correct ROS y (negative scaleY flips Y).
   */
  var rect         = mapCanvas.getBoundingClientRect();
  var px           = clientX - rect.left;
  var py           = clientY - rect.top;
  var scaleFactorX = mapCanvas.width  / rect.width;
  var scaleFactorY = mapCanvas.height / rect.height;
  var cx           = px * scaleFactorX;
  var cy           = py * scaleFactorY;
  var scene        = viewer.scene;
  var localX       = (cx - scene.x) / scene.scaleX;   /* correct ROS x */
  var localY       = (cy - scene.y) / scene.scaleY;   /* correct ROS y */
  return { x: localX, y: localY };
}


/* ---------- DRAW SCAN DOTS ---------- */

/* ── DUAL-LIDAR SCAN DRAWING ────────────────────────────────────────────────
 *
 * Your robot has two LiDARs merged into /merged_laser (base_link frame):
 *   Front: xyz="0.330  0.227  0.178"  rpy="0 0 π"   (pointing backward)
 *   Back:  xyz="-0.330 -0.227 0.178"  rpy="0 0 0"   (pointing forward)
 *
 * Since /merged_laser is already in base_link frame (ira_laser_tools or
 * similar does the TF transform), we just need to draw from robot pose.
 * All offsets = 0.  If your merge node does NOT apply TF (raw concatenation),
 * set SCAN_USE_RAW_CONCAT = true and the two lidar origins will be applied.
 * ────────────────────────────────────────────────────────────────────────── */

var SCAN_USE_RAW_CONCAT = false;   /* false = merged scan in base_link frame (normal) */
                                    /* true  = raw concat without TF (uncommon)       */

/* Front lidar mounting (from URDF) */
var LIDAR_FRONT_X   =  0.330;
var LIDAR_FRONT_Y   =  0.227;
var LIDAR_FRONT_YAW =  Math.PI;    /* 180° — sensor points backward, scan angles wrap */

/* Back lidar mounting (from URDF) */
var LIDAR_BACK_X    = -0.330;
var LIDAR_BACK_Y    = -0.227;
var LIDAR_BACK_YAW  =  0.0;

/* Robot body dimensions from URDF (0.806 x 0.600 m) */
var ROBOT_W = 0.806;
var ROBOT_H = 0.600;


/* ── LASER SCAN RENDERER ────────────────────────────────────────────────────
 *
 * Architecture: separate DOM <canvas> overlay on top of the EaselJS canvas.
 * This completely avoids EaselJS coordinate/transform issues.
 *
 * Two LiDARs merged by ira_laser_tools into /merged_laser (frame: base_link):
 *   Front: xyz=( 0.330,  0.227, 0.178)  rpy="0 0 π"  → already TF-corrected
 *   Back:  xyz=(-0.330, -0.227, 0.178)  rpy="0 0 0"  → already TF-corrected
 *
 * Since ira_laser_tools applies full TF, all points in merged_laser ARE in
 * base_link frame → transform to map frame = rotate by robot yaw + translate
 * by robot position. No per-lidar offsets needed (SCAN_USE_RAW_CONCAT=false).
 *
 * Render pipeline:
 *   1. collectHits()  → array of (x,y) in ROS map frame (metres)
 *   2. world→pixel    → using viewer.scene transform (scaleX, scaleY, x, y)
 *   3. ctx.arc()      → draw each dot independently (no path connections)
 * ──────────────────────────────────────────────────────────────────────── */

/* ── LASER SCAN RENDERER (RViz-style) ──────────────────────────────────────
 *
 * Renders on a DOM <canvas> overlay (z-index 5) placed above the EaselJS
 * canvas.  Uses worldToPixel() with scene.scaleY (negative in ROS2D) for
 * correct Y-flip without any bitmap transform hacks.
 *
 * Visual style matches RViz LaserScan display:
 *   • Flat square pixels (fillRect), not circles
 *   • Size: 2×2 px at normal zoom, scales slightly with zoom
 *   • Colour: bright white (#ffffff) with slight transparency  
 *   • Max range cap to avoid stray infinity readings
 *
 * Coordinate pipeline (SCAN_USE_RAW_CONCAT = false, normal mode):
 *   scan point (laser frame, metres)
 *     → rotate by robotYaw, translate by robotPos
 *     → ROS map frame (metres)
 *     → worldToPixel()
 *     → screen pixels on overlay canvas
 * ──────────────────────────────────────────────────────────────────────── */

var SCAN_MAX_RANGE  = 20.0;   /* metres — cap stray inf readings        */
var SCAN_DOT_SIZE   = 2;      /* pixels per dot side at reference zoom  */
var SCAN_COLOR      = "#ef4444";  /* red — matches RViz default laser color */

/* ── overlay canvas ─────────────────────────────────────────────────── */
var _scanCanvas = (function() {
  var c = document.createElement("canvas");
  c.id = "scan-overlay";
  c.style.cssText = "position:absolute;top:0;left:0;pointer-events:none;z-index:5;";
  setTimeout(function() {
    /* Attach to #map div — always inside mapContainer regardless of DOM position */
    var m = document.getElementById("map");
    if (m) { m.style.position = "relative"; m.appendChild(c); }
  }, 50);
  return c;
})();
var _scanCtx = _scanCanvas.getContext("2d");

function _syncScanCanvas() {
  var sc  = stage.canvas;
  var sw  = sc.width  || parseInt(sc.style.width)  || 800;
  var sh  = sc.height || parseInt(sc.style.height) || 500;
  if (_scanCanvas.width  !== sw) _scanCanvas.width  = sw;
  if (_scanCanvas.height !== sh) _scanCanvas.height = sh;
  /* CSS size must match attribute size — no browser scaling */
  _scanCanvas.style.width  = sw + "px";
  _scanCanvas.style.height = sh + "px";
}

/* ROS world (metres) → overlay canvas pixel.
 * scene.scaleX is negated when FLIP_X=-1 (done in _placeBitmap after fit),
 * so +wx automatically goes leftward on screen. worldToPixel uses raw scene. */
function worldToPixel(wx, wy) {
  var s = viewer.scene;
  return {
    px: s.x + wx * s.scaleX,
    py: s.y + wy * s.scaleY
  };
}

/* Collect hit points in ROS map frame from one scan origin */
function collectHits(ranges, angleMin, angleInc, rangeMin, rangeMax,
                     originX, originY, scanYaw) {
  var maxR  = Math.min(rangeMax, SCAN_MAX_RANGE);
  var cosT  = Math.cos(scanYaw);
  var sinT  = Math.sin(scanYaw);
  var hits  = [];
  var angle = angleMin;
  for (var i = 0; i < ranges.length; i++) {
    var r = ranges[i];
    if (r > rangeMin && r < maxR && isFinite(r)) {
      var lx = r * Math.cos(angle);
      var ly = r * Math.sin(angle);
      hits.push(originX + lx * cosT - ly * sinT);   /* map X */
      hits.push(originY + lx * sinT + ly * cosT);   /* map Y */
    }
    angle += angleInc;
  }
  return hits;
}

function drawScan(scan, robotX, robotY, robotYaw) {
  _syncScanCanvas();
  _scanCtx.clearRect(0, 0, _scanCanvas.width, _scanCanvas.height);
  if (!scan || !scan.ranges) return;

  /* Guard: don't draw until map is loaded and scene is properly scaled */
  var scaleX = viewer.scene.scaleX;
  if (!scaleX || Math.abs(scaleX) < 2) return;

  var aMin = scan.angle_min;
  var aInc = scan.angle_increment;
  var rMin = scan.range_min;
  var rMax = scan.range_max;
  var cosR = Math.cos(robotYaw);
  var sinR = Math.sin(robotYaw);
  var hits = [];

  if (SCAN_USE_RAW_CONCAT) {
    /* Raw concat mode: ira_laser_tools NOT applying TF — apply manually */
    var mid = Math.floor(scan.ranges.length / 2);
    var fx = robotX + LIDAR_FRONT_X * cosR - LIDAR_FRONT_Y * sinR;
    var fy = robotY + LIDAR_FRONT_X * sinR + LIDAR_FRONT_Y * cosR;
    hits = hits.concat(collectHits(
      scan.ranges.slice(0, mid), aMin, aInc, rMin, rMax,
      fx, fy, robotYaw + LIDAR_FRONT_YAW));
    var bx = robotX + LIDAR_BACK_X * cosR - LIDAR_BACK_Y * sinR;
    var by = robotY + LIDAR_BACK_X * sinR + LIDAR_BACK_Y * cosR;
    hits = hits.concat(collectHits(
      scan.ranges.slice(mid), aMin, aInc, rMin, rMax,
      bx, by, robotYaw + LIDAR_BACK_YAW));
  } else {
    /* Normal: /merged_laser already in base_link frame via ira_laser_tools TF */
    hits = collectHits(scan.ranges, aMin, aInc, rMin, rMax,
                       robotX, robotY, robotYaw);
  }

  if (hits.length === 0) return;

  /* Dot size: fixed 2px at normal zoom, slightly larger when zoomed in */
  var zoom    = Math.abs(scaleX);
  var dotSize = Math.max(2, Math.min(4, SCAN_DOT_SIZE * zoom / 50));
  var half    = dotSize / 2;
  var cw      = _scanCanvas.width;
  var ch      = _scanCanvas.height;

  /* Scan dot colour — red like RViz LaserScan default */
  _scanCtx.fillStyle = SCAN_COLOR;

  for (var i = 0; i < hits.length; i += 2) {
    var p = worldToPixel(hits[i], hits[i + 1]);
    if (p.px < -half || p.px > cw + half) continue;
    if (p.py < -half || p.py > ch + half) continue;
    _scanCtx.fillRect(p.px - half, p.py - half, dotSize, dotSize);
  }

  /* Draw robot on same canvas — on top of scan dots */
  if (poseHasBeenSet && robotLayer.visible) {
    var rp = worldToPixel(robotX, robotY);
    _drawRobot(_scanCtx, rp.px, rp.py, robotYaw, viewer.scene.scaleX);
  }
}

/* Draw scan + robot preview at a given pose and yaw.
 * Used during pose estimate drag so robot body tracks the arrow direction. */
function drawScanWithRobot(scan, rosX, rosY, yaw) {
  _syncScanCanvas();
  _scanCtx.clearRect(0, 0, _scanCanvas.width, _scanCanvas.height);
  if (Math.abs(viewer.scene.scaleX) < 2) return;

  /* Draw scan dots at the preview pose */
  if (scan && scan.ranges) {
    var aMin = scan.angle_min, aInc = scan.angle_increment;
    var rMin = scan.range_min, rMax = scan.range_max;
    var hits = collectHits(scan.ranges, aMin, aInc, rMin, rMax, rosX, rosY, yaw);
    var dotSize = Math.max(2, SCAN_DOT_SIZE);
    var half = dotSize / 2;
    _scanCtx.fillStyle = SCAN_COLOR;
    for (var i = 0; i < hits.length; i += 2) {
      var p = worldToPixel(hits[i], hits[i + 1]);
      if (p.px < -half || p.px > _scanCanvas.width  + half) continue;
      if (p.py < -half || p.py > _scanCanvas.height + half) continue;
      _scanCtx.fillRect(p.px - half, p.py - half, dotSize, dotSize);
    }
  }

  /* Draw robot body at preview pose with preview yaw */
  var rp = worldToPixel(rosX, rosY);
  _drawRobot(_scanCtx, rp.px, rp.py, yaw, viewer.scene.scaleX);
}

/* Redraw overlay with latest robot+scan (called on zoom/pan/odom) */
var _lastCostmapGrid = null;   /* cache for redraw on zoom/pan */

function redrawOverlay() {
  if (!poseHasBeenSet) return;
  var yaw  = _robotYawRad || 0;
  var rosX = robotLayer.x;    /* robotLayer.x = ros_x directly (no FLIP_X needed) */
  var rosY = -robotLayer.y;   /* robotLayer.y = -ros_y always */

  if (lastScan && !poseEstimateMode) {
    drawScan(lastScan, rosX, rosY, yaw);
  } else {
    _syncScanCanvas();
    _scanCtx.clearRect(0, 0, _scanCanvas.width, _scanCanvas.height);
  }
  if (typeof drawCostmap === "function" && _lastCostmapGrid &&
      typeof costmapVisible !== "undefined" && costmapVisible) {
    drawCostmap(_lastCostmapGrid);
  }
  if (typeof redrawPath === "function") redrawPath();
}

function clearScan() {
  if (_scanCtx) _scanCtx.clearRect(0, 0, _scanCanvas.width, _scanCanvas.height);
}


/* ---------- LASER SCAN SUBSCRIBER ---------- */

var scanTopic = new ROSLIB.Topic({
  ros          : ros,
  name         : RobotConfig.topics.scan,
  messageType  : "sensor_msgs/LaserScan",
  compression  : "none",
  throttle_rate: RobotConfig.throttle.scan
});
scanTopic.subscribe(function (scan) {
  lastScan = scan;
  if (!poseHasBeenSet || poseEstimateMode) return;
  var yaw  = _robotYawRad || 0;
  /* robotLayer.x = EaselJS scene x = ros_x * FLIP_X → un-flip to get true ROS x */
  drawScan(scan, robotLayer.x, -robotLayer.y, yaw);
});


/* ---------- PANEL RESIZE (horizontal + vertical) ---------- */

(function initResize() {

  function makeHDragger(handleId, panelId, direction) {
    var handle = document.getElementById(handleId);
    var panel  = document.getElementById(panelId);
    if (!handle || !panel) return;
    var isDown = false, startX, startW;

    handle.addEventListener("mousedown", function (e) {
      isDown = true; startX = e.clientX; startW = panel.offsetWidth;
      handle.classList.add("dragging");
      document.body.style.cursor     = "col-resize";
      document.body.style.userSelect = "none";
      e.preventDefault();
    });

    document.addEventListener("mousemove", function (e) {
      if (!isDown) return;
      var delta     = direction === "right" ? e.clientX - startX : startX - e.clientX;
      var desired   = startW + delta;
      var container = document.querySelector(".container");
      var leftW     = document.getElementById("left-panel").offsetWidth;
      var rightW    = document.getElementById("right-panel").offsetWidth;
      var handles   = 14;
      var available = container.offsetWidth - handles;
      var maxPanel  = available - 300 - (direction === "right" ? rightW : leftW);
      var clamped   = Math.min(Math.max(desired, 200), Math.max(200, maxPanel));
      panel.style.width = clamped + "px";
      resizeViewer();
    });

    document.addEventListener("mouseup", function () {
      if (!isDown) return;
      isDown = false;
      handle.classList.remove("dragging");
      document.body.style.cursor     = "";
      document.body.style.userSelect = "";
    });
  }

  function makeVDragger(handleId, bottomId) {
    var handle = document.getElementById(handleId);
    var bottom = document.getElementById(bottomId);
    if (!handle || !bottom) return;
    var isDown = false, startY, startH;

    handle.addEventListener("mousedown", function (e) {
      isDown = true; startY = e.clientY; startH = bottom.offsetHeight;
      handle.classList.add("dragging");
      document.body.style.cursor     = "row-resize";
      document.body.style.userSelect = "none";
      e.preventDefault();
    });

    document.addEventListener("mousemove", function (e) {
      if (!isDown) return;
      var newH = Math.min(Math.max(startH + (startY - e.clientY), 120), 600);
      bottom.style.height = newH + "px";
      resizeViewer();
    });

    document.addEventListener("mouseup", function () {
      if (!isDown) return;
      isDown = false;
      handle.classList.remove("dragging");
      document.body.style.cursor     = "";
      document.body.style.userSelect = "";
    });
  }

  makeHDragger("resize-handle",       "left-panel",       "right");
  makeHDragger("resize-handle-right", "right-panel",      "left");
  makeVDragger("resize-handle-v",     "cameraContainer");
})();


/* ---------- TOAST ---------- */

function showToast(msg, type) {
  var t    = document.getElementById("toast");
  t.textContent = msg;
  t.className   = "toast toast-" + type + " show";
  clearTimeout(t._timer);
  t._timer = setTimeout(function () { t.classList.remove("show"); }, 3800);
}


/* ---------- SECTION LOCKING ---------- */

var MAPPING_BTN_IDS = ["btn-start-mapping", "btn-stop-mapping", "btn-save-map"];
var LOC_BTN_IDS     = ["btn-preview-map", "btn-start-loc"];

function lockSection(ids, reason) {
  ids.forEach(function (id) {
    var btn = document.getElementById(id);
    if (!btn) return;
    btn.disabled = true;
    btn._lockedReason = reason;
    btn.title = reason;
  });
}
function unlockSection(ids) {
  ids.forEach(function (id) {
    var btn = document.getElementById(id);
    if (!btn || !btn._lockedReason) return;
    btn.disabled = false;
    btn._lockedReason = null;
    btn.title = "";
  });
}
function lockMapping()        { lockSection(MAPPING_BTN_IDS, "Disabled during localization"); }
function unlockMapping()      { unlockSection(MAPPING_BTN_IDS); }
function lockLocalization()   { lockSection(LOC_BTN_IDS, "Disabled during mapping"); }
function unlockLocalization() { unlockSection(LOC_BTN_IDS); }

/* ---------- CANCEL NAVIGATION GOAL ---------- */

/* ── DOCK / UNDOCK ──────────────────────────────────────────────────── */
var _docked = false;

function toggleDock() {
  var btn = document.getElementById('btn-dock');
  if (!_docked) {
    fetch(SERVER_URL + '/dock', { method: 'POST' })
      .then(function(r) { return r.json(); })
      .then(function() {
        showToast('🔌 Docking...', 'info');
        setTimeout(function() {
          _docked = true;
          if (btn) {
            btn.innerHTML = '🔋 UNDOCK';
            btn.style.background   = 'rgba(249,115,22,0.12)';
            btn.style.borderColor  = 'rgba(249,115,22,0.4)';
            btn.style.color        = 'var(--accent2)';
          }
          showToast('🔌 Robot docked — charging', 'success');
        }, 2000);
      })
      .catch(function() { showToast('⚠ Dock command failed', 'error'); });
  } else {
    fetch(SERVER_URL + '/undock', { method: 'POST' })
      .then(function(r) { return r.json(); })
      .then(function() {
        _docked = false;
        if (btn) {
          btn.innerHTML = '🔌 DOCK';
          btn.style.background   = 'rgba(34,197,94,0.12)';
          btn.style.borderColor  = 'rgba(34,197,94,0.4)';
          btn.style.color        = 'var(--green)';
        }
        showToast('🔋 Undocking — robot free', 'info');
      })
      .catch(function() { showToast('⚠ Undock command failed', 'error'); });
  }
}

function cancelGoal() {
  /* Send cancel to Nav2 action server */
  fetch(SERVER_URL + "/cancel_goal", { method: "POST" })
    .then(function(r) { return r.json(); })
    .then(function() { showToast("⛔ Navigation cancelled — robot stopping", "info"); })
    .catch(function() { showToast("⚠ Failed to cancel goal", "error"); });

  /* Immediately publish zero velocity so robot stops without waiting for Nav2 */
  if (cmdVel) {
    cmdVel.publish(new ROSLIB.Message({
      linear:  { x: 0.0, y: 0.0, z: 0.0 },
      angular: { x: 0.0, y: 0.0, z: 0.0 }
    }));
    /* Publish zero twice to ensure it arrives */
    setTimeout(function() {
      if (cmdVel) cmdVel.publish(new ROSLIB.Message({
        linear:  { x: 0.0, y: 0.0, z: 0.0 },
        angular: { x: 0.0, y: 0.0, z: 0.0 }
      }));
    }, 100);
  }
}