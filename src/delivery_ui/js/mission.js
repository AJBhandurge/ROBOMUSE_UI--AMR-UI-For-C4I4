/* ==========================================================================
   mission.js — Waypoint mission system
   ========================================================================== */

var waypoints      = [];
var waypointMode   = false;
var missionRunning = false;
var _MISSION_KEY   = "rbm_mission";

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

function enableWaypointMode() {
  if (!poseHasBeenSet) { showToast("⚠ Set 2D Pose Estimate first", "error"); return; }
  waypointMode = true; poseEstimateMode = false; goalPoseMode = false;
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

function redrawWaypointMarkers() {
  waypointLayer.removeAllChildren();
  var scaleX_fix = (typeof FLIP_X !== "undefined" && FLIP_X < 0) ? -1 : 1;
  waypoints.forEach(function(wp, idx) {
    var dot = new createjs.Shape();
    dot.graphics.beginFill("#f97316").drawCircle(0,0,0.10);
    dot.x = wp.x; dot.y = wp.y;
    var tick = new createjs.Shape();
    tick.graphics.setStrokeStyle(0.035).beginStroke("#f97316").moveTo(0,0).lineTo(0.18,0);
    tick.x = wp.x; tick.y = wp.y; tick.rotation = wp.yaw * (180/Math.PI);
    var lbl = new createjs.Text(String(idx+1),"bold 0.14px Arial","#ffffff");
    lbl.textAlign="center"; lbl.textBaseline="middle";
    lbl.x=wp.x; lbl.y=wp.y; lbl.scaleY=-1; lbl.scaleX=scaleX_fix;
    waypointLayer.addChild(dot); waypointLayer.addChild(tick); waypointLayer.addChild(lbl);
  });
  stage.update();
}

function renderWpList() {
  var el = document.getElementById("wp-list");
  if (waypoints.length === 0) {
    el.innerHTML = "<div style='color:var(--muted);font-size:13px;padding:8px 0;'>No waypoints yet</div>";
    var b = document.getElementById("btn-start-mission"); if (b) b.disabled = true;
    return;
  }
  el.innerHTML = waypoints.map(function(wp, i) {
    var delay = wp.delay !== undefined ? wp.delay : 0;
    return "<div class='wp-item' draggable='true' data-idx='"+i+"'" +
      " ondragstart='wpDragStart(event,"+i+")' ondragover='wpDragOver(event)'" +
      " ondrop='wpDrop(event,"+i+")' ondragend='wpDragEnd(event)'>" +
      "<span class='wp-drag'>⠿</span>" +
      "<span class='wp-name'>"+(i+1)+". "+wp.name+"</span>" +
      "<div class='wp-delay-wrap'><input class='wp-delay-input' type='number' min='0' step='1' value='"+delay+"'" +
      " onchange='setWpDelay("+i+",this.value)' onclick='event.stopPropagation()'></div>" +
      "<button class='wp-del' onclick='removeWaypoint("+i+")'>✕</button></div>";
  }).join("");
  var bs = document.getElementById("btn-start-mission"); if (bs) bs.disabled = false;
}

var _dragIdx = null;
function wpDragStart(e,idx){_dragIdx=idx;e.currentTarget.classList.add("wp-dragging");e.dataTransfer.effectAllowed="move";}
function wpDragOver(e){e.preventDefault();e.dataTransfer.dropEffect="move";document.querySelectorAll(".wp-item").forEach(function(el){el.classList.remove("wp-drag-over");});e.currentTarget.classList.add("wp-drag-over");}
function wpDrop(e,toIdx){e.preventDefault();if(_dragIdx===null||_dragIdx===toIdx)return;var moved=waypoints.splice(_dragIdx,1)[0];waypoints.splice(toIdx,0,moved);renderWpList();redrawWaypointMarkers();saveMissionFile();}
function wpDragEnd(){_dragIdx=null;document.querySelectorAll(".wp-item").forEach(function(el){el.classList.remove("wp-dragging");el.classList.remove("wp-drag-over");});}
function setWpDelay(idx,val){waypoints[idx].delay=Math.max(0,parseFloat(val)||0);saveMissionFile();}
function removeWaypoint(idx){waypoints.splice(idx,1);renderWpList();redrawWaypointMarkers();saveMissionFile();}
function clearWaypoints(){waypoints=[];renderWpList();redrawWaypointMarkers();showToast("🗑 Waypoints cleared","info");}

function saveMissionFile() {
  fetch(SERVER_URL+"/mission/save",{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify({waypoints:waypoints})}).catch(function(){});
}

/* ── Session ── */
function _saveMissionSession() {
  try {
    var m = document.getElementById("mapSelect");
    sessionStorage.setItem(_MISSION_KEY, JSON.stringify({running:true, map: m ? m.value : ""}));
  } catch(e){}
}
function _clearMissionSession(){ try{sessionStorage.removeItem(_MISSION_KEY);}catch(e){} }
function _loadMissionSession(){ try{var r=sessionStorage.getItem(_MISSION_KEY);return r?JSON.parse(r):null;}catch(e){return null;} }

/* ── UI state ── */
function _uiRunning() {
  var bs=document.getElementById("btn-start-mission");
  var bst=document.getElementById("btn-stop-mission");
  if(bs){bs.textContent="🚀 Running…";bs.disabled=true;bs.style.display="none";}
  if(bst){bst.style.display="block";bst.disabled=false;}
}
function _uiStopped() {
  var bs=document.getElementById("btn-start-mission");
  var bst=document.getElementById("btn-stop-mission");
  if(bs){bs.textContent="▶ Run Mission";bs.disabled=(waypoints.length===0);bs.style.display="block";}
  if(bst){bst.style.display="none";bst.disabled=false;}
  /* Immediately unlock all nav buttons — no waiting, no disabling */
  if(poseHasBeenSet){
    var g=document.getElementById("btn-goal");
    var h=document.getElementById("btn-home");
    var c=document.getElementById("btn-cancel-goal");
    if(g){g.disabled=false;g.classList.add("btn-goal-ready");}
    if(h) h.disabled=false;
    if(c) c.disabled=false;
  }
}

/* ── Start ── */
function startMission() {
  if(waypoints.length===0){showToast("⚠ Add waypoints first","error");return;}
  fetch(SERVER_URL+"/mission/save",{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify({waypoints:waypoints})})
    .then(function(r){return r.json();})
    .then(function(){return fetch(SERVER_URL+"/mission/start",{method:"POST"});})
    .then(function(r){return r.json();})
    .then(function(){
      missionRunning=true;
      if(typeof lockTeleop==="function") lockTeleop();
      _saveMissionSession();
      _uiRunning();
      showToast("🚀 Mission started — "+waypoints.length+" waypoints","success");
      _poll();
    })
    .catch(function(){showToast("⚠ Failed to start mission","error");});
}

/* ── Stop ──
   UI unlocked IMMEDIATELY — server cancel happens in background.
   This guarantees Go-to-Point works the instant user clicks Stop. */
function stopMission() {
  missionRunning = false;
  if(typeof unlockTeleop==="function") unlockTeleop();
  if(typeof clearNavPath==="function") clearNavPath();
  _clearMissionSession();
  _uiStopped();
  showToast("⛔ Mission stopped — ready","success");

  /* Background: kill mission_runner and cancel Nav2 goal.
     Nav2 cancel is now NON-blocking (Popen not run) — we just kill the
     runner process. Nav2 will finish the current goal segment then idle.
     This is correct behaviour and avoids the 5s synchronous wait that
     was blocking the UI thread. */
  fetch(SERVER_URL+"/mission/stop",{method:"POST"}).catch(function(){});
}

/* ── Poll ── */
var _pollTimer=null;
function _poll(){
  if(_pollTimer){clearTimeout(_pollTimer);_pollTimer=null;}
  if(!missionRunning) return;
  fetch(SERVER_URL+"/mission/status")
    .then(function(r){return r.json();})
    .then(function(d){
      if(!missionRunning) return;
      if(!d.running){
        missionRunning=false;
        if(typeof unlockTeleop==="function") unlockTeleop();
        if(typeof clearNavPath==="function") clearNavPath();
        _clearMissionSession();
        _uiStopped();
        showToast("✅ Mission complete","success");
      } else {
        _pollTimer=setTimeout(_poll,3000);
      }
    })
    .catch(function(){ if(missionRunning) _pollTimer=setTimeout(_poll,5000); });
}

function enableMissionButtons(){var el=document.getElementById("btn-add-wp");if(el)el.disabled=false;}

/* ── Load waypoints on page load ── */
fetch(SERVER_URL+"/mission")
  .then(function(r){return r.json();})
  .then(function(data){if(data.waypoints&&data.waypoints.length>0){waypoints=data.waypoints;renderWpList();}})
  .catch(function(){});

/* ── Refresh reconnect ─────────────────────────────────────────────────────
   When the user refreshes:
   • Flask keeps running — mission_runner.py is still alive
   • We check /mission/status — if alive, restore UI ONLY
   • We do NOT call startLocalization() — Nav2 + AMCL are already running
   • We do NOT publish initialpose — AMCL is already tracking
   • The /amcl_pose subscriber fires as soon as ROSBridge reconnects and
     updates the robot position on the map automatically
   ────────────────────────────────────────────────────────────────────── */
function _reconnect() {
  var sess = _loadMissionSession();
  if(!sess||!sess.running) return;

  fetch(SERVER_URL+"/mission/status")
    .then(function(r){return r.json();})
    .then(function(d){
      if(!d.running){_clearMissionSession();return;}

      /* Restore UI — nothing else needed */
      missionRunning = true;
      if(typeof lockTeleop==="function") lockTeleop();
      _uiRunning();

      /* Set correct map in selector (visual only — stack is already running) */
      if(sess.map){
        var ms=document.getElementById("mapSelect");
        if(ms){for(var i=0;i<ms.options.length;i++){if(ms.options[i].value===sess.map){ms.selectedIndex=i;break;}}}
      }

      /* Mark pose as set so nav buttons are enabled when AMCL pose arrives */
      poseHasBeenSet=true;
      var g=document.getElementById("btn-goal");
      var h=document.getElementById("btn-home");
      if(g){g.disabled=false;g.classList.add("btn-goal-ready");}
      if(h) h.disabled=false;
      if(typeof enableMissionButtons==="function") enableMissionButtons();

      showToast("🔄 Reconnected — mission still running on robot","success");
      _poll();
    })
    .catch(function(){_clearMissionSession();});
}

setTimeout(_reconnect, 1500);
