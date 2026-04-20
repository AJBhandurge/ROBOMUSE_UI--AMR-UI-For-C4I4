/* ==========================================================================
   teleop.js — Robot teleoperation (buttons + keyboard)
   Depends on: app.js (cmdVel, showToast, ros)

   KEY DESIGN: motor.py has a 0.5s watchdog — if no /cmd_vel arrives in
   0.5s it stops the robot. We must publish CONTINUOUSLY at 100ms (10 Hz)
   while a button is held. On release we simply stop publishing — motor.py
   watchdog handles the physical stop automatically.
   ========================================================================== */


/* ---------- SPEED HELPERS ---------- */
function getLinearSpeed() {
  var el = document.getElementById("speed-linear");
  return el ? (parseFloat(el.value) || 0.4) : 0.4;
}
function getAngularSpeed() {
  var el = document.getElementById("speed-angular");
  return el ? (parseFloat(el.value) || 0.8) : 0.8;
}


/* ---------- VELOCITY PUBLISHER ---------- */
function publishVel(linX, angZ) {
  if (!cmdVel) return;   /* ROS not connected yet — silently ignore */
  cmdVel.publish(new ROSLIB.Message({
    linear:  { x: linX, y: 0.0, z: 0.0 },
    angular: { x: 0.0,  y: 0.0, z: angZ }
  }));
}
function stop() { publishVel(0, 0); }


/* ---------- CONTINUOUS PUBLISHING WHILE BUTTON HELD -------------------
 * _velInterval fires every 100 ms while a direction button is held.
 * This keeps motor.py watchdog (0.5 s) fed so robot never stops mid-press.
 * On release: clearInterval → stop publishing → motor.py stops robot.
 * ---------------------------------------------------------------------- */
var _velInterval  = null;   /* active interval handle     */
var _velLinX      = 0;      /* current linear  velocity   */
var _velAngZ      = 0;      /* current angular velocity   */

function _startContinuous(linX, angZ) {
  /* Update velocities */
  _velLinX = linX;
  _velAngZ = angZ;
  /* Publish immediately (don't wait for first tick) */
  publishVel(_velLinX, _velAngZ);
  /* If no interval running yet, start one */
  if (!_velInterval) {
    _velInterval = setInterval(function () {
      publishVel(_velLinX, _velAngZ);
    }, 100);   /* 10 Hz — well within motor.py 0.5 s watchdog */
  }
}

function _stopContinuous() {
  /* Stop publishing — motor.py watchdog handles physical stop */
  if (_velInterval) {
    clearInterval(_velInterval);
    _velInterval = null;
  }
  _velLinX = 0;
  _velAngZ = 0;
  /* Do NOT send stop() — motor.py watchdog is the stop mechanism */
}


/* ---------- DIRECTION FUNCTIONS (called by buttons + keyboard) --------- */
function forward() { _startContinuous( getLinearSpeed(),  0); }
function back()    { _startContinuous(-getLinearSpeed(),  0); }
function left()    { _startContinuous( 0,  getAngularSpeed()); }
function right()   { _startContinuous( 0, -getAngularSpeed()); }

/* Hard stop — used by stop button (■) and spacebar */
function stopHard() {
  _stopContinuous();
  stop();   /* send explicit zero once for immediate stop */
}

/* Button release handler — just stop publishing, let motor.py watchdog act */
function releaseButton() { _stopContinuous(); }


/* ---------- KEYBOARD TELEOP -------------------------------------------- */
var keysDown = {};

document.addEventListener("keydown", function (e) {
  var tag = document.activeElement && document.activeElement.tagName;
  if (tag === "INPUT" || tag === "SELECT" || tag === "TEXTAREA") return;
  if (keysDown[e.key]) return;   /* suppress browser key-repeat */
  keysDown[e.key] = true;

  switch (e.key) {
    case "ArrowUp":    case "w": case "W": forward();  break;
    case "ArrowDown":  case "s": case "S": back();     break;
    case "ArrowLeft":  case "a": case "A": left();     break;
    case "ArrowRight": case "d": case "D": right();    break;
    case " ":                              stopHard();  e.preventDefault(); break;
  }
});

document.addEventListener("keyup", function (e) {
  keysDown[e.key] = false;
  var moveKeys = ["ArrowUp","ArrowDown","ArrowLeft","ArrowRight",
                  "w","a","s","d","W","A","S","D"];
  if (moveKeys.indexOf(e.key) !== -1) releaseButton();
});


/* ---------- SAFETY STOP on page hide ----------------------------------- */
document.addEventListener("visibilitychange", function () {
  if (document.hidden) stopHard();
});