/* ==========================================================================
   run_program.js — List and run Python programs from the server
   Depends on: app.js (SERVER_URL, showToast)
   ========================================================================== */


/* ---------- LOAD PROGRAM LIST ---------- */

function loadProgramList() {
  fetch(SERVER_URL + "/programs")
    .then(r => r.json())
    .then(function (data) {
      var sel = document.getElementById("programSelect");
      sel.innerHTML = "<option value=''>— select program —</option>";
      data.forEach(function (p) {
        var o = document.createElement("option");
        o.value = o.textContent = p;
        sel.appendChild(o);
      });
    })
    .catch(() => showToast("⚠ Could not load programs", "error"));
}
loadProgramList();


/* ---------- RUN PROGRAM ---------- */

function runProgram() {
  var prog = document.getElementById("programSelect").value;
  if (!prog) { showToast("⚠ Select a program first", "error"); return; }
  fetch(SERVER_URL + "/run_program", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ program: prog })
  })
    .then(r => r.json())
    .then(function() {
      showToast("▶ Running: " + prog, "success");
      var el = document.getElementById("program-status");
      if (el) { el.textContent = "Running: " + prog; el.style.color = "var(--green)"; }
    })
    .catch(() => showToast("⚠ Failed to run program", "error"));
}


/* ---------- STOP PROGRAM ---------- */

function stopProgram() {
  fetch(SERVER_URL + "/stop_program", { method: "POST" })
    .then(r => r.json())
    .then(function(d) {
      showToast("⏹ Program stopped", "info");
      var el = document.getElementById("program-status");
      if (el) { el.textContent = "Idle"; el.style.color = "var(--muted)"; }
    })
    .catch(() => showToast("⚠ Failed to stop program", "error"));
}

/* ---------- STOP PROGRAM + GO HOME ---------- */

function stopProgramAndGoHome() {
  fetch(SERVER_URL + "/stop_program", { method: "POST" })
    .then(r => r.json())
    .then(function(d) {
      var el = document.getElementById("program-status");
      if (el) { el.textContent = "Idle"; el.style.color = "var(--muted)"; }
      if (d.going_home) {
        showToast("⏹ Program stopped — navigating home", "info");
      } else {
        showToast("⏹ Program stopped", "info");
      }
    })
    .catch(() => showToast("⚠ Failed to stop program", "error"));
}