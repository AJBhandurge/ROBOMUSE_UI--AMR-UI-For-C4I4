from flask import Flask, request, jsonify, send_from_directory, send_file
from flask_cors import CORS
import subprocess
import os
import signal
import time
import math
import json
import atexit

# server.py lives in amr_ui/backend/ — root of the package is one level up
BACKEND_DIR     = os.path.dirname(os.path.abspath(__file__))
BASE_DIR        = os.path.dirname(BACKEND_DIR)   # amr_ui/
UI_FOLDER       = BASE_DIR
MAP_FOLDER      = os.path.join(BASE_DIR, "maps")
MISSION_FOLDER  = os.path.join(BASE_DIR, "missions")
MISSION_FILE    = os.path.join(MISSION_FOLDER, "mission.json")
RUNNER_SCRIPT   = os.path.join(BACKEND_DIR, "mission_runner.py")
PROGRAMS_FOLDER = os.path.join(BASE_DIR, "programs")

os.makedirs(MAP_FOLDER,      exist_ok=True)
os.makedirs(MISSION_FOLDER,  exist_ok=True)
os.makedirs(PROGRAMS_FOLDER, exist_ok=True)

import json as _json

# ── Load robot_config.js as Python config ────────────────────────────────────
import json as _json, re as _re

def _load_robot_config():
    cfg_path = os.path.join(BASE_DIR, "config", "robot_config.js")
    try:
        with open(cfg_path) as f:
            src = f.read()
        # Strip JS block and line comments
        src = _re.sub(r'/\*.*?\*/', '', src, flags=_re.DOTALL)
        src = _re.sub(r'//[^\n]*', '', src)
        # Extract object between first { and the matching }
        brace = src.find('var RobotConfig')
        src = src[brace:]
        depth = 0; start = src.index('{'); i = start
        for i, ch in enumerate(src[start:], start):
            if ch == '{': depth += 1
            elif ch == '}':
                depth -= 1
                if depth == 0: break
        obj_str = src[start:i+1]
        # Quote unquoted JS keys
        obj_str = _re.sub(r'([\{,])\s*([a-zA-Z_][a-zA-Z0-9_]*)\s*:', r'\1"\2":', obj_str)
        # Remove trailing commas
        obj_str = _re.sub(r',(\s*[\}\]])', r'\1', obj_str)
        return _json.loads(obj_str)
    except Exception as e:
        print(f"[robot_config] Could not parse robot_config.js ({e}) — using defaults")
        return {
            "mode": "real", "use_sim_time": False,
            "topics": {"map": "/map"},
            "launch": {
                "slam":         {"package": "slam_toolbox", "file": "online_async_launch.py", "extra": []},
                "localization": {"package": "nav2_bringup", "file": "localization_launch.py", "extra": []},
                "navigation":   {"package": "nav2_bringup", "file": "navigation_launch.py",   "extra": []}
            }
        }

_cfg          = _load_robot_config()
USE_SIM_TIME  = str(_cfg.get("use_sim_time", False)).lower()
SLAM_PKG      = _cfg["launch"]["slam"]["package"]
SLAM_LAUNCH   = _cfg["launch"]["slam"]["file"]
SLAM_EXTRA    = _cfg["launch"]["slam"].get("extra", [])
LOC_PKG       = _cfg["launch"]["localization"]["package"]
LOC_LAUNCH    = _cfg["launch"]["localization"]["file"]
LOC_EXTRA     = _cfg["launch"]["localization"].get("extra", [])
NAV_PKG       = _cfg["launch"]["navigation"]["package"]
NAV_LAUNCH    = _cfg["launch"]["navigation"]["file"]
NAV_EXTRA     = _cfg["launch"]["navigation"].get("extra", [])
MAP_TOPIC     = _cfg.get("topics", {}).get("map", "/map")
print(f"[robot_config] mode={_cfg.get('mode')} use_sim_time={USE_SIM_TIME} slam={SLAM_PKG}/{SLAM_LAUNCH}")



# ── ROS2 shared context ──────────────────────────────────────────────────────
# rclpy.init() must be called ONCE per process.
# We init here at import time, then all nodes (map listener, delivery, etc.)
# share this context via a single MultiThreadedExecutor.
import threading
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

rclpy.init()

# QoS that matches nav2 map server (transient-local, reliable, depth 1)
_MAP_QOS = QoSProfile(
    reliability  = ReliabilityPolicy.RELIABLE,
    durability   = DurabilityPolicy.TRANSIENT_LOCAL,
    history      = HistoryPolicy.KEEP_LAST,
    depth        = 1
)

_latest_map  = None
_map_lock    = threading.Lock()
_ros_executor = MultiThreadedExecutor(num_threads=4)

class _MapListenerNode(Node):
    def __init__(self):
        super().__init__("flask_map_listener")
        self.create_subscription(OccupancyGrid, MAP_TOPIC, self._cb, _MAP_QOS)
        self.get_logger().info(f"Subscribed to {MAP_TOPIC}")

    _seq = 0

    def _cb(self, msg):
        global _latest_map
        info = msg.info
        _MapListenerNode._seq += 1
        with _map_lock:
            _latest_map = {
                "width":      info.width,
                "height":     info.height,
                "resolution": info.resolution,
                "origin": {
                    "x": info.origin.position.x,
                    "y": info.origin.position.y,
                    "yaw": 0.0
                },
                "seq":  _MapListenerNode._seq,
                "data": list(msg.data)
            }

_map_node = _MapListenerNode()
_ros_executor.add_node(_map_node)

def _ros_spin_thread():
    try:
        _ros_executor.spin()
    except Exception as e:
        print(f"[ros_executor] stopped: {e}")

threading.Thread(target=_ros_spin_thread, daemon=True).start()
print(f"[ros] Executor started — listening on {MAP_TOPIC}")

app = Flask(__name__, static_folder=UI_FOLDER, static_url_path="")
CORS(app)


# ── Serve UI ──────────────────────────────────────────────────────────────────

@app.route("/")
def index():
    return send_file(os.path.join(UI_FOLDER, "index.html"))

slam_process         = None
localization_process = None
navigation_process   = None
mission_process      = None   # mission_runner.py subprocess


def kill_process(proc):
    if proc and proc.poll() is None:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        except:
            pass


def kill_nav_nodes():
    os.system("pkill -f map_server")
    os.system("pkill -f amcl")
    os.system("pkill -f lifecycle_manager")
    os.system("pkill -f slam_toolbox")
    os.system("pkill -f nav2")
    os.system("pkill -f bt_navigator")
    os.system("pkill -f planner_server")
    os.system("pkill -f controller_server")
    os.system("pkill -f waypoint_follower")


def cleanup_all():
    global slam_process, localization_process, navigation_process, mission_process
    kill_process(slam_process)
    kill_process(localization_process)
    kill_process(navigation_process)
    kill_process(mission_process)
    kill_nav_nodes()

atexit.register(cleanup_all)


# ── Map routes ──────────────────────────────────────────────────────────────

@app.route("/maps")
def get_maps():
    maps = []
    for file in os.listdir(MAP_FOLDER):
        if file.endswith(".pgm"):
            maps.append(file)
    return jsonify(maps)


@app.route("/map_image/<path:name>")
def get_map_image(name):
    from flask import Response
    import io
    base = name
    for ext in (".yaml", ".yml"):
        if base.lower().endswith(ext):
            base = base[:-len(ext)]
            break
    has_ext = "." in os.path.basename(base)
    candidates = [base] if has_ext else [
        base + ".pgm", base + ".png", base + ".jpg", base + ".jpeg"
    ]
    filepath, found_name = None, None
    for c in candidates:
        fp = os.path.join(MAP_FOLDER, c)
        if os.path.isfile(fp):
            filepath, found_name = fp, c
            break
    if not filepath:
        return jsonify({"error": "map image not found: " + name}), 404
    if found_name.lower().endswith((".png", ".jpg", ".jpeg")):
        resp = send_from_directory(MAP_FOLDER, found_name)
        resp.headers["Access-Control-Allow-Origin"] = "*"
        return resp
    try:
        from PIL import Image
        img = Image.open(filepath)
        if img.mode not in ("RGB", "RGBA"):
            img = img.convert("RGB")
        buf = io.BytesIO()
        img.save(buf, format="PNG")
        buf.seek(0)
        resp = Response(buf.read(), mimetype="image/png")
        resp.headers["Access-Control-Allow-Origin"] = "*"
        return resp
    except Exception as e:
        return jsonify({"error": str(e)}), 500



# ── Mapping routes ───────────────────────────────────────────────────────────

@app.route("/start_mapping", methods=["POST"])
def startMapping():
    global slam_process, _latest_map
    cleanup_all()
    # Clear cached map so new mapping session starts fresh (not stale localization map)
    with _map_lock:
        _latest_map = None
    slam_process = subprocess.Popen(
        [f"ros2", "launch", SLAM_PKG, SLAM_LAUNCH,
         f"use_sim_time:={USE_SIM_TIME}"] + SLAM_EXTRA,
        preexec_fn=os.setsid
    )
    return jsonify({"status": "mapping_started"})


@app.route("/stop_mapping", methods=["POST"])
def stopMapping():
    global slam_process
    kill_process(slam_process)
    kill_nav_nodes()
    return jsonify({"status": "mapping_stopped"})


# ── Robot bringup ─────────────────────────────────────────────────────────────
robot_process = None

@app.route("/start_robot", methods=["POST"])
def startRobot():
    global robot_process
    if robot_process and robot_process.poll() is None:
        return jsonify({"status": "already_running"})
    robot_process = subprocess.Popen(
        ["ros2", "launch", "robot_bringup", "robomuse_launch.py"],
        preexec_fn=os.setsid
    )
    return jsonify({"status": "robot_started"})

@app.route("/stop_robot", methods=["POST"])
def stopRobot():
    global robot_process, slam_process, localization_process, navigation_process
    # 1. Kill the bringup launch process group (kills all children including lidar drivers)
    kill_process(robot_process)
    robot_process = None
    # 2. Kill by process name to catch any orphaned nodes from the bringup
    for pattern in [
        "robomuse_launch", "robot_bringup",
        "ira_laser_tools", "laser_merger", "merged_laser",   # lidar merger
        "ldlidar", "rplidar", "urg_node", "laser_scan",       # common lidar drivers
        "robot_state_publisher", "joint_state_publisher",     # robot description
    ]:
        try:
            subprocess.Popen(["pkill", "-9", "-f", pattern], preexec_fn=os.setsid)
        except Exception:
            pass
    # 3. Also kill nav/slam if running
    try:
        kill_nav_nodes()
        kill_process(slam_process);         slam_process         = None
        kill_process(localization_process); localization_process = None
        kill_process(navigation_process);   navigation_process   = None
    except Exception:
        pass
    # 4. Short delay for nodes to die, then cancel any pending /cmd_vel
    time.sleep(0.3)
    return jsonify({"status": "robot_stopped"})

@app.route("/robot_status")
def robotStatus():
    global robot_process
    running = robot_process is not None and robot_process.poll() is None
    return jsonify({"running": running})


@app.route("/save_map", methods=["POST"])
def saveMap():
    data     = request.json
    name     = data["name"]
    out_path = os.path.join(MAP_FOLDER, name)

    # ── Strategy 1: save from cached map data in memory (always works) ──────
    with _map_lock:
        cached = _latest_map

    if cached:
        try:
            w   = cached["width"]
            h   = cached["height"]
            res = cached["resolution"]
            ox  = cached["origin"]["x"]
            oy  = cached["origin"]["y"]
            dat = cached["data"]

            # Build PGM image (map_saver_cli convention):
            #   PGM is stored top-to-bottom visually.
            #   map_saver_cli writes: PGM row 0 = TOP of map (highest y).
            #   OccupancyGrid row 0 = BOTTOM of map (y=origin_y).
            #   So we must VERTICALLY FLIP: write rows in reverse order.
            # Pixel values: 205=unknown, 254=free, 0=occupied
            converted = []
            for v in dat:
                if v == -1:   converted.append(205)
                elif v == 0:  converted.append(254)
                else:         converted.append(0)

            # Flip rows: write row h-1 first (map top), row 0 last (map bottom)
            pixels = []
            for row in range(h - 1, -1, -1):
                pixels.extend(converted[row * w : row * w + w])

            pgm_path  = out_path + ".pgm"
            yaml_path = out_path + ".yaml"

            # Write binary PGM (P5)
            with open(pgm_path, "wb") as f:
                header = "P5\n" + str(w) + " " + str(h) + "\n255\n"
                f.write(header.encode())
                f.write(bytes(pixels))

            # Write YAML
            yaml_content = (
                "image: " + os.path.basename(pgm_path) + "\n"
                "resolution: " + str(res) + "\n"
                "origin: [" + str(ox) + ", " + str(oy) + ", 0.0]\n"
                "negate: 0\n"
                "occupied_thresh: 0.65\n"
                "free_thresh: 0.25\n"
            )
            with open(yaml_path, "w") as f:
                f.write(yaml_content)

            app.logger.info(f"Map saved from cache: {yaml_path}")
            return jsonify({"status": "map_saved", "method": "cache", "path": yaml_path})

        except Exception as e:
            app.logger.error(f"Cache save failed: {e}, trying map_saver_cli")

    # ── Strategy 2: fallback — use map_saver_cli (requires /map topic live) ──
    try:
        result = subprocess.run([
            "ros2", "run", "nav2_map_server", "map_saver_cli",
            "-f", out_path, "--timeout-sec", "8",
            "--ros-args", "-p", f"use_sim_time:={USE_SIM_TIME}"
        ], timeout=12, capture_output=True, text=True)
        if result.returncode == 0:
            return jsonify({"status": "map_saved", "method": "map_saver_cli"})
        else:
            return jsonify({"status": "error", "message": result.stderr}), 500
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500


@app.route("/map_data")
def mapData():
    """Returns full map JSON (used by save_map). For display use /map_tile."""
    with _map_lock:
        if _latest_map is None:
            return jsonify({"status": "no_map"}), 204
        return jsonify(_latest_map)


@app.route("/map_meta")
def mapMeta():
    """Lightweight poll endpoint — returns metadata + checksum only.
    JS polls this at 1Hz; fetches /map_tile only when seq changes."""
    with _map_lock:
        if _latest_map is None:
            return jsonify({"status": "no_map"}), 204
        m = _latest_map
        return jsonify({
            "status":     "ok",
            "width":      m["width"],
            "height":     m["height"],
            "resolution": m["resolution"],
            "origin":     m["origin"],
            "seq":        m.get("seq", 0)
        })


@app.route("/map_tile")
def mapTile():
    """Serves the current live map as a PNG image (RViz colour scheme).
    Called by JS only when seq has changed."""
    import io
    with _map_lock:
        if _latest_map is None:
            return jsonify({"status": "no_map"}), 204
        m      = _latest_map
        w      = m["width"]
        h      = m["height"]
        dat    = m["data"]

    # Build RGBA pixels exactly matching RViz OccupancyGrid display:
    #   -1   → unknown   → 127,127,127 (mid grey, same as RViz)
    #   0    → free      → 255,255,255 (pure white, same as RViz)
    #   1-99 → inflation → scaled grey (darker = more occupied)
    #   100  → occupied  → 0,0,0 (black, same as RViz)
    rgba = bytearray(w * h * 4)
    for i, v in enumerate(dat):
        base = i * 4
        if v < 0:           # unknown → mid grey (RViz)
            rgba[base]=127; rgba[base+1]=127; rgba[base+2]=127; rgba[base+3]=255
        elif v == 0:        # free → white (RViz)
            rgba[base]=255; rgba[base+1]=255; rgba[base+2]=255; rgba[base+3]=255
        elif v >= 100:      # fully occupied → black (RViz)
            rgba[base]=0;   rgba[base+1]=0;   rgba[base+2]=0;   rgba[base+3]=255
        else:               # partially occupied → interpolate white→black
            g = max(0, int(255 * (1.0 - v / 100.0)))
            rgba[base]=g; rgba[base+1]=g; rgba[base+2]=g; rgba[base+3]=255

    # Encode as PNG using zlib-compressed raw RGBA
    try:
        from PIL import Image
        img = Image.frombytes("RGBA", (w, h), bytes(rgba))
        # ROS OccupancyGrid: data[0] = bottom-left cell (y = origin_y)
        # PIL stores top-to-bottom, so PIL row 0 = ROS bottom row
        # EaselJS scaleY<0 handles the flip correctly when bmp.scaleY = +res
        buf = io.BytesIO()
        img.save(buf, format="PNG", optimize=False, compress_level=1)
        buf.seek(0)
        from flask import Response
        resp = Response(buf.read(), mimetype="image/png")
        resp.headers["Cache-Control"] = "no-store"
        resp.headers["Access-Control-Allow-Origin"] = "*"
        return resp
    except ImportError:
        # PIL not available — send raw PGM
        pgm = ("P5\n" + str(w) + " " + str(h) + "\n255\n").encode()
        pix = bytearray(w * h)
        for i, v in enumerate(dat):
            if v == -1:   pix[i] = 127
            elif v == 0:  pix[i] = 255
            else:         pix[i] = max(0, 255 - int(v * 2.55))
        from flask import Response
        resp = Response(pgm + bytes(pix), mimetype="image/x-portable-graymap")
        resp.headers["Access-Control-Allow-Origin"] = "*"
        return resp


@app.route("/ros_status")
def rosStatus():
    """Debug endpoint — shows ROS node health and map subscription status."""
    with _map_lock:
        has_map = _latest_map is not None
        map_info = {
            "width":  _latest_map["width"]  if has_map else 0,
            "height": _latest_map["height"] if has_map else 0,
        } if has_map else {}
    return jsonify({
        "rclpy_ok":    rclpy.ok(),
        "map_topic":   MAP_TOPIC,
        "map_received": has_map,
        "map_info":    map_info,
        "executor_nodes": len(_ros_executor._nodes) if hasattr(_ros_executor, '_nodes') else "unknown"
    })


# ── Localization + Navigation ─────────────────────────────────────────────────

@app.route("/stop_localization", methods=["POST"])
def stopLocalization():
    global localization_process, navigation_process
    kill_process(localization_process)
    kill_process(navigation_process)
    localization_process = None
    navigation_process   = None
    return jsonify({"status": "localization_stopped"})


@app.route("/start_localization", methods=["POST"])
def startLocalization():
    global localization_process, navigation_process, slam_process
    data     = request.json
    map_yaml = data["map"]
    map_path = os.path.join(MAP_FOLDER, map_yaml)
    cleanup_all()

    ros_env = os.environ.copy()
    ros_env["ROS_DOMAIN_ID"] = ros_env.get("ROS_DOMAIN_ID", "0")

    def _launch_stack():
        """Use bringup_launch.py which starts AMCL + map_server + full Nav2
        in a single launch file with a single map argument. This is the
        standard, reliable way to start everything together."""
        global localization_process, navigation_process

        time.sleep(1)   # brief pause for old nodes to die

        # bringup_launch.py handles: map_server, AMCL, bt_navigator,
        # controller_server, planner_server, costmap nodes — everything.
        # Standard argument is map:=<yaml_path>
        # If user has a custom bringup, they can set it in robot_config.js
        bringup_pkg    = _cfg["launch"].get("bringup", {}).get("package", "nav2_bringup")
        bringup_launch = _cfg["launch"].get("bringup", {}).get("file",    "bringup_launch.py")
        bringup_extra  = _cfg["launch"].get("bringup", {}).get("extra",   [])

        localization_process = subprocess.Popen(
            ["ros2", "launch", bringup_pkg, bringup_launch,
             f"map:={map_path}",
             f"use_sim_time:={USE_SIM_TIME}"] + bringup_extra,
            preexec_fn=os.setsid,
            env=ros_env
        )
        # navigation_process points to same process for cleanup compatibility
        navigation_process = localization_process

    threading.Thread(target=_launch_stack, daemon=True).start()

    return jsonify({"status": "localization_and_navigation_starting"})


@app.route("/navigate_to_pose", methods=["POST"])
def navigateToPose():
    data = request.json
    x    = float(data["x"])
    y    = float(data["y"])
    yaw  = float(data["yaw"])
    qz   = math.sin(yaw / 2.0)
    qw   = math.cos(yaw / 2.0)

    goal_yaml = f"pose: {{header: {{frame_id: map}}, pose: {{position: {{x: {x}, y: {y}, z: 0.0}}, orientation: {{x: 0.0, y: 0.0, z: {qz}, w: {qw}}}}}}}"

    subprocess.Popen([
        "ros2", "action", "send_goal",
        "/navigate_to_pose",
        "nav2_msgs/action/NavigateToPose",
        goal_yaml
    ])
    return jsonify({"status": "goal_sent", "x": x, "y": y, "yaw": yaw})


# ── Mission routes ────────────────────────────────────────────────────────────

@app.route("/mission", methods=["GET"])
def getMission():
    """Return current waypoints from mission.json (empty list if none)."""
    if not os.path.exists(MISSION_FILE):
        return jsonify({"waypoints": []})
    with open(MISSION_FILE, "r") as f:
        return jsonify(json.load(f))


@app.route("/mission/save", methods=["POST"])
def saveMission():
    """Save the full waypoints list sent from the frontend."""
    data = request.json          # { "waypoints": [...] }
    with open(MISSION_FILE, "w") as f:
        json.dump(data, f, indent=2)
    return jsonify({"status": "mission_saved", "count": len(data.get("waypoints", []))})


@app.route("/mission/start", methods=["POST"])
def startMission():
    """Launch mission_runner.py as a background process."""
    global mission_process

    # Stop any running mission first
    kill_process(mission_process)

    if not os.path.exists(MISSION_FILE):
        return jsonify({"status": "error", "message": "No mission file found"}), 400

    mission_process = subprocess.Popen(
        ["python3", RUNNER_SCRIPT],
        preexec_fn=os.setsid
    )
    return jsonify({"status": "mission_started"})


@app.route("/dock", methods=["POST"])
def dockRobot():
    """Call manual_dock service on the auto_charging_node."""
    try:
        subprocess.Popen(
            ["ros2", "service", "call",
             "/auto_charging_node/manual_dock",
             "std_srvs/srv/Trigger", "{}"],
            preexec_fn=os.setsid
        )
    except Exception:
        pass
    return jsonify({"status": "docking"})


@app.route("/undock", methods=["POST"])
def undockRobot():
    """Call manual_undock service on the auto_charging_node."""
    try:
        subprocess.Popen(
            ["ros2", "service", "call",
             "/auto_charging_node/manual_undock",
             "std_srvs/srv/Trigger", "{}"],
            preexec_fn=os.setsid
        )
    except Exception:
        pass
    return jsonify({"status": "undocking"})


@app.route("/cancel_goal", methods=["POST"])
def cancelGoal():
    """Cancel all active Nav2 navigation goals.
    Uses the correct ROS2 CLI: ros2 action cancel /navigate_to_pose
    """
    try:
        subprocess.Popen(
            ["ros2", "action", "cancel", "/navigate_to_pose"],
            preexec_fn=os.setsid
        )
    except Exception:
        pass
    return jsonify({"status": "goal_cancelled"})


@app.route("/mission/stop", methods=["POST"])
def stopMission():
    """Stop the running mission."""
    global mission_process
    kill_process(mission_process)
    mission_process = None
    # Also cancel any active Nav2 goal
    subprocess.Popen(["ros2", "action", "send_goal", "/navigate_to_pose",
                      "nav2_msgs/action/NavigateToPose", "{}"])
    return jsonify({"status": "mission_stopped"})


@app.route("/mission/status", methods=["GET"])
def missionStatus():
    """Check if mission_runner is still running."""
    global mission_process
    running = mission_process is not None and mission_process.poll() is None
    return jsonify({"running": running})


# ── Program Runner ────────────────────────────────────────────────────────────

program_process  = None


@app.route("/programs")
def getPrograms():
    progs = []
    if os.path.isdir(PROGRAMS_FOLDER):
        for f in sorted(os.listdir(PROGRAMS_FOLDER)):
            if f.endswith(".py"):
                progs.append(f)
    return jsonify(progs)


@app.route("/run_program", methods=["POST"])
def runProgram():
    global program_process
    data = request.json
    prog = data.get("program", "")
    path = os.path.join(PROGRAMS_FOLDER, prog)
    if not os.path.isfile(path):
        return jsonify({"status": "error", "message": "File not found"}), 404
    kill_process(program_process)
    program_process = subprocess.Popen(
        ["python3", path],
        preexec_fn=os.setsid
    )
    return jsonify({"status": "running", "program": prog})


@app.route("/stop_program", methods=["POST"])
def stopProgram():
    global program_process
    kill_process(program_process)
    program_process = None
    # Navigate robot to home position after program stops
    home = _cfg.get("home", {})
    hx   = float(home.get("x", 0.0))
    hy   = float(home.get("y", 0.0))
    hyaw = float(home.get("yaw", 0.0))
    qz   = math.sin(hyaw / 2.0)
    qw   = math.cos(hyaw / 2.0)
    goal_yaml = (f"pose: {{header: {{frame_id: map}}, pose: {{"
                 f"position: {{x: {hx}, y: {hy}, z: 0.0}}, "
                 f"orientation: {{x: 0.0, y: 0.0, z: {qz}, w: {qw}}}}}}}")
    try:
        subprocess.Popen(["ros2", "action", "send_goal", "/navigate_to_pose",
                          "nav2_msgs/action/NavigateToPose", goal_yaml])
    except Exception:
        pass
    return jsonify({"status": "stopped", "going_home": True})


# ── Shutdown ──────────────────────────────────────────────────────────────────

@app.route("/shutdown", methods=["POST"])
def shutdown():
    cleanup_all()
    os.kill(os.getpid(), signal.SIGTERM)
    return jsonify({"status": "server_shutdown"})


# ── Load Cell Weight (always-on, starts with robot bringup) ──────────────────

_weight_value      = 0.0
_weight_poll_timer = None

def _start_weight_polling():
    """Called once when server starts. Continuously reads /load_cell_data."""
    import re as _re, time as _time, threading as _th
    def _poll():
        global _weight_value
        ros_env = os.environ.copy()
        ros_env["ROS_DOMAIN_ID"] = ros_env.get("ROS_DOMAIN_ID", "0")
        while True:
            try:
                result = subprocess.run(
                    ["ros2", "topic", "echo", "--once", "/load_cell_data"],
                    capture_output=True, text=True, timeout=3, env=ros_env
                )
                m = _re.search(r"data:\s*([\-\d.]+)", result.stdout)
                if m:
                    _weight_value = float(m.group(1))
            except Exception:
                pass
            _time.sleep(0.8)
    _th.Thread(target=_poll, daemon=True).start()

# Start polling immediately when server loads
_start_weight_polling()


@app.route("/weight/status")
def weightStatus():
    return jsonify({"weight": _weight_value})


# ── C4i4 Delivery ─────────────────────────────────────────────────────────────

_delivery_launch_process = None
_delivery_weight         = 0.0
_delivery_state          = "IDLE"

def _delivery_weight_listener():
    import re as _re, time as _time
    global _delivery_weight, _delivery_launch_process
    ros_env = os.environ.copy()
    ros_env["ROS_DOMAIN_ID"] = ros_env.get("ROS_DOMAIN_ID", "0")
    while _delivery_launch_process is not None and _delivery_launch_process.poll() is None:
        try:
            echo = subprocess.run(
                ["ros2", "topic", "echo", "--once", "/load_cell_data"],
                capture_output=True, text=True, timeout=3, env=ros_env
            )
            m = _re.search(r"data:\s*([\-\d.]+)", echo.stdout)
            if m:
                _delivery_weight = float(m.group(1))
        except Exception:
            pass
        _time.sleep(0.8)


@app.route("/delivery/start", methods=["POST"])
def deliveryStart():
    global _delivery_launch_process, _delivery_state, _delivery_weight
    if _delivery_launch_process is not None and _delivery_launch_process.poll() is None:
        return jsonify({"status": "already_running"})
    ros_env = os.environ.copy()
    ros_env["ROS_DOMAIN_ID"] = ros_env.get("ROS_DOMAIN_ID", "0")
    _delivery_launch_process = subprocess.Popen(
        ["ros2", "launch", "load_cell_pkg", "delivery_v2.launch.py"],
        preexec_fn=os.setsid,
        env=ros_env,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL
    )
    _delivery_state  = "IDLE"
    _delivery_weight = 0.0
    import threading as _th
    _th.Thread(target=_delivery_weight_listener, daemon=True).start()
    return jsonify({"status": "started"})


@app.route("/delivery/stop", methods=["POST"])
def deliveryStop():
    global _delivery_launch_process, _delivery_state, _delivery_weight
    if _delivery_launch_process is not None:
        try:
            import signal as _sig
            os.killpg(os.getpgid(_delivery_launch_process.pid), _sig.SIGTERM)
        except Exception:
            try: _delivery_launch_process.terminate()
            except Exception: pass
        _delivery_launch_process = None
    _delivery_state  = "IDLE"
    _delivery_weight = 0.0
    # Send robot home
    home = _cfg.get("home", {})
    hx   = float(home.get("x",   0.0))
    hy   = float(home.get("y",   0.0))
    hyaw = float(home.get("yaw", 0.0))
    import math as _math
    hz = _math.sin(hyaw / 2.0)
    hw = _math.cos(hyaw / 2.0)
    goal = (
        '{"pose": {"header": {"frame_id": "map"}, "pose": {"position": '
        '{"x": %f, "y": %f, "z": 0.0}, "orientation": '
        '{"x": 0.0, "y": 0.0, "z": %f, "w": %f}}}}' % (hx, hy, hz, hw)
    )
    ros_env = os.environ.copy()
    ros_env["ROS_DOMAIN_ID"] = ros_env.get("ROS_DOMAIN_ID", "0")
    subprocess.Popen(
        ["ros2", "action", "send_goal", "/navigate_to_pose",
         "nav2_msgs/action/NavigateToPose", goal],
        preexec_fn=os.setsid, env=ros_env,
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
    )
    return jsonify({"status": "stopped", "going_home": True})


@app.route("/delivery/status")
def deliveryStatus():
    global _delivery_launch_process, _delivery_weight, _delivery_state
    if _delivery_launch_process is None or _delivery_launch_process.poll() is not None:
        return jsonify({"status": "not_running"}), 204
    return jsonify({
        "state":     _delivery_state,
        "weight":    _delivery_weight,
        "threshold": _cfg.get("delivery", {}).get("weight_threshold", 2.0),
    })


app.run(host="0.0.0.0", port=5000)