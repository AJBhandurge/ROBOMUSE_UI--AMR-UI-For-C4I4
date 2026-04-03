/**
 * ============================================================
 *  robot_config.js  —  AMR Dashboard Configuration
 *  Load this FIRST before all other JS files.
 *
 *  This is the ONLY file you need to edit when:
 *    • Deploying to a new robot
 *    • Switching between simulation and real hardware
 *    • Changing topic names, frame IDs, or ports
 * ============================================================
 */

var RobotConfig = {

  /* ----------------------------------------------------------
     ROBOT MODE
     "sim"  → Gazebo / simulation  (use_sim_time = true)
     "real" → Physical robot       (use_sim_time = false)
  ---------------------------------------------------------- */
  mode: "real",   /* change to "sim" for Gazebo */


  /* ----------------------------------------------------------
     NETWORK — ports the dashboard connects to
  ---------------------------------------------------------- */
  ports: {
    flask:      5000,   /* Flask backend (this server) */
    rosbridge:  9090,   /* rosbridge WebSocket */
    camera:     8080    /* web_video_server */
  },


  /* ----------------------------------------------------------
     ROS TOPICS
     Change these to match your robot's actual topic names.
     Run:  ros2 topic list
     to see what your robot publishes.
  ---------------------------------------------------------- */
  topics: {
    /* Navigation & Localisation */
    cmd_vel:          "/cmd_vel",
    odom:             "/odom",
    map:              "/map",
    scan:             "/merged_laser",
    amcl_pose:        "/amcl_pose",
    initial_pose:     "/initialpose",
    nav_plan:         "/plan",
    global_costmap:   "/global_costmap/costmap",

    /* Camera — topic streamed by web_video_server */
    camera:           "/camera/camera/color/image_raw",   /* RealSense D435i color stream */

    /* Battery (set battery.enabled = true when available) */
    battery_state:    "/jkbms_node/battery_state",   /* JK BMS topic */
    temperature:      "/temperature"    /* optional — set to "" to disable */
  },


  /* ----------------------------------------------------------
     COORDINATE FRAMES
  ---------------------------------------------------------- */
  frames: {
    map:    "map",
    odom:   "odom",
    base:   "base_footprint"   /* use "base_link" if your robot has no footprint */
  },


  /* ----------------------------------------------------------
     THROTTLE RATES (milliseconds)
     Higher = less network traffic but slower updates.
     Lower  = smoother but more bandwidth.
  ---------------------------------------------------------- */
  throttle: {
    scan:      100,    /* laser scan  — 10 Hz */
    odom:      100,    /* odometry    — 10 Hz */
    amcl:      100,    /* AMCL pose   — 10 Hz */
    plan:      500,    /* nav path    — 2 Hz  */
    costmap:  2000,    /* costmap     — 0.5 Hz */
    map_poll: 800      /* HTTP map poll — ~1.2 Hz — fast enough to track SLAM */
  },


  /* ----------------------------------------------------------
     BATTERY DISPLAY
  ---------------------------------------------------------- */
  battery: {
    enabled:          true,    /* set true when /battery_state is available */
    warn_percent:     20,      /* turn orange below this % */
    critical_percent: 10       /* turn red below this % */
  },


  /* ----------------------------------------------------------
     ROBOT DISPLAY (visual size on map canvas, in metres)
  ---------------------------------------------------------- */
  robot: {
    width:  0.50,   /* robot body width  (m) */
    height: 0.40,   /* robot body height (m) */
    color:  "#009900"   /* green — matches RViz robot display */
  },

  /* ----------------------------------------------------------
     DISPLAY — map orientation
     flip_map_x: set true if the map appears left/right mirrored
     compared to the real room. This flips the X axis of the entire
     map display to match RViz orientation.
  ---------------------------------------------------------- */
  display: {
    flip_map_x: false   /* false = normal orientation (matches RViz default)
                              Set true ONLY if map appears left/right mirrored vs RViz */
  },


  /* ----------------------------------------------------------
     LAUNCH PACKAGES
     Change these if your robot uses custom launch files
     instead of the standard nav2_bringup / slam_toolbox.
  ---------------------------------------------------------- */
  launch: {
    slam: {
      package: "slam_toolbox",
      file:    "online_async_launch.py",
      /* Extra args appended to the launch command.
         Add your slam params file here if needed:
         extra: ["params_file:=/path/to/slam_params.yaml"] */
      extra:   []
    },
    localization: {
      package: "nav2_bringup",
      file:    "localization_launch.py",
      extra:   []
      /* e.g. extra: ["params_file:=/path/to/nav2_params.yaml"] */
    },
    navigation: {
      package: "nav2_bringup",
      file:    "navigation_launch.py",
      extra:   []
    },
    /* bringup_launch.py launches EVERYTHING: AMCL + map_server + Nav2
       This is the recommended way — avoids timing issues between nodes.
       Argument: map:=/path/to/map.yaml
       Add your nav2 params file here: extra: ["params_file:=/path/to/nav2_params.yaml"] */
    bringup: {
      package: "nav2_bringup",
      file:    "bringup_launch.py",
      extra:   []
    }
  },


  /* ----------------------------------------------------------
     NAVIGATION ACTION
     Most Nav2 setups use NavigateToPose.
     Change if your robot uses a different action server.
  ---------------------------------------------------------- */
  nav_action: "/navigate_to_pose",


  /* ----------------------------------------------------------
     HOME POSITION
     Where "Go Home" navigates to (in map frame, metres).
  ---------------------------------------------------------- */
  home: {
    x:   0.0,
    y:   0.0,
    yaw: 0.0
  },


  /* ----------------------------------------------------------
     DELIVERY — Load Cell + AprilTag Docking
  ---------------------------------------------------------- */
  auth: {
    username: "ROBOMUSE",
    password: "1234"
  },

  delivery: {

    /* Load cell topic */
    load_cell_topic:    "/load_cell_data",

    /* Weight (kg) to trigger a delivery run */
    weight_threshold:   2.0,

    /* Seconds weight must be stable before accepting load/unload */
    weight_stable_time: 5.0,

    /* Delay (s) after load confirmed, before robot moves toward delivery */
    pre_navigate_delay: 5.0,

    /* Delay (s) after unload confirmed, before robot returns home */
    post_unload_delay:  5.0,

    /* AprilTag detections topic (shared by both stations) */
    tag_topic: "/tag_detections",

    /* ── DOCKING BEHAVIOUR (shared by both stations) ──────
       Tune these once — they apply to both docks.       */
    dock: {
      dock_distance:   0.40,   /* metres — stop this far in front of tag */
      align_tolerance: 0.03,   /* metres — lateral error tolerance       */
      angle_tolerance: 0.05,   /* radians — yaw error tolerance          */
      linear_speed:    0.08,   /* m/s — approach speed                   */
      angular_speed:   0.30,   /* rad/s — correction speed               */
      timeout:         15.0    /* seconds before giving up on dock       */
    },

    /* ── LOADING STATION (where the robot starts / returns to) ───
       tag_id: AprilTag ID mounted at loading station
       waypoint: ~0.5–1 m in front of tag, facing it (map frame)
       Get waypoint from:  ros2 topic echo /amcl_pose --once
       (drive robot to that position first)                      */
    loading_station: {
      tag_id:  1,
      waypoint: {
        x:  0.24,  y:  0.08,  z: 0.0,
        qx: 0.0,   qy: 0.0,   qz: -0.13,  qw: 0.99
      }
    },

    /* ── DELIVERY STATION (where the robot drops off the load) ───
       tag_id: AprilTag ID mounted at delivery station
       waypoint: ~0.5–1 m in front of tag, facing it (map frame) */
    delivery_station: {
      tag_id:  0,
      waypoint: {
        x:  0.21,  y:  3.99,  z: 0.0,
        qx: 0.0,   qy: 0.0,   qz:  0.03,  qw: 0.99
      }
    }
  }

};


/* ============================================================
   DO NOT EDIT BELOW THIS LINE
   These derive computed values from the config above.
   ============================================================ */

RobotConfig.use_sim_time = (RobotConfig.mode === "sim");

/* Convenience: resolved camera URL built at runtime in app.js
   using RobotConfig.topics.camera and RobotConfig.ports.camera */

console.log(
  "[robot_config] Mode:", RobotConfig.mode,
  "| use_sim_time:", RobotConfig.use_sim_time,
  "| cmd_vel:", RobotConfig.topics.cmd_vel
)