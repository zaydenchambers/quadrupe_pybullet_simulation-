"""Editable parameters for the SM3 SpotMicro URDF build.

Edit anything in this file, then run:   python build_urdf.py
(measure_parts.py only needs re-running if the STL files change.)

Every value is tagged:
  [MEASURED]  read straight off the STL geometry (high confidence) - normally derived
              automatically from reports/features.json; the number here is only a
              fallback/override hook.
  [INFERRED]  follows from matching features between parts (screw patterns, bearing
              seats, horn recesses). High-to-medium confidence; see README.md.
  [ESTIMATED] not recoverable from the meshes (mass, servo internals, limits).
              Replace with real hardware data when you have it.

Units: millimetres for geometry, kilograms, radians, N*m, rad/s.
"""

ROBOT_NAME = "spotmicro_sm3"

# --------------------------------------------------------------------------- #
# Frame convention: X forward, Y left, Z up. base_link origin = centre of the
# chassis trunk in X/Y, at the height of the hip-roll axes (front).
# --------------------------------------------------------------------------- #

# ---- hip placement ------------------------------------------------------- #
# None = derive from the shoulder stack (recommended). Give a dict to override,
# e.g. {"front_x": 140.0, "rear_x": -142.0, "y": 39.0, "front_z": 0.0, "rear_z": 0.5}
HIP_OVERRIDE = None

# [INFERRED] the 688 bearing (8x16x5) sits in a 16.26 mm seat and stands proud of
# the bearing plate by this much; the coax's peg-side face rests against it.
BEARING_PROTRUSION = 1.5

# ---- servo internals (DS3218-class standard servo) ------------------------ #
# [ESTIMATED] Position of the output-spline tip along the servo height axis, in each
# part's print frame. The meshes only contain the servo pockets, not the servo,
# so these come from the ear-slot position + a DS3218 datasheet stack-up. They set
# the lateral (Y) offsets of the leg planes, +/-2 mm.
FEMUR_SPLINE_TOP_Y = 15.0      # femur print Y where the hip-pitch spline tip meets the coax horn recess
TIBIA_Z_AT_KNEE_PLANE = -13.85  # tibia print Z (mirrored-right-tibia frame) at the femur's knee horn recess floor

# ---- materials / masses ------------------------------------------------- #
# [ESTIMATED] effective density of printed parts (solid mesh volume x this).
# PLA is 1.24 g/cm^3 solid; typical 20-30 % infill with 3 perimeters lands ~0.6-0.8.
PRINT_DENSITY_G_CM3 = {"frame": 0.75, "cover": 0.85, "foot": 1.0}   # foot assumed TPU, near-solid
SERVO_MASS = 0.060               # [ESTIMATED] kg, DS3218 is 60 g
SERVO_BOX = (40.0, 20.0, 40.5)   # [ESTIMATED] mm, servo body used for inertia
# [ESTIMATED] electronics + battery lumped into base_link (Pi/Jetson, PCA9685, buck
# converters, wiring, 2S LiPo). Position is base frame, mm; box is its inertia envelope.
ELECTRONICS = {"mass": 0.45, "com": (0.0, 0.0, -25.0), "box": (120.0, 80.0, 40.0)}

# ---- joints --------------------------------------------------------------- #
# [ESTIMATED] limits. Knee folds with negative angle (foot swings forward, knee points back).
JOINT_LIMITS = {
    "hip_roll":  (-0.80, 0.80),
    "hip_pitch": (-1.60, 1.60),
    "knee":      (-2.60, 0.10),
}
JOINT_EFFORT = 2.0      # [ESTIMATED] N*m  (DS3218 ~20 kg*cm @ 6.8 V)
JOINT_VELOCITY = 6.5    # [ESTIMATED] rad/s (0.16 s / 60 deg)
JOINT_DAMPING = 0.02    # [ESTIMATED]
JOINT_FRICTION = 0.01   # [ESTIMATED]

# ---- contact -------------------------------------------------------------- #
FOOT_LATERAL_FRICTION = 1.0   # [ESTIMATED] TPU tyre on hard floor
FOOT_SPINNING_FRICTION = 0.01
FOOT_ROLLING_FRICTION = 0.001

# ---- standing pose used by validation / example loader -------------------- #
# Distance from the hip-pitch axis to the foot (tyre centre) in the stance, mm.
STANCE_LEG_EXTENSION = 175.0

# ---- visuals -------------------------------------------------------------- #
VISUAL_MAX_FACES = 80000   # decimate very dense meshes (the head panel is 404k faces)
COLORS = {                  # rough match to the reference photo
    "black":  (0.08, 0.08, 0.09, 1.0),
    "yellow": (0.91, 0.70, 0.30, 1.0),
    "red":    (0.80, 0.22, 0.18, 1.0),
    "grey":   (0.55, 0.55, 0.58, 1.0),
}
PART_COLOR = {
    "Frame_ChassisTrunk": "black", "Frame_FrontShoulderInner": "black", "Frame_FrontShoulderMiddle": "black",
    "Frame_FrontShoulderOuter": "black", "Frame_RearShoulderInner": "black", "Frame_RearShoulderMiddle": "black",
    "Frame_RearShoulderOuter": "black", "coax": "black", "femur": "red", "tibia": "black", "foot": "yellow",
    "femur_cover": "yellow", "cover": "yellow",
}

# Parts in the kit that are deliberately NOT placed in the URDF (reason shown in README).
NOT_PLACED = {
    "Stand_Cone": "display stand, not part of the robot",
    "Stand_ConeBase": "display stand, not part of the robot",
    "Stand_ConeCap": "display stand, not part of the robot",
    "CalibrationTool": "servo-calibration jig, not part of the robot",
    "Cover_Emblem": "decorative badge; no mating feature found - position unknown",
    "Cover_HeadPanelSlug": "blanking plug for the head camera opening; optional",
    "Cover_RearPanelBtnHolder_6x6LongBtn": "internal button bracket; mass lumped in ELECTRONICS",
    "Electronics_BuckConveterCase": "internal; mass lumped in ELECTRONICS",
    "Mp3ProSpeakerMount": "internal; mass lumped in ELECTRONICS",
    "Sensor_MP3SpeakerMount": "internal; mass lumped in ELECTRONICS",
    "Sensor_FrontPIRMount": "internal sensor bracket; mass lumped in ELECTRONICS",
    "Sensor_RearPIRMount": "internal sensor bracket; mass lumped in ELECTRONICS",
    "Sensor_PIRCup": "internal sensor bracket; mass lumped in ELECTRONICS",
    "Sensor_UltraSonicPlate": "internal sensor bracket; mass lumped in ELECTRONICS",
}

# --------------------------------------------------------------------------- #
# Reinforcement learning (quadruped_env.py / train.py / evaluate.py).
# These do not affect the URDF build.
# --------------------------------------------------------------------------- #

# ---- simulation / control -------------------------------------------------- #
# 480 Hz instead of 240: with 240 Hz the feet sank up to 20 mm into the floor on hard landings.
SIM_TIMESTEP = 1 / 480.0      # PyBullet physics step, s
ACTION_REPEAT = 16            # physics steps per env step -> 30 Hz control
# Sim-only contact fixes (the URDF is untouched). The 3.6 g foot link against a 2.2 kg body is a
# bad mass ratio for the contact solver, so the foot is made heavier and its contact stiffer.
SIM_FOOT_MASS = 0.030         # kg
SIM_FOOT_CONTACT_STIFFNESS = 3e4   # N/m
SIM_FOOT_CONTACT_DAMPING = 300.0   # N*s/m
ACTION_FILTER_ALPHA = 0.4     # low-pass on actions: applied = a*new + (1-a)*previous applied (1 = off)
ACTION_SCALE = 0.35           # rad; action in [-1, 1] maps to +/- this offset from the stance
MOTOR_POSITION_GAIN = 0.3     # same servo model as sm3_sim.set_pose
MOTOR_VELOCITY_GAIN = 1.0
EPISODE_SECONDS = 20.0        # time limit -> truncation

# ---- reset randomisation ----------------------------------------------------- #
RESET_JOINT_NOISE = 0.05      # rad, uniform +/- on every joint around the stance
RESET_ORIENT_NOISE = 0.03     # rad, uniform +/- on initial base roll/pitch/yaw
RESET_HEIGHT_NOISE = 0.01     # m, uniform +/- on initial drop height

# ---- reward weights (per env step) ------------------------------------------ #
TARGET_FORWARD_SPEED = 0.30   # m/s; commanded walking speed along the body's forward axis
FORWARD_SPEED_SIGMA = 0.15    # m/s; width of the speed-tracking reward
REWARD_FORWARD_VEL = 2.0      # x exp(-(speed error / sigma)^2)
REWARD_ALIVE = 0.25           # constant bonus for not having fallen (keeps "fall early" unattractive)
PENALTY_TILT = 0.5            # x (roll^2 + pitch^2), rad^2
PENALTY_ENERGY = 0.01         # x sum |torque * joint velocity| * dt over the env step, J
PENALTY_ACTION_RATE = 0.10   # x sum (action - previous action)^2
PENALTY_JOINT_VEL = 0.002     # x sum joint velocity^2, (rad/s)^2; discourages twitching
PENALTY_BODY_CONTACT = 1.0    # x number of non-foot links touching the ground (dragging shins/body)
PENALTY_LATERAL_DRIFT = 0.5   # x |y velocity| (m/s); keeps it walking straight
# Gait shaping: a trot clock replaces the old air-time bonus, which rewarded hopping (26 % foot contact).
GAIT_FREQUENCY_HZ = 2.5       # full trot cycles per second; FL+RR and FR+RL alternate
REWARD_GAIT_MATCH = 0.75      # x fraction of feet whose contact matches the trot clock
PENALTY_FLIGHT = 1.5          # x fraction of the step with all four feet off the ground
PENALTY_VERTICAL_VEL = 1.0    # x base vertical speed^2, (m/s)^2
PENALTY_YAW = 1.0             # x heading error^2, rad^2
PENALTY_LATERAL_OFFSET = 1.0  # x sideways distance from the start line^2, m^2
PENALTY_FOOT_SLIP = 0.25      # x sum over feet in contact of (horizontal foot speed)^2, (m/s)^2

# ---- termination --------------------------------------------------------------- #
MIN_BASE_HEIGHT_FRAC = 0.55   # fall if base height < this fraction of the stance height
MAX_TILT = 0.6                # rad (~34 deg) of roll or pitch

# ---- PPO / training ------------------------------------------------------------ #
TRAIN_TOTAL_STEPS = 15_000_000
TRAIN_N_ENVS = 8              # parallel PyBullet processes
TRAIN_CHECKPOINT_EVERY = 200_000   # env steps (summed over all envs)
PPO_KWARGS = dict(
    n_steps=1024,             # per env -> 8192 samples per update
    batch_size=2048,
    n_epochs=10,
    learning_rate=1.5e-4,
    target_kl=0.02,           # stop an update's epochs early if the policy moves too far (KL was ~0.06)
    gamma=0.99,
    gae_lambda=0.95,
    clip_range=0.2,
    ent_coef=0.0,
    policy_kwargs=dict(net_arch=dict(pi=[256, 256], vf=[256, 256]), log_std_init=-1.0),
)
