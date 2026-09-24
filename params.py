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
