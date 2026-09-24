# SM3 SpotMicro → URDF (PyBullet)

A 12-DOF URDF of the SM3 SpotMicro-style quadruped, built from the kit's 38 individually print-oriented STL files. The folder also contains the scripts that regenerate the URDF, a PyBullet loader and a validation suite.

```
pip install -r requirements.txt
python build_urdf.py                                        # ~7 s, regenerates URDF + meshes/
python validate_pybullet.py --photo reports/reference_photo.png   # 9 checks, renders, exit 0 = pass
python load_example.py                                      # GUI: drop, stand, slow squats
python tools/measure_parts.py                               # only if the STLs change (~6 min)
```

All paths resolve relative to the script location, so you can move the folder anywhere. The URDF uses relative mesh paths (`meshes/*.stl`, `scale="0.001 0.001 0.001"`). For ROS, change them to `package://<pkg>/meshes/...`.

## What's in the box

| path | what |
|---|---|
| `spotmicro_sm3.urdf` | generated robot: 29 links, 12 revolute + 16 fixed joints |
| `meshes/` | 25 STL files (mm) already baked into their link frames. FL/RL share `L_*` and FR/RR share `R_*` |
| `params.py` | **everything you should edit**: hip overrides, masses, servo specs, limits, friction, colours |
| `build_urdf.py` | steps 2–4: re-orients the parts, places the panels, computes inertials and writes the URDF |
| `tools/measure_parts.py` | step 1: bbox/PCA/proximal-distal table and functional-feature extraction → `reports/features.json` |
| `tools/sm3lib.py` | geometry helpers (bore finder, tab-plane fit, mirror-pair ICP, box inertia) |
| `tools/photo_match.py` | fits a camera to the reference photo by silhouette IoU and renders the URDF from it |
| `validate_pybullet.py` | steps 5–6: topology, gravity stand, support polygon, shove recovery, renders |
| `sm3_sim.py`, `load_example.py` | stance IK helper and example loader |
| `source_stl/` | the original kit STLs (inputs) |
| `reports/` | `part_measurements.md`, `features.json`, `build_summary.json` (every derived number with its provenance tag), `validation_results.json`, `gravity_sim_log.json`, renders |

## Kinematics

Frame convention: X forward, Y left, Z up. The `base_link` origin sits at the centre of the chassis trunk in X and Y, at the height of the front hip-roll axes.

```
base_link ─┬─ {leg}_hip_roll  (revolute, X) → {leg}_coax
           │     └ {leg}_hip_pitch (revolute, Y) → {leg}_femur ── fixed → {leg}_femur_cover
           │           └ {leg}_knee (revolute, Y) → {leg}_tibia
           │                 └ {leg}_foot_fixed → {leg}_foot   (sphere collision r = 18.5 mm)
           └─ fixed → cover_head, cover_rear_shell, cover_rear_base, cover_top_front, cover_top_rear,
                      cover_bottom_front, cover_bottom_rear, cover_belly_hatch
```

Every leg link has its origin on its proximal joint axis, and its distal joint lies on the link's −Z axis. The small built-in tilt of each part (femur 4.3°, tibia −1.7°) is baked into the mesh, so a zero pose means straight legs.

| quantity | value | tag |
|---|---|---|
| hip-roll axes, lateral half-spacing | 38.85 mm | MEASURED (bearing seats ±39.1 / horn recesses) |
| hip x (front / rear) | +139.3 / −143.2 mm | INFERRED (shoulder stack, see below) |
| rear roll axis relative to front | +0.40 mm | INFERRED |
| roll axis → pitch axis (vertical drop) | 10.55 mm | MEASURED (coax peg vs horn pattern) |
| roll axis → femur plane (lateral) | 24.71 mm | MEASURED |
| femur, hip → knee | 107.98 mm | MEASURED (servo-ear line + knee horn pattern) |
| tibia, knee → tyre centre | 116.00 mm | MEASURED (Ø8 knee pin → tip arc centre) |
| tyre outer radius | 18.5 mm | MEASURED |
| foot lateral offset from centreline | 88.2 mm | INFERRED |
| roll axis height, stance of 175 mm | 204.0 mm | from `STANCE_LEG_EXTENSION` |

Sign conventions: all four legs use the same axes. `hip_pitch > 0` swings the knee backwards, and `knee < 0` folds the tibia forward, so the knees point back like Spot's. `hip_roll > 0` moves the foot toward +Y, which is abduction on the left legs and adduction on the right legs.

## How each part was placed (the evidence)

There's no shared assembly frame, so every placement comes from features that have to mate. None of the placements come from eyeballing.

**Joint axes come from servo interfaces.** Each leg part carries one servo pocket, and each part receives a round Ø20 horn recess with a 4+1 hole pattern from the next servo. A standard servo has ears 48.5 mm apart, and its spline is offset about 10.5 mm from the body centre toward one ear. I found:
- **Coax.** It holds the **roll** servo. Its Ø8.00 idler peg sits coaxial with the servo spline (shaft offset from the ear centre is 10.75 mm). The horn recess on its +X face takes the **pitch** servo, which is 10.55 mm below the roll axis and perpendicular to it.
- **Femur.** It holds the **pitch** servo (ear pairs 48.57 mm apart, tilted 2.96°). A horn recess at its far end takes the knee servo. The spline sits on the ear line, 10.49 mm from the servo centre. That offset is the mean of the coax peg and tibia pin measurements. The spline is on the end away from the knee, which gives 107.98 mm hip→knee.
- **Tibia.** It holds the **knee** servo. Its Ø8.00 idler pin is 10.24 mm from its servo centre (i.e. coaxial with the spline). The foot is the centre of the r = 15 mm tip arc, and the half-tyre (r = 18.5) wraps it.
- **Femur cover.** It has a Ø16.26 seat for a 688 bearing (8×16×5) on the knee axis. Its screw tab lands 1.1 mm from the femur's outer servo-ear holes, and the tibia's Ø8 pin reaches 1.8 mm into that bearing.
- **Right-side parts** are exact mirrors of the left ones. ICP after reflection gives residuals of 0.005–0.012 mm mean, so the right legs are the left geometry mirrored through the sagittal plane.

**Body (shoulder stacks).** The screw patterns chain the parts together:
- The trunk's end holes (y ±51.7, z 5/24) match the Inner plate holes, and the trunk top (46.9 mm) sits flush with the Inner plate top.
- The Inner posts' top holes match the bearing plate (front "Middle", rear "Outer"), which has 16.26 mm bearing seats for the coax pegs.
- The bearing-plate flange ends match the horn plate (front "Outer", rear "Middle"), which has Ø20 horn recesses holding the roll-servo horns.
- The coax spans the 48 mm bay between the two plates. Its peg rides in the bearing and the roll-servo spline goes into the horn. The front and rear stacks are built in opposite orders, so every coax faces the same way. That's why FL and RL (and FR and RR) share one mesh.
- The roll axis ends up 9 mm above shoulder centre. Three independent checks agree: the trunk-top flush, the Left/Right coax chirality, and the ±20 mm cover-screw holes sitting on the roll-axis side, where the ultrasonic-sensor cover (TopRear) has to be on top.

**Shell panels.** These were all exported standing on edge, so none of them was placed at identity:
- **Head and rear panel.** Each has two thin mounting tabs, and each tab's plane normal comes from PCA. The tab pairs are parallel to within 0.14° (head) and 0.19° (rear), so each panel has one rigid mounting orientation. Sectioning perpendicular to that normal makes every tab screw hole circular (circularity 0.993–0.995); it isn't circular on any other axis. The lateral hole spacing (±20 top, ±5 bottom) matches the shoulder flange holes. The tab-plane separation (68.9 mm) equals the flange span (66 mm) plus one tab thickness, and it's identical for both panels. Result: the head's print bed face is its front face, tipped 25.5° below horizontal (a visor), and the rear panel's print bed face is its back face, tipped 4.0°.
- **Top and bottom U-section covers.** Their tongue holes (±20 top, ±5 bottom) share those flange screws. With the panel tab stacked under each tongue, the inner wall of all four covers lands on the shoulder plates' outer faces to within 0.1 mm (`check.*_inner_wall_vs_shoulder_face_mm`), without any tuning. The BottomFront cover's r = 20 cut-out lands 1.4 mm from the trunk's r = 20 end cut-out.
- **Belly hatch.** It's printed diagonally, so it was oriented by PCA (213 × 108 × 23). Its end holes are 191.2 mm apart, and the bottom covers' centre bosses are 192.1 mm apart.

**Proximal/distal heuristic vs features.** The bulkier-cross-section rule is reported in `part_measurements.md`. It picks the right end for the tibia and the **wrong** end for the femur and femur cover. The femur's knee end is a solid horn disc and its hip end is a hollow servo pocket. The functional features override the heuristic wherever they disagree.

## Confidence: what to trust, what to replace

| item | confidence | notes / how to refine |
|---|---|---|
| link lengths (femur 107.98, tibia 116.00, roll→pitch 10.55, tyre r 18.5), hip half-spacing 38.85 | **high** (MEASURED) | read straight off bores, pins and horn patterns |
| part orientations inside each link, left/right mirroring | **high** | ICP residual ≤ 0.012 mm |
| hip x positions (+139.3 / −143.2), roll-axis height | **high–medium** (INFERRED) | chained through 3–4 screw patterns, ±1–2 mm. The asymmetry is real: the coax sits off-centre in its bay. Override with `HIP_OVERRIDE` |
| head / rear panel / cover placement | **medium–high** | orientation is hard-constrained. Covers seat within 0.1 mm. The top/bottom side seam shows a 1.5–1.9 mm gap. The panel tab under each cover tongue is inferred from that fit |
| lateral leg-plane offsets (femur spline tip, tibia knee plane) | **medium** (ESTIMATED) | depend on the servo's ear-to-spline height, which isn't in the meshes. `FEMUR_SPLINE_TOP_Y`, `TIBIA_Z_AT_KNEE_PLANE` (±2 mm) |
| masses, inertias | **low** (ESTIMATED) | printed volume × 0.75/0.85 g/cm³ + 60 g per servo + 450 g electronics = **2.23 kg**. Weigh your parts and put the real numbers in `params.py` |
| joint limits, effort 2 N·m, velocity 6.5 rad/s, damping, friction | **low** (ESTIMATED) | DS3218-class guesses. Measure the mechanical stops on the real robot |
| trunk front/back orientation | **low** | the trunk is nearly symmetric. It's placed as printed, which only affects visuals |

Parts deliberately **not placed** (listed with reasons in `params.NOT_PLACED`): the display stand (3 parts), the calibration tool, the emblem and the head blanking slug (no mating feature found), and 7 internal brackets (PIR, ultrasonic, speaker, buck converter, button holder). Their mass is covered by the 450 g electronics lump.

## Validation (`reports/validation_results.json`)

All 9 checks pass:
- 12 revolute joints. Each chain goes base → coax → femur → tibia → foot, with world axes X/Y/Y.
- All four legs have identical FK lengths, and left/right are symmetric.
- **Gravity sim at 240 Hz.** The robot is dropped from 30 mm and held by position servos at 2 N·m. It settles at 203.7 mm against 204.0 mm from IK, with roll ≤ 0.14° and pitch ≤ 0.31°.
- **Load on the feet only.** Time-averaged contact forces: FL 23 %, FR 22 %, RL 27 %, RR 28 % of weight, summing exactly to the weight, with 0 N on any other link.
- **COM inside the support polygon** at every sample. Minimum margin 88 mm. The COM sits 38 mm aft of the trunk centre. That puts it 14 mm aft of the midpoint between the feet, which is why the rear feet carry more of the load.
- It recovers from a 0.58 N·s sideways shove (peak tilt 0.4°).
- **Visual check.** A camera fitted to the reference photo reaches **0.73 silhouette IoU** (`render_vs_photo.png`). The biggest mismatch is the head's lower front edge, which reaches a little further forward in the render than in the photo. Both photo-specific details — how the robot sits on the stand and the hand-set leg angles — also cost IoU. The photo's black vertical rods on the shoulders don't match any file in the kit.

Instantaneous PyBullet contact lists flicker between 3 and 4 feet on a rigid 4-legged stance. The checks therefore use time-averaged forces.

## Regenerating after edits

Edit `params.py`, then run `python build_urdf.py && python validate_pybullet.py`. Common edits:
- **Real masses:** `SERVO_MASS`, `PRINT_DENSITY_G_CM3`, `ELECTRONICS`.
- **Hip placement:** `HIP_OVERRIDE = {"front_x": ..., "rear_x": ..., "y": ..., "front_z": 0, "rear_z": ...}` (mm).
- **Limits:** `JOINT_LIMITS`, `JOINT_EFFORT`, `JOINT_VELOCITY`.
- **Different servo:** `FEMUR_SPLINE_TOP_Y`, `TIBIA_Z_AT_KNEE_PLANE`, `SERVO_BOX`.
