# RoboCup@Home — Navigation Pipeline Architecture
**Scope:** HRI · Pick & Place · GPSR  
**Stack:** ROS 2 Humble · Nav2 · SLAM Toolbox · AMCL · BT (BehaviorTree.CPP)

---

## 1. System Overview

```
┌─────────────────────────────────────────────────────────────────────┐          
│                         SENSOR LAYER                                │      
│                                                                     │
│   SLAMTEC RPLIDAR S3          RealSense D455 / D405                 │
│         │                           │                               │
│     LaserScan                  Depth Image ──► PointCloud           │
│         │                           │           (depth_image_proc)  │
└─────────┼───────────────────────────┼───────────────────────────────┘
          │                           │
          ▼                           ▼
┌─────────────────────────────────────────────────────────────────────┐   ________________________________________________________________
│                MAPPING and LOCALISATION LAYER                       │   |                      TF2 Transformation                      |
│                                                                     │   |                                                              |
│   SLAM Toolbox ──► 2D Occupancy Grid                                |   |map  ──►  odom  ──►  base_link  ──►  sensors (lidar, camera)  |
|                                                                     |   | |         └── published by: robot_localization EKF           |
│                                                                     |   | │                             (fuses wheel odom + IMU)       |
|                                                                     |   | └── published by: AMCL                                       |
|                                                                     |   |     (corrects the drifting odom → map transform)             |  
│   Static TF2 publishers                                             |   |                                                              |            
│           +               -->   AMCL   --> Navigation node          |   |______________________________________________________________|                        
│   robot_localisation EKF                     & costmap              |          
| (fusion of wheel encoder + IMU)                                     |   
|            +                                                        |   
|     2D Occupancy Grid, i.e. map                                     |    
│                                                                     |  
└──────────────────────────┬──────────────────────────────────────────┘
                           │
                           ▼
┌─────────────────────────────────────────────────────────────────────┐
│                       COSTMAP LAYER                                 │
│                                                                     │
│   Static Map ──┐                                                    │
│                ├──► Costmap (Obstacle Layer)                        │
│   LiDAR scan ──┤    · Global costmap  (pre-mapped, static)          │
│                │    · Local costmap   (real-time, 360° sensor)      │
│   Depth PC  ───┘    · Voxel layer     (3D obstacles / narrow gaps)  │
└──────────────────────────┬──────────────────────────────────────────┘
                           │
                           ▼
┌─────────────────────────────────────────────────────────────────────┐
│                       PLANNING LAYER                                │
│                                                                     │
│            Nav2 Stack                                               │
│         ┌──────────────────────────────┐                           │
│         │  Global Plan                 │                           │
│         │  A* / SmacPlannerHybrid      │                           │
│         │  (omni-kinematics footprint) │                           │
│         ├──────────────────────────────┤                           │
│         │  Local Plan                  │                           │
│         │  DWB Controller + Shim       │                           │
│         │  (915 mm door clearance)     │                           │
│         ├──────────────────────────────┤                           │
│         │  Cmd Vel                     │                           │
│         │  Omni Base Driver            │                           │
│         └──────────────────────────────┘                           │
└──────────────────────────┬──────────────────────────────────────────┘
                           │
                           ▼
┌─────────────────────────────────────────────────────────────────────┐
│                    BEHAVIOUR LAYER  (BT + FSM)                      │
│                   Task Manager (Custom Node)                        │
│                                                                     │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────────────────┐  │
│  │  HRI Module  │  │  Nav Module  │  │  Manipulation Interface  │  │
│  └──────────────┘  └──────────────┘  └──────────────────────────┘  │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 2. Navigation Behaviours — What Comes From Where

For every box in the tree, ask ONE question: **"Does a Nav2 program already do
this, or do I have to write it?"**  That splits everything into three buckets:

  Bucket A  — comes from Nav2 / BT.CPP        → you write NO logic
  Bucket B  — custom C++ nodes                → you write these
  Bucket C  — subtrees & task trees (XML)     → no code, just arrangement

Labels used below:  [A] = from Nav2/BT.CPP   [B] = you write it
[B-thin] = ~10-line wrapper around a Nav2 server   [C] = XML arrangement of boxes

### 2.1 Bucket A — Comes from Nav2 / BT.CPP (no code to write)

**Action servers** — separate running programs. They are NOT boxes in your tree;
your tree sends REQUESTS to them.

```
/navigate_to_pose        plan + drive to one pose
/navigate_through_poses  plan + drive through a list of poses
/spin                    rotate in place
/backup                  drive backward
/wait                    pause
/dock_robot              precision approach to a detected target  (docking server)
```

  KEY IDEA: Nav2 is ITSELF a behaviour tree. When you call /navigate_to_pose,
  Nav2 runs its OWN internal tree (ComputePathToPose -> FollowPath -> recoveries).
  You do NOT merge your task tree into it — you call it as a black box.

**Control & decorator nodes** — the structural glue. Built-in, free, no code.

```
Sequence              run children in order, all must succeed
Fallback              try children until one succeeds   <- the recovery pattern
Parallel              run children at the same time
RecoveryNode          pair an action with a recovery    (Nav2-provided)
PipelineSequence      re-tick earlier children as later ones run  (Nav2-provided)
RetryUntilSuccessful / Timeout / Inverter    decorators
```

### 2.2 Bucket B — Custom nodes you write (C++)

**Kind 1 — thin client wrappers** (~10 lines each; the real work happens inside
the Nav2 server from Bucket A):

```
NAV_TO_POSE         -> wraps /navigate_to_pose
NAV_THROUGH_POSES   -> wraps /navigate_through_poses   (P&P multi-dest, GPSR routes)
ROTATE_IN_PLACE     -> wraps /spin
DOCK                -> wraps /dock_robot
STOP                -> cancel goal / publish zero velocity   (trivial)
```

**Kind 2 — real-logic leaves** (Nav2 has nothing for these; you write them fully):

```
IsPersonVisible       condition: is the person TF present?
ComputeStandoffPose   TF math: a pose 0.8-1.2 m in front of a person
ComputeApproachPose   TF math: approach offset for a furniture item
GetFurniturePose      read a furniture pose from the Knowledge Base / TF
RotateToFace          turn to face a target            (or just reuse /spin)
CheckTargetFound      condition: did perception find the target?
GetViewpoints         return the room-scan poses for a room
FOLLOW_PERSON         <- the big one: the PID control loop (see note)
```

  NOTE — FOLLOW_PERSON is ONE async leaf, NOT a subtree. Following is a tight,
  continuous control loop (PID on distance + angular velocity -> /cmd_vel).
  A subtree ticked at ~10 Hz re-issuing discrete goals cannot do smooth velocity
  control. The BT just ticks this single box and reacts to its return status:
  RUNNING while following, SUCCESS when done, FAILURE on track-lost.

### 2.3 Bucket C — Subtrees & task trees (pure XML, no code)

A subtree is NOT a box — it is a NAMED GROUP of boxes from Buckets A and B,
arranged in XML. Zero C++. Each subtree below lists its child boxes with labels.

APPROACH_PERSON                              [HRI, GPSR]
```
Sequence                                     [A]
├── IsPersonVisible(person_tf)               [B]
├── ComputeStandoffPose -> standoff_pose     [B]   0.8-1.2 m in front of person
├── NAV_TO_POSE(standoff_pose)               [B-thin -> /navigate_to_pose]
└── RotateToFace(person_tf)                  [B]
```

FOLLOW_PERSON  (with recovery)               [HRI — host follow]
```
FOLLOW_PERSON itself is the single Bucket-B leaf above — NOT a subtree.
You wrap it together with its recovery in XML:

RetryUntilSuccessful                         [A]
└── Fallback                                 [A]
    ├── FOLLOW_PERSON(person_tf)             [B]   returns FAILURE on track-lost
    └── REACQUIRE_PERSON(person_tf)          [C — subtree below]
```

REACQUIRE_PERSON                             [HRI — recovery]
```
Fallback                                     [A]
├── Sequence                                 [A]
│   ├── STOP()                               [B]
│   ├── ROTATE_IN_PLACE(360°)                [B-thin -> /spin]   scan for last-known ID
│   └── IsPersonVisible(person_tf)           [B]                 -> resume following
└── RequestAssistance                        [B]   publish /nav/lost_person
                                                   -> HRI module runs the dialogue
```

DOOR_APPROACH_AND_CROSS                      [HRI — entrance/exit]
```
Sequence                                     [A]
├── NAV_TO_POSE(door_approach_pose)          [B-thin]   1.0 m in front of door
├── WaitForDoorOpen(door_id)                 [B]        door state from depth / LiDAR gap
│      (or trigger an OpenDoor manipulation action instead)
├── NAV_TO_POSE(door_through_pose)           [B-thin]   1.0 m past the threshold
└── (footprint clearing the door frame is handled by the voxel layer in the costmap)
```

PRECISE_PLACEMENT_APPROACH                   [P&P — dishwasher, cabinet, bin]
```
Sequence                                     [A]
├── GetFurniturePose(furniture_id)           [B]   from Knowledge Base
├── ComputeApproachPose -> approach_pose     [B]   nominal + per-furniture offset
│      (dishwasher: side-angle · cabinet: frontal · bin: top-angle)
├── NAV_TO_POSE(approach_pose)               [B-thin]   base parks "close enough"
└── DOCK(target)  OR  arm-side fine-align    [B-thin -> /dock_robot]   to within ~5 cm
```
  WHERE THE PRECISION LIVES: in the BASE (docking server) OR in the ARM. If the
  arm has reach margin + an eye-in-hand camera, the base parks loosely and the
  ARM closes the last few cm via its pre-grasp approach — simpler and more robust.

ROOM_SCAN                                    [GPSR — find object/person in a known room]
```
Sequence                                     [A]
├── GetViewpoints(room_id)                   [B]   PREDEFINED poses per room
│      (hand-annotated during Setup Days — the arena is KNOWN, so no
│       frontier exploration is needed; that is only for unknown maps)
└── loop over viewpoints, early-exit on hit  [A decorator]
    ├── NAV_TO_POSE(viewpoint)               [B-thin]
    └── CheckTargetFound                     [B]   trigger perception; stop when found
```

**Task trees** (HRI / P&P / GPSR) are also Bucket C — arrangements of the subtrees
above. Their flows are in Section 3.

**NOT in the tree at all — SOCIAL_NAV_INFLATE:**
```
This is NOT a behaviour and NOT a BT box. It is a COSTMAP LAYER that inflates
cost ~0.85 m around detected people. Run it as a standalone always-on node (or a
costmap plugin) fed by YOLOv11 person detections. Toggling costmap layers mid-run
via a service is finicky — prefer leaving it on, and gate "whether you care" in
context rather than flipping the layer from the tree.
```

---

## 3. Per-Challenge Pipeline

### 3.1 HRI — Human Robot Interaction Challenge

```
START (outside arena)
    │
    ▼
DOOR_APPROACH_AND_CROSS(entrance)
    │   [optional: trigger door open for +200 pts × 2]
    ▼
NAV_TO_POSE(start_position, near seating area)
    │
    ▼
    ┌─────────────── GUEST LOOP (×2 guests) ──────────────┐
    │                                                      │
    │  WAIT for doorbell / timer                           │
    │      │                                               │
    │      ▼                                               │
    │  APPROACH_PERSON(guest_tf)                           │
    │      │  [guest detected by YOLOv11 at door]          │
    │      ▼                                               │
    │  HRI: greet, get name + drink                        │
    │      │                                               │
    │      ▼                                               │
    │  SOCIAL_NAV_INFLATE(true)                            │
    │  NAV_TO_POSE(living_room_seating)                    │
    │      │  [look in direction of nav — gaze module]     │
    │      ▼                                               │
    │  Offer seat, introduce guests                        │
    │                                                      │
    └──────────────────────────────────────────────────────┘
    │
    ▼  [Guest 2 only — bag handover + host follow]
HRI: receive bag from guest 2
    │
    ▼
APPROACH_PERSON(host_tf)
    │
    ▼
FOLLOW_PERSON(host_tf)
    │   [host walks to random drop location]
    │   [REACQUIRE_PERSON() on track loss]
    ▼
Drop bag at host-designated location
    │
    ▼
END

Key costmap config for HRI:
  · Social inflation layer: ON throughout guest escort
  · Local costmap radius: 3 m  (living room furniture density)
  · DWB critics: ObstacleFootprint + GoalAlign + PathAlign
```

### 3.2 P&P — Pick and Place Challenge

```
START (outside arena)
    │
    ▼
NAV_TO_POSE(kitchen_entry)
    │
    ▼
PRECISE_PLACEMENT_APPROACH(dining_table, frontal_0.5m)
    │   Perception: identify all 6 objects, classify destinations
    ▼
    ┌──────────── MULTI-DEST SEQUENCER ─────────────────────┐
    │                                                        │
    │  Input: object list + destination map                  │
    │  Algorithm: greedy nearest-destination clustering      │
    │             group by destination, minimise base moves  │
    │                                                        │
    │  Typical sequence (optimised):                         │
    │  1. Cutlery + tableware  → dishwasher                  │
    │  2. Trash item(s)        → bin                         │
    │  3. Other objects        → cabinet                     │
    │  4. Breakfast items      → dining table (clear area)   │
    │                                                        │
    │  For each pick-place cycle:                            │
    │  ┌──────────────────────────────────────────────────┐  │
    │  │ PRECISE_PLACEMENT_APPROACH(source_furniture)     │  │
    │  │     → Manipulation: pick object                  │  │
    │  │ PRECISE_PLACEMENT_APPROACH(dest_furniture)       │  │
    │  │     → Manipulation: place object                 │  │
    │  └──────────────────────────────────────────────────┘  │
    └────────────────────────────────────────────────────────┘
    │
    ▼
END

Furniture approach offsets (tuned values):
  · Dishwasher  : side-angle 45°, 0.4 m clearance (narrow space)
  · Cabinet     : frontal,    0.5 m, shelf-level alignment
  · Trash bin   : frontal,    0.4 m, top-down drop clearance
  · Dining table: frontal,    0.6 m (arm reach + safe place margin)

Obstacle avoidance notes:
  · Chairs around dining table → inflated as obstacles in local costmap
  · Robot navigates around chairs (no human assistance request)
  · Voxel layer active: detects table legs, rack edges, dishwasher door
```

### 3.3 GPSR — General Purpose Service Robot Challenge

```
START (outside arena)
    │
    ▼
NAV_TO_POSE(instruction_point)
    │
    ▼
    ┌──────────── COMMAND LOOP (×3 commands) ───────────────┐
    │                                                        │
    │  RECEIVE command from operator                         │
    │  PARSE → extract: action, object, location, person     │
    │                                                        │
    │  ─── INTERLEAVED PLANNER (if all 3 given at once) ─── │
    │  │  Input: 3 command goal-poses                        │
    │  │  Evaluate: TSP-lite over goal sequence              │
    │  │    · compute all 3! = 6 orderings                   │
    │  │    · score each by total path cost (Nav2 planner)   │
    │  │    · select minimum-cost ordering                   │
    │  │  Execute in optimised order                         │
    │  │  → award interleaved bonus (+200 pts)               │
    │  ────────────────────────────────────────────────────  │
    │                                                        │
    │  FOR EACH command:                                     │
    │  ┌──────────────────────────────────────────────────┐  │
    │  │                                                  │  │
    │  │  Is goal a KNOWN LOCATION?                       │  │
    │  │    YES → NAV_TO_POSE(known_pose)                 │  │
    │  │    NO  → ROOM_SCAN(target_room)                  │  │
    │  │                                                  │  │
    │  │  Is goal a PERSON?                               │  │
    │  │    YES → APPROACH_PERSON(person_tf)              │  │
    │  │          HRI: dialogue                           │  │
    │  │                                                  │  │
    │  │  Is goal an OBJECT?                              │  │
    │  │    YES → PRECISE_PLACEMENT_APPROACH(furniture)   │  │
    │  │          Manipulation: pick / place              │  │
    │  │                                                  │  │
    │  └──────────────────────────────────────────────────┘  │
    │                                                        │
    │  NAV_TO_POSE(instruction_point)   [if one-by-one mode] │
    └────────────────────────────────────────────────────────┘
    │
    ▼
END
```

---

## 4. Recovery Behaviours (all challenges)

```
FAILURE EVENT                   RECOVERY ACTION
─────────────────────────────   ────────────────────────────────────────
Nav goal unreachable            Clear costmap → retry with relaxed tolerance
                                If still fails → NAV_TO_POSE(nearest clear pose)

Stuck / no progress > 5 s      Execute escape manoeuvre:
                                  ROTATE_IN_PLACE(180°) → back up 0.3 m → retry

Person lost during follow       REACQUIRE_PERSON() — see §2.3
(FOLLOW_PERSON track loss)        rotate scan → perception re-ID → resume

Door blocked / not open         Wait 5 s → re-check → publish to HRI module
                                HRI module requests door open via dialogue

Localisation diverged           Trigger AMCL reinitialisation at last known pose
(AMCL covariance > threshold)   ROTATE_IN_PLACE(360°) to gather LiDAR features

Object not found in ROOM_SCAN   Report failure to Task Manager
                                Task Manager: skip subtask, continue next command
```

---

## 5. Topic & Interface Map

```
PUBLISHERS (nav → other modules)
  /nav/goal_status          → Task Manager  (reached / failed / in_progress)
  /nav/lost_person          → HRI Module    (triggers recovery dialogue)
  /nav/current_room         → Task Manager  (room-level awareness)

SUBSCRIBERS (nav ← other modules)
  /perception/person_tf     ← YOLOv11 + DeepSort  (person pose for approach/follow)
  /perception/object_poses  ← YOLOv11-seg          (object locations for approach)
  /hri/follow_start         ← HRI Module           (trigger FOLLOW_PERSON)
  /hri/follow_stop          ← HRI Module           (stop following, place bag)
  /task/nav_goal            ← Task Manager         (BT sends goal_pose or behaviour ID)

SHARED
  /tf                       SLAM Toolbox + AMCL → all nodes
  /map                      SLAM Toolbox         → Nav2 global costmap
  /scan                     RPLIDAR S3           → SLAM + local costmap
  /camera/depth/points      RealSense            → voxel layer + PointCloud
```

---

## 6. Configuration Reference

```
COMPONENT               KEY PARAMETERS
──────────────────────  ────────────────────────────────────────────────
SmacPlannerHybrid       minimum_turning_radius: 0.0   (omni → 0)
                        allow_unknown: true
                        use_reeds_shepp: false

DWB Controller          min_vel_x / min_vel_y: –0.3 m/s   (omni)
+ Shim                  max_vel_theta: 1.0 rad/s
                        footprint: adjusted for 915 mm door pass

Local Costmap           width / height: 3.0 m
                        inflation_radius: 0.35 m
                        social_inflation_radius: 0.85 m  (toggle)

Global Costmap          update_frequency: 1.0 Hz
                        publish_frequency: 0.5 Hz
                        plugins: [static, obstacle, inflation]

AMCL                    min_particles: 500
                        max_particles: 2000
                        laser_max_range: 12.0 m   (RPLIDAR S3 spec)

EKF (robot_localization) sensor fusion: wheel odom + IMU
                        output frame: odom → map via AMCL
```

---

## 7. Phase Mapping to Roadmap

```
ROADMAP PHASE    NAVIGATION DELIVERABLE
───────────────  ─────────────────────────────────────────────────────
Phase 1 (Sim)    · SLAM Toolbox map of Gazebo apartment
                 · EKF + AMCL running in simulation
                 · Dummy /perception/person_tf publisher for testing

Phase 2 (Sim)    · SmacPlannerHybrid + DWB+Shim tuned
                 · 915 mm door clearance validated in sim
                 · Global + local + voxel costmaps configured

Phase 2 (Real)   · EKF + AMCL deployed on physical chassis
                 · Physical table-to-table transit validated
                 · PRECISE_PLACEMENT_APPROACH offsets measured

Phase 3 (Sim)    · BT XML for HRI: DOOR_APPROACH, APPROACH_PERSON,
                   FOLLOW_PERSON, REACQUIRE_PERSON
                 · ROOM_SCAN sub-tree for GPSR
                 · SOCIAL_NAV_INFLATE toggle integrated

Phase 3 (Real)   · Swap mock /perception/person_tf for real detections
                 · Social inflation layer validated with real people
                 · MULTI-DEST SEQUENCER validated (P&P full run)

Phase 4 (Real)   · Full HRI scenario end-to-end
                 · Full P&P scenario end-to-end
                 · Full GPSR scenario with interleaved planner
                 · BT recovery behaviours stress-tested
```

---

*Document version: June 2026 — aligned with RoboCup@Home Rulebook 2026 Rev-1*
