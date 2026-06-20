# Navigation Behaviour Tree — Structure Reference

## How these fit together

```
Task trees  (one per challenge: HRI / P&P / GPSR)
    └─ call shared Subtrees  (APPROACH_PERSON, DOOR_APPROACH_AND_CROSS, etc.)
           └─ call Custom Nodes  (Bucket B: leaves with real logic)
                  └─ call Action Servers  (Bucket A: Nav2 /navigate_to_pose, /spin, etc.)
```

- **Task trees** are the entry point selected at runtime by the task manager.
- **Subtrees** are reusable compositions — XML only, no new C++ code.
- **Custom Nodes** are C++ leaves (thin wrappers + real-logic nodes).
- **Action Servers** (Nav2 built-ins) are called by thin wrapper leaves, never merged into your tree.
- `→` = Sequence (all must succeed),  `?` = Fallback (try until one succeeds),  `leaf` = action/condition node.

---

## 1 — Shared Subtrees

### 1a. APPROACH_PERSON
```mermaid
flowchart TD
    AP["APPROACH_PERSON\n(Subtree)"]
    AP --> S1["→ Sequence"]
    S1 --> CP["ComputeApproachPose\n(custom leaf)\nstandoff 0.8–1.2 m"]
    S1 --> N2P["NAV_TO_POSE\n(wrapper → /navigate_to_pose)"]
    S1 --> RTF["RotateToFace\n(custom leaf)\nfine-rotate to face person"]
```

### 1b. FOLLOW_PERSON  *(single async leaf — not a subtree)*
```mermaid
flowchart TD
    FP["FOLLOW_PERSON\n(Custom Leaf — async)\nPID loop → /cmd_vel\ndistance + angular velocity\nruns until person lost or task done"]
```

### 1c. REACQUIRE_PERSON
```mermaid
flowchart TD
    RP["REACQUIRE_PERSON\n(Subtree)"]
    RP --> F1["? Fallback"]
    F1 --> S1["→ Sequence: re-find"]
    F1 --> LOST["Publish /nav/lost_person\n→ how???"]
    S1 --> ROT["ROTATE_IN_PLACE\n(wrapper → /spin)\n360° scan"]
    S1 --> CTF["CheckTargetFound\n(custom leaf)\nscans for last-known track ID"]
    S1 --> RESUME["NAV_TO_POSE\n(wrapper)\nresume to re-found pose"]
```

### 1d. DOOR_APPROACH_AND_CROSS
```mermaid
flowchart TD
    DA["DOOR_APPROACH_AND_CROSS\n(Subtree)"]
    DA --> S1["→ Sequence"]
    S1 --> N1["NAV_TO_POSE\n1.0 m standoff"]
    S1 --> WD["WaitForDoorOpen\n(custom leaf)\nLiDAR gap / depth threshold"]
    S1 --> N2["NAV_TO_POSE\ncross threshold (0.5m after the door)"]
    S1 --> VC["VoxelCheck\n(condition leaf)\nassert footprint cleared"]
```
VoxelCheck — a condition leaf you run after crossing. The voxel layer (your 3D depth costmap) can still see ghost inflation from the door frame edges, the door itself if it swung into the path, or a person who stepped into the doorway during crossing. VoxelCheck asserts that the footprint at the robot's current pose is actually clear in the voxel layer before the sequence considers the door crossing done. If it's not clear, the Sequence fails and your recovery Fallback (one level up) handles it — e.g. back up and retry. Without this check you could proceed into the next task while the costmap still thinks you're partially inside an obstacle, which breaks subsequent planning.


### 1e. PRECISE_PLACEMENT_APPROACH
```mermaid
flowchart TD
    PPA["PRECISE_PLACEMENT_APPROACH\n(Subtree)"]
    PPA --> S1["→ Sequence"]
    S1 --> GFP["GetFurniturePose\n(custom leaf)\nlookup KB: dishwasher / bin / cabinet / table"]
    S1 --> CApp["ComputeApproachPose\n(custom leaf)\nper-furniture offset + angle\n dishwasher: 45° side\n bin: frontal top-drop\n cabinet / table: frontal"]
    S1 --> N2P["NAV_TO_POSE\n(wrapper → /navigate_to_pose)"]
    S1 --> DOCK["DOCK\n(wrapper → /opennav_docking)\n±5 cm precision, optional"]
```

### 1f. ROOM_SCAN
```mermaid
flowchart TD
    RS["ROOM_SCAN\n(Subtree)"]
    RS --> S1["→ Sequence"]
    S1 --> GV["GetViewpoints\n(custom leaf)\n4–6 coverage poses for room"]
    S1 --> NTP["NAV_THROUGH_POSES\n(wrapper → /navigate_through_poses)"]
    S1 --> F1["? Fallback (at each stop)"]
    F1 --> CTF["CheckTargetFound\n(custom leaf)\npublish found pose"]
    F1 --> CONT["→ continue to next viewpoint"]
    S1 --> FAIL["Report failure if all\nviewpoints exhausted"]
```

---

## 2 — Task Trees

### 2a. HRI Task Tree
```mermaid
flowchart TD
    HRI["HRI_TASK\n(Task Tree — entry point)"]
    HRI --> S_ROOT["→ Root Sequence"]

    S_ROOT --> ENTER["DOOR_APPROACH_AND_CROSS\n(subtree 1d)"]
    S_ROOT --> WAIT_BELL["WaitForDoorbell\n(custom leaf)\n/audio or guest-at-door flag"]

    S_ROOT --> GUEST_LOOP["→ Sequence ×2 guests"]
    GUEST_LOOP --> AP1["APPROACH_PERSON\n(subtree 1a)"]
    GUEST_LOOP --> HRI_GREET["[HRI] Greet + learn name & drink\n(HRI module — not nav)"]
    GUEST_LOOP --> NAV_SEAT["NAV_TO_POSE\nguide to living room"]
    GUEST_LOOP --> SEAT["[HRI] Offer seat + introduce\n(HRI module)"]

    S_ROOT --> BAG["→ Sequence: bag task"]
    BAG --> GRAB["[Manip] Grab bag\n(manipulation module)"]
    BAG --> HOST_AP["APPROACH_PERSON\nhost in living room"]
    BAG --> FOLLOW_HOST["? Fallback: follow host"]
    FOLLOW_HOST --> FP["FOLLOW_PERSON\n(custom leaf 1b)"]
    FOLLOW_HOST --> RA["REACQUIRE_PERSON\n(subtree 1c)"]
    BAG --> DROP["[Manip] Drop bag\n(manipulation module)"]
```

### 2b. P&P Task Tree
```mermaid
flowchart TD
    PNP["PP_TASK\n(Task Tree — entry point)"]
    PNP --> S_ROOT["→ Root Sequence"]

    S_ROOT --> ENTER["DOOR_APPROACH_AND_CROSS\n(subtree 1d)"]
    S_ROOT --> NAV_TABLE["NAV_TO_POSE\ndining table"]

    S_ROOT --> MULTI["MULTI_DEST_SEQUENCER\n(custom leaf / planner)\ncluster objects → greedy visit order"]

    MULTI --> OBJ_LOOP["→ Per-object loop"]
    OBJ_LOOP --> PERCEIVE["[Perception] Detect + classify object"]
    OBJ_LOOP --> PPA_DEST["PRECISE_PLACEMENT_APPROACH\n(subtree 1e)\ndishwasher / bin / cabinet / table"]
    OBJ_LOOP --> PICK["[Manip] Pick object"]
    OBJ_LOOP --> PPA_PLACE["PRECISE_PLACEMENT_APPROACH\n(subtree 1e)\nplace destination"]
    OBJ_LOOP --> PLACE["[Manip] Place object"]

    S_ROOT --> BREAKFAST["→ Sequence: set breakfast"]
    BREAKFAST --> PPA_B["PRECISE_PLACEMENT_APPROACH\n(subtree 1e)\nbreakfast area on table"]
    BREAKFAST --> PLACE_B["[Manip] Place bowl, spoon,\ncereal, milk"]
```

### 2c. GPSR Task Tree
```mermaid
flowchart TD
    GPSR["GPSR_TASK\n(Task Tree — entry point)"]
    GPSR --> S_ROOT["→ Root Sequence"]

    S_ROOT --> ENTER["DOOR_APPROACH_AND_CROSS\n(subtree 1d)"]
    S_ROOT --> NAV_IP["NAV_TO_POSE\nInstruction Point"]

    S_ROOT --> CMD_RECV["[HRI] Receive commands\n1 or 3 at once"]

    S_ROOT --> PLAN["INTERLEAVED_PATH_PLANNER\n(custom leaf)\nscore 3!=6 orderings by Nav2 path cost\npick min-cost order"]

    S_ROOT --> CMD_LOOP["→ Execute each command"]
    CMD_LOOP --> F_CMD["? Fallback per command"]
    F_CMD --> NAV_CMD["NAV_TO_POSE\ngo to target location"]
    F_CMD --> RS["ROOM_SCAN\n(subtree 1f)\nif target location unknown"]

    CMD_LOOP --> F_ACTION["? Fallback: action at target"]
    F_ACTION --> AP_CMD["APPROACH_PERSON\n(subtree 1a)\nif command involves a person"]
    F_ACTION --> PPA_CMD["PRECISE_PLACEMENT_APPROACH\n(subtree 1e)\nif command involves an object"]
    F_ACTION --> MANIP["[Manip] Pick / Place / Pour\n(manipulation module)"]

    S_ROOT --> RTN["NAV_TO_POSE\nreturn to Instruction Point"]
```

---

## 3 — Custom Node Catalogue (Bucket B)

```mermaid
flowchart TD
    subgraph WRAPPERS["Thin Wrappers — ~10 lines C++ each"]
        W1["NAV_TO_POSE\n→ /navigate_to_pose"]
        W2["NAV_THROUGH_POSES\n→ /navigate_through_poses"]
        W3["ROTATE_IN_PLACE\n→ /spin"]
        W4["DOCK\n→ /opennav_docking"]
        W5["STOP\n→ /cmd_vel zero"]
    end

    subgraph LOGIC["Real-Logic Leaves — own C++ logic"]
        L1["IsPersonVisible\ncondition: person_tf present?"]
        L2["ComputeApproachPose\nstandoff geometry"]
        L3["ComputeStandoffPose\nused by approach + follow"]
        L4["GetFurniturePose\nKB lookup: dishwasher/bin/cabinet"]
        L5["RotateToFace\nin-place heading to target"]
        L6["CheckTargetFound\nscans for track ID in perception feed"]
        L7["GetViewpoints\ncoverage pose generator for ROOM_SCAN"]
        L8["WaitForDoorOpen\nLiDAR gap / depth threshold poll"]
        L9["FOLLOW_PERSON\nasync PID → /cmd_vel"]
        L10["INTERLEAVED_PATH_PLANNER\npath-cost scorer for GPSR ordering"]
        L11["MULTI_DEST_SEQUENCER\ngreedy clustering for P&P destinations"]
    end
```

---

## 4 — Action Servers (Bucket A — Nav2 built-ins, never merged into your tree)

```mermaid
flowchart LR
    subgraph NAV2["Nav2 Action Servers — called by wrapper leaves"]
        A1["/navigate_to_pose"]
        A2["/navigate_through_poses"]
        A3["/spin"]
        A4["/backup"]
        A5["/wait"]
        A6["/opennav_docking\nSimpleNonChargingDock\nuse_external_detection_pose"]
    end
    subgraph CTRL["BT Control Nodes — from BT.CPP v3, no code"]
        C1["Sequence →\nall children must succeed"]
        C2["Fallback ?\ntry until one succeeds"]
        C3["RetryUntilSuccessful\nfor recovery loops"]
        C4["Inverter\ncondition negation"]
    end
```
