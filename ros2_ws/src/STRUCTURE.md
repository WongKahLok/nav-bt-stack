# RoboCup@Home — Behaviour Tree: structure & build plan

**Stack:** ROS 2 Humble · Nav2 · BehaviorTree.CPP **v3** · hand-written XML (no Groot)
**Owner:** kahlok (Navigation BT)

This doc is the map from the runnable Phase-1 tree in this package to the full
planned trees in your diagrams. Read it top to bottom once; after that it's a
reference.

---

## 0. The mental model (how a BT.CPP program is wired)

Three things, and only three:

1. **Leaf nodes** — C++ classes with the real logic. One class per box that
   *does* something (drives the base, checks a condition, runs a PID loop).
   This is where ~all your engineering effort goes.
2. **The XML tree** — pure arrangement. It says "tick `NavToPose`, and if it
   succeeds tick `WaitForDoorOpen`…". It contains **no logic**, just structure.
   You can change the whole strategy by editing XML, no recompile.
3. **The executor** (`bt_main.cpp`) — registers every leaf class in a
   `BehaviorTreeFactory` (mapping XML tag → C++ class), loads the XML, and
   **ticks** the tree in a loop.

A "tick" is one top-down pass. Control nodes route the tick; leaves return
`SUCCESS`, `FAILURE`, or `RUNNING`. A leaf that takes time (any navigation goal)
returns `RUNNING` and gets re-ticked next loop — it must **never block**.

```
  initial_tree.xml ──load──► BehaviorTreeFactory ──tick loop──► leaves ──goals──► Nav2 servers
        (data)                  (tag → C++ class)                 (C++)            (separate processes)
```

---

## 1. Build & run the Phase-1 tree

```bash
# from your colcon workspace root, with the package under src/robocup_bt
colcon build --packages-select robocup_bt
source install/setup.bash

# make sure Nav2 + your map/localisation are already up, then:
ros2 run robocup_bt bt_main
# or point it at any other tree:
ros2 run robocup_bt bt_main --ros-args -p bt_xml_path:=/abs/path/to/tree.xml -p tick_hz:=15.0
```

Set the coordinates in `bt_xml/initial_tree.xml` to real, reachable points on
your map. Watch the base drive to pose 1, then pose 2. That round-trip proves
the loop end-to-end and *is* the Phase-1 deliverable ("initial BT XML + a simple
wrapper that calls nav_to_pose").

> Note: this was written against the Humble + BT.CPP v3 API but not compiled in
> the authoring environment — do a `colcon build` and fix any include/name
> drift for your exact distro before relying on it.

---

## 2. Directory layout (the planned shape)

```
robocup_bt/
├── package.xml
├── CMakeLists.txt
├── include/robocup_bt/
│   ├── nav_to_pose.hpp            # [B-thin] DONE   wrapper -> /navigate_to_pose
│   ├── nav_through_poses.hpp      # [B-thin] TODO   wrapper -> /navigate_through_poses
│   ├── spin_action.hpp            # [B-thin] TODO   wrapper -> /spin   (ROTATE_IN_PLACE)
│   ├── dock_action.hpp            # [B-thin] TODO   wrapper -> /opennav_docking (optional)
│   ├── compute_approach_pose.hpp  # [B] TODO   standoff math (Sync)
│   ├── get_furniture_pose.hpp     # [B] TODO   knowledge-base lookup (Sync)
│   ├── get_viewpoints.hpp         # [B] TODO   4-6 room coverage poses (Sync)
│   ├── wait_for_door_open.hpp     # [B] TODO   LiDAR gap / depth threshold (Stateful)
│   ├── voxel_check.hpp            # [B] TODO   footprint cleared? (Condition)
│   ├── check_target_found.hpp     # [B] TODO   perception query, publishes pose (Condition)
│   ├── rotate_to_face.hpp         # [B] TODO   fine-rotate to face target (Stateful)
│   ├── follow_person.hpp          # [B] TODO   PID -> /cmd_vel (Stateful)
│   ├── publish_lost_person.hpp    # [B] TODO   one-shot event publisher (Sync)
│   ├── multi_dest_sequencer.hpp   # [B] TODO   greedy visit order (custom control)
│   └── interleaved_path_planner.hpp # [B] TODO min-cost command order (custom control)
├── src/
│   └── bt_main.cpp                # executor (register leaves here)
└── bt_xml/
    ├── initial_tree.xml           # DONE
    ├── subtrees/                  # [C] XML-only composition
    │   ├── door_approach_and_cross.xml
    │   ├── room_scan.xml
    │   ├── approach_person.xml
    │   ├── precise_placement_approach.xml
    │   └── reacquire_person.xml
    └── tasks/
        ├── gpsr_task.xml
        ├── hri_task.xml
        └── pp_task.xml
```

`FollowPerson` shows up as "(subtree)" in the HRI diagram but is really a single
leaf, so it lives as a leaf class; if you ever wrap retry/timeout around it,
promote it to a one-node subtree then.

---

## 3. Leaf inventory — every box in your diagrams

Tags carried over from the architecture doc:
`[A]` = comes from Nav2/BT.CPP (you write nothing) ·
`[B-thin]` = ~one wrapper around a Nav2 action (copy `NavToPose`) ·
`[B]` = your own logic ·
`[C]` = XML arrangement of other boxes (a subtree).

| Box in diagram | Tag | BT.CPP base class | ROS interface |
|---|---|---|---|
| `Sequence`, `Fallback`, `ReactiveFallback`, decorators | [A] | built-in control/decorator | — |
| Nav2 action *servers* (planner, controller, behaviors) | [A] | not a tree node — separate process | — |
| **NavToPose** | [B-thin] | `StatefulActionNode` | action client → `/navigate_to_pose` |
| **NavThroughPoses** | [B-thin] | `StatefulActionNode` | action client → `/navigate_through_poses` |
| **Spin** (ROTATE_IN_PLACE) | [B-thin] | `StatefulActionNode` | action client → `/spin` |
| **Dock** (optional, ±5 cm) | [B-thin] | `StatefulActionNode` | action client → `/opennav_docking` |
| **WaitForDoorOpen** | [B] | `StatefulActionNode` | sub: `/scan` (LiDAR) and/or depth |
| **FollowPerson** | [B] | `StatefulActionNode` | sub: person TF/track · pub: `/cmd_vel` |
| **RotateToFace** | [B] | `StatefulActionNode` | sub: target TF · pub: `/cmd_vel` |
| **ComputeApproachPose** | [B] | `SyncActionNode` | TF lookup (instant) |
| **GetFurniturePose** | [B] | `SyncActionNode` | knowledge-base query |
| **GetViewpoints** | [B] | `SyncActionNode` | map / KB query |
| **PublishLostPerson** | [B] | `SyncActionNode` | pub: `/nav/lost_person` |
| **VoxelCheck** (footprint cleared) | [B] | `ConditionNode` | sub: costmap / voxel layer |
| **CheckTargetFound** | [B] | `ConditionNode` | sub: perception target topic |
| **MultiDestSequencer** | [B] | custom `ControlNode` | — (orders its children) |
| **InterleavedPathPlanner** | [B] | custom `ControlNode` | — (orders its children) |
| **DOOR_APPROACH_AND_CROSS** | [C] | subtree | — |
| **ROOM_SCAN** | [C] | subtree | — |
| **APPROACH_PERSON** | [C] | subtree | — |
| **PRECISE_PLACEMENT_APPROACH** | [C] | subtree | — |
| **REACQUIRE_PERSON** | [C] | subtree | — |
| **ReceiveCommands** `[HRI]` | — | owned by HRI module | you call their service |
| **Pick / Place / Pour** `[Manip]` | — | owned by Manipulation | you call their action |

The last two rows are *not your code* — they're boxes other teammates own. In
your tree they appear as a thin service/action-client leaf (same `NavToPose`
pattern) whose contract you agree with that owner.

---

## 4. The three leaf patterns you'll actually write

You already have pattern (a) — `NavToPose`. The other two are short. Together
these three cover every box above.

### (a) Action-client leaf — for anything that talks to an action server
Done: `nav_to_pose.hpp`. To make `NavThroughPoses`, `Spin`, `Dock`: copy it,
swap the action type + server name, and change what `onStart()` puts in the
goal. The async machinery (onStart/onRunning/onHalted) stays identical.

### (b) Stateful leaf that drives `/cmd_vel` — e.g. FollowPerson

```cpp
#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace robocup_bt {

// [B] FOLLOW_PERSON: PID -> /cmd_vel, runs until the person is lost or the task
// signals done. Same RUNNING-until-resolved contract as NavToPose, but instead
// of an action goal it publishes a velocity each tick.
class FollowPerson : public BT::StatefulActionNode {
public:
  FollowPerson(const std::string& name, const BT::NodeConfiguration& cfg,
               const rclcpp::Node::SharedPtr& node)
  : BT::StatefulActionNode(name, cfg), node_(node) {
    cmd_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    // TODO subscribe to the person pose/track that Perception publishes
    //      (a TF frame, or a topic) and cache the latest in a member.
  }

  static BT::PortsList providedPorts() {
    return { BT::InputPort<std::string>("person_frame", "person", "person TF frame"),
             BT::InputPort<double>("desired_distance", 1.0, "follow distance [m]") };
  }

  BT::NodeStatus onStart() override { /* reset PID state */ return BT::NodeStatus::RUNNING; }

  BT::NodeStatus onRunning() override {
    // 1) get the person's pose in the base frame (TF). If not seen recently:
    //        publishZero(); return BT::NodeStatus::FAILURE;   // -> Fallback to REACQUIRE
    // 2) error = (range - desired_distance, bearing); PID -> linear.x, angular.z
    // 3) cmd_pub_->publish(twist);
    // 4) if the task signals "arrived / done": publishZero(); return SUCCESS;
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override { publishZero(); }   // ALWAYS stop the base when halted

private:
  void publishZero() { cmd_pub_->publish(geometry_msgs::msg::Twist{}); }
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
};

}  // namespace robocup_bt
```

Returning `FAILURE` on "person lost" is the whole trick that lets the parent
`Fallback` switch to `REACQUIRE_PERSON` (see §5). And `onHalted` publishing zero
velocity is non-negotiable — otherwise a halted follow leaves the base creeping.

### (c) Condition leaf — e.g. CheckTargetFound / VoxelCheck

```cpp
#include "behaviortree_cpp_v3/condition_node.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
// #include "your_perception_msgs/msg/target.hpp"   // Perception defines this

namespace robocup_bt {

// [B] CHECK_TARGET_FOUND: instant yes/no. On yes, push the found pose onto the
// blackboard so a later NavToPose can read it. ConditionNode::tick() must be
// cheap and synchronous (no RUNNING).
class CheckTargetFound : public BT::ConditionNode {
public:
  CheckTargetFound(const std::string& name, const BT::NodeConfiguration& cfg,
                   const rclcpp::Node::SharedPtr& node)
  : BT::ConditionNode(name, cfg), node_(node) {
    // TODO subscribe to Perception's target topic; cache latest + a 'found' flag
  }
  static BT::PortsList providedPorts() {
    return { BT::OutputPort<geometry_msgs::msg::PoseStamped>("found_pose") };
  }
  BT::NodeStatus tick() override {
    if (/* have a fresh detection of the wanted target/track id */ false) {
      // setOutput("found_pose", cached_pose_);
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }
private:
  rclcpp::Node::SharedPtr node_;
};

}  // namespace robocup_bt
```

`VoxelCheck` is the same shape with no output port: returns SUCCESS if the
footprint cell(s) in the costmap/voxel layer are clear, else FAILURE.

> The red "Publish /nav/lost_person — how???" box is just pattern (c)'s cousin:
> a one-shot `SyncActionNode` holding a publisher. In its `tick()`:
> `pub_->publish(std_msgs::msg::Empty{}); return SUCCESS;` — pick `std_msgs/Empty`
> for a pure event, or a tiny custom msg if HRI also wants the last-known pose /
> track id. Whether it returns SUCCESS or FAILURE is a design choice you settle
> with the HRI owner (does "lost the host" fail the task, or just notify and
> continue?).

---

## 5. The subtrees ([C] — XML only, no new C++)

These are the boxes that compose leaves. Drop each in `bt_xml/subtrees/`.
Coordinates/poses generally come from a compute leaf into a blackboard key
(`{like_this}`) that a later `NavToPose` reads.

### door_approach_and_cross.xml  (your Phase-2 deliverable)
```xml
<root main_tree_to_execute="DoorApproachAndCross">
  <BehaviorTree ID="DoorApproachAndCross">
    <Sequence>
      <NavToPose      x="{door_standoff_x}" y="{door_standoff_y}" yaw="{door_standoff_yaw}"/> <!-- 1.0 m standoff -->
      <WaitForDoorOpen timeout="30.0"/>                                                        <!-- LiDAR gap / depth -->
      <NavToPose      x="{door_cross_x}"    y="{door_cross_y}"    yaw="{door_cross_yaw}"/>     <!-- 0.5 m past the door -->
      <VoxelCheck/>                                                                            <!-- footprint cleared -->
    </Sequence>
  </BehaviorTree>
</root>
```

### approach_person.xml
```xml
<root main_tree_to_execute="ApproachPerson">
  <BehaviorTree ID="ApproachPerson">
    <Sequence>
      <ComputeApproachPose target_frame="person" standoff="1.0"
                           out_x="{ap_x}" out_y="{ap_y}" out_yaw="{ap_yaw}"/>  <!-- 0.8-1.2 m -->
      <NavToPose    x="{ap_x}" y="{ap_y}" yaw="{ap_yaw}"/>
      <RotateToFace target_frame="person"/>
    </Sequence>
  </BehaviorTree>
</root>
```

### precise_placement_approach.xml
```xml
<root main_tree_to_execute="PrecisePlacementApproach">
  <BehaviorTree ID="PrecisePlacementApproach">
    <Sequence>
      <GetFurniturePose    furniture="{target_furniture}" out_pose="{furn_pose}"/>   <!-- dishwasher/bin/cabinet/table -->
      <ComputeApproachPose furniture_pose="{furn_pose}" mode="{target_furniture}"
                           out_x="{pp_x}" out_y="{pp_y}" out_yaw="{pp_yaw}"/>        <!-- per-furniture offset+angle -->
      <NavToPose x="{pp_x}" y="{pp_y}" yaw="{pp_yaw}"/>
      <Dock dock_id="{target_furniture}"/>                                          <!-- optional ±5 cm park -->
    </Sequence>
  </BehaviorTree>
</root>
```

### room_scan.xml
The diagram is a per-viewpoint loop: get 4-6 poses, then at each stop try
`CheckTargetFound`, else continue; fail if all exhausted. Cleanest in v3 is a
tiny custom `IterateViewpoints` control node that ticks its body once per
viewpoint and reports success on first found / failure when the list runs out.
```xml
<root main_tree_to_execute="RoomScan">
  <BehaviorTree ID="RoomScan">
    <Sequence>
      <GetViewpoints room="{target_room}" out_poses="{viewpoints}"/>   <!-- 4-6 coverage poses -->
      <IterateViewpoints poses="{viewpoints}" current="{vp}">          <!-- custom control, loops body -->
        <Sequence>
          <NavThroughPoses goals="{vp}"/>                              <!-- wrapper -> /navigate_through_poses -->
          <Fallback>
            <CheckTargetFound found_pose="{target_pose}"/>             <!-- publishes found pose -->
            <AlwaysFailure/>                                           <!-- not here -> next viewpoint -->
          </Fallback>
        </Sequence>
      </IterateViewpoints>
    </Sequence>
  </BehaviorTree>
</root>
```
If you'd rather not write `IterateViewpoints` first, hand-unroll 4-6 explicit
`<Sequence>` blocks to get scanning working, then refactor to the loop.

### reacquire_person.xml
```xml
<root main_tree_to_execute="ReacquirePerson">
  <BehaviorTree ID="ReacquirePerson">
    <Fallback>
      <Sequence name="re_find">
        <Spin angle="6.283"/>                       <!-- ROTATE_IN_PLACE 360° -> /spin -->
        <CheckTargetFound found_pose="{refound}"/>  <!-- scan for last-known track id -->
        <NavToPose x="{refound_x}" y="{refound_y}" yaw="{refound_yaw}"/>  <!-- resume to re-found pose -->
      </Sequence>
      <PublishLostPerson/>                          <!-- couldn't re-find -> notify HRI -->
    </Fallback>
  </BehaviorTree>
</root>
```

---

## 6. The task trees ([C]) and how subtrees plug in

Reference a subtree by ID with `<SubTree ID="DoorApproachAndCross"/>`. To pass
a value in/out, remap a port: `<SubTree ID="ApproachPerson" person="{host}"/>`
(or add `__autoremap="true"` to share all same-named blackboard keys).

The executor loads them all, then builds the one you want:
```cpp
factory.registerBehaviorTreeFromFile(".../subtrees/door_approach_and_cross.xml");
factory.registerBehaviorTreeFromFile(".../subtrees/approach_person.xml");
// ... all subtrees ...
factory.registerBehaviorTreeFromFile(".../tasks/gpsr_task.xml");
auto tree = factory.createTree("GpsrTask");   // resolves the SubTree refs by ID
```

### gpsr_task.xml (matches the GPSR diagram)
```xml
<root main_tree_to_execute="GpsrTask">
  <BehaviorTree ID="GpsrTask">
    <Sequence name="root">
      <SubTree ID="DoorApproachAndCross"/>                <!-- enter the door -->
      <NavToPose x="{instr_x}" y="{instr_y}" yaw="{instr_yaw}"/>   <!-- to instruction point -->
      <ReceiveCommands out_commands="{cmds}"/>            <!-- [HRI] 1 or 3 at once -->
      <InterleavedPathPlanner commands="{cmds}" current="{cmd}">   <!-- min path-cost order -->
        <Sequence name="one_command">
          <Fallback name="go_to_target">
            <NavToPose x="{cmd_x}" y="{cmd_y}" yaw="{cmd_yaw}"/>    <!-- known location -->
            <SubTree ID="RoomScan" target_room="{cmd_room}"/>      <!-- else search -->
          </Fallback>
          <Fallback name="act_at_target">
            <SubTree ID="ApproachPerson"/>                         <!-- if command needs a person -->
            <SubTree ID="PrecisePlacementApproach"/>               <!-- if command needs an object -->
          </Fallback>
          <!-- [Manip] Pick / Place / Pour leaf goes here -->
        </Sequence>
      </InterleavedPathPlanner>
      <NavToPose x="{instr_x}" y="{instr_y}" yaw="{instr_yaw}"/>   <!-- return to instruction point -->
    </Sequence>
  </BehaviorTree>
</root>
```

### hri_task.xml (matches the HRI diagram)
```xml
<root main_tree_to_execute="HriTask">
  <BehaviorTree ID="HriTask">
    <Sequence name="root">
      <WaitForDoorbell/>
      <Repeat num_cycles="2">                       <!-- ×2 guests -->
        <Sequence name="receive_guest">
          <SubTree ID="ApproachPerson"/>            <!-- people-approaching feature -->
          <!-- greet + learn name & drink (HRI, not nav) -->
          <NavToPose x="{living_x}" y="{living_y}" yaw="{living_yaw}"/>  <!-- guide to living room -->
          <!-- offer seat + introduce (HRI, not nav) -->
        </Sequence>
      </Repeat>
      <Sequence name="bag_task">
        <!-- [Manip] grab bag -->
        <SubTree ID="ApproachPerson"/>              <!-- host in living room -->
        <Fallback name="follow_host">
          <FollowPerson person_frame="host"/>       <!-- people-following feature -->
          <SubTree ID="ReacquirePerson"/>           <!-- people-reacquiring feature -->
        </Fallback>
        <!-- [Manip] drop bag -->
      </Sequence>
    </Sequence>
  </BehaviorTree>
</root>
```

### pp_task.xml (matches the Pick & Place diagram)
```xml
<root main_tree_to_execute="PpTask">
  <BehaviorTree ID="PpTask">
    <Sequence name="root">
      <SubTree ID="DoorApproachAndCross"/>          <!-- enter the door -->
      <NavToPose x="{dining_x}" y="{dining_y}" yaw="{dining_yaw}"/>  <!-- to dining table -->
      <MultiDestSequencer objects="{objects}" current="{obj}">      <!-- greedy visit order -->
        <Sequence name="per_object">
          <!-- [Perception] detect + classify object -->
          <SubTree ID="PrecisePlacementApproach" target_furniture="{obj_source}"/>
          <!-- [Manip] pick object -->
          <SubTree ID="PrecisePlacementApproach" target_furniture="{obj_dest}"/>
          <!-- [Manip] place object -->
        </Sequence>
      </MultiDestSequencer>
      <Sequence name="set_breakfast">
        <SubTree ID="PrecisePlacementApproach" target_furniture="table"/>
        <!-- [Manip] place bowl, spoon, cereal, milk -->
      </Sequence>
    </Sequence>
  </BehaviorTree>
</root>
```

---

## 7. Suggested order of work (maps to your phase rows)

1. **Phase 1 (now):** ship the runnable initial tree in this package. Done once
   the base patrols the two poses.
2. **Phase 2:** write `WaitForDoorOpen` + `VoxelCheck`, assemble
   `door_approach_and_cross.xml` → that's your "Door crossing (sim)" row. In
   parallel, the DWB/shim tuning row is Nav2 config, not BT code.
3. **Phase 3:** write `FollowPerson`, `RotateToFace`, `ComputeApproachPose`,
   `CheckTargetFound`, `PublishLostPerson`; assemble `approach_person`,
   `reacquire_person`, and the dynamic-obstacle behaviour. Then stitch the task
   trees. This is the "People approaching/following" + "BT integration" rows.

Adding a behaviour is always the same two steps: **(1)** write+register the leaf
class, **(2)** drop it into an XML tree. Layout changes never need a recompile —
only new leaf *classes* do.
