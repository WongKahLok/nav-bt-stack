# C++ & ROS 2 Beginner's Guide
### Using `nav_bt_stack` as a Living Example

> **Who is this for?**  
> You lead the navigation team but are new to C++. This guide walks you through every major C++ concept you will encounter in `nav_bt_stack`, using the **actual project files** as examples. Read it top-to-bottom the first time; later use it as a reference.

---

## Table of Contents

1. [How C++ Files Are Organized](#1-how-c-files-are-organized)
2. [The `#include` System](#2-the-include-system)
3. [Include Guards — Preventing Double-Inclusion](#3-include-guards--preventing-double-inclusion)
4. [Namespaces — Avoiding Name Clashes](#4-namespaces--avoiding-name-clashes)
5. [Variables, Types, and `const`](#5-variables-types-and-const)
6. [Functions](#6-functions)
7. [Classes — The Heart of Object-Oriented C++](#7-classes--the-heart-of-object-oriented-c)
8. [Inheritance — Reusing and Extending Classes](#8-inheritance--reusing-and-extending-classes)
9. [Constructors and Member Initializer Lists](#9-constructors-and-member-initializer-lists)
10. [Access Specifiers: `public`, `private`, `protected`](#10-access-specifiers-public-private-protected)
11. [`static` Members and Methods](#11-static-members-and-methods)
12. [`override` — Safe Virtual Method Overriding](#12-override--safe-virtual-method-overriding)
13. [Smart Pointers — Memory Without `delete`](#13-smart-pointers--memory-without-delete)
14. [Templates — Generic Code](#14-templates--generic-code)
15. [Type Aliases with `using`](#15-type-aliases-with-using)
16. [STL Containers: `std::vector`, `std::map`, `std::string`](#16-stl-containers-stdvector-stdmap-stdstring)
17. [Lambda Functions — Inline Anonymous Functions](#17-lambda-functions--inline-anonymous-functions)
18. [References (`&`) and `const` References](#18-references--and-const-references)
19. [`auto` — Let the Compiler Deduce the Type](#19-auto--let-the-compiler-deduce-the-type)
20. [Range-Based `for` Loops](#20-range-based-for-loops)
21. [`static_cast` — Explicit Type Conversion](#21-static_cast--explicit-type-conversion)
22. [Exception Handling: `try` / `catch`](#22-exception-handling-try--catch)
23. [Brace Initialization of Member Variables](#23-brace-initialization-of-member-variables)
24. [The Build System: CMakeLists.txt and package.xml](#24-the-build-system-cmakeliststxt-and-packagexml)
25. [ROS 2 Concepts in C++](#25-ros-2-concepts-in-c)
26. [BehaviorTree.CPP Patterns](#26-behaviortreecpp-patterns)
27. [Putting It All Together — Reading `bt_main.cpp`](#27-putting-it-all-together--reading-bt_maincpp)
28. [Quick-Reference Cheat Sheet](#28-quick-reference-cheat-sheet)

---

## 1. How C++ Files Are Organized

C++ code is split into two kinds of files:

| File type | Extension | What goes inside |
|---|---|---|
| **Header** | `.hpp` or `.h` | Class/function **declarations** (the "what") |
| **Source** | `.cpp` | Function/method **definitions** (the "how") |

In `nav_bt_stack` every node class lives entirely in a header (`.hpp`) because the classes are small enough. Only the entry-point lives in a `.cpp`:

```
include/nav_bt_stack/
    nav_to_pose.hpp        ← class NavToPose declared AND defined here
    back_up.hpp
    interleaved_planner.hpp
    ...
src/
    bt_main.cpp            ← main() function — includes all the headers
```

**Why headers?**  
When another file does `#include "nav_bt_stack/nav_to_pose.hpp"`, the compiler literally pastes the contents of that header in place. This is how it learns about the class.

---

## 2. The `#include` System

```cpp
// From bt_main.cpp
#include <memory>           // standard library (angle brackets)
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"                        // third-party / ROS 2
#include "nav_bt_stack/nav_to_pose.hpp"              // your own project
```

**Rule of thumb:**
- `<angle_brackets>` → standard library or installed libraries (compiler finds them automatically).
- `"double_quotes"` → files inside your own project, or libraries whose path you configured in CMake.

Every header in the project includes *only* what it needs. For example, `nav_to_pose.hpp` pulls in `<chrono>` (for time durations), `<future>` (for async results), and the ROS 2 action headers, but nothing else. This keeps compile times short.

---

## 3. Include Guards — Preventing Double-Inclusion

Every header in the project starts and ends like this:

```cpp
// nav_to_pose.hpp
#ifndef NAV_BT_STACK__NAV_TO_POSE_HPP_   // "if NOT defined"
#define NAV_BT_STACK__NAV_TO_POSE_HPP_   // define it now

// ... all class code ...

#endif  // NAV_BT_STACK__NAV_TO_POSE_HPP_
```

**What problem does this solve?**  
If `bt_main.cpp` includes both `nav_to_pose.hpp` and `some_other.hpp`, and `some_other.hpp` also includes `nav_to_pose.hpp`, the class would be defined twice → compile error. The guard makes the second paste a no-op.

The naming convention `PACKAGE__FILENAME_HPP_` is the ROS 2 standard. Always follow it.

---

## 4. Namespaces — Avoiding Name Clashes

```cpp
// Every .hpp wraps its code in:
namespace nav_bt_stack
{
  class NavToPose { /* ... */ };
  class BackUp    { /* ... */ };
}  // namespace nav_bt_stack
```

A namespace is like a folder for names. Without it, if another package also defines `NavToPose`, there would be a conflict. With namespaces, the full name becomes `nav_bt_stack::NavToPose`, which is unique.

**In `bt_main.cpp`:**
```cpp
using namespace std::chrono_literals;  // lets you write 2s instead of std::chrono::seconds(2)

// To use the class you either:
registerRosLeaf<nav_bt_stack::NavToPose>(...);   // fully qualified
// OR put "using namespace nav_bt_stack;" at the top (not done here — keeps it explicit)
```

> **Tip:** In ROS 2 code, avoid `using namespace std;` in header files — it pollutes every file that includes yours.

---

## 5. Variables, Types, and `const`

### Fundamental types you'll see

| Type | What it stores | Example |
|---|---|---|
| `int` | whole numbers | `int n = 6;` |
| `double` | floating-point | `double x = 1.5;` |
| `float` | smaller float | `float speed = 0.15f;` |
| `bool` | true/false | `bool done = false;` |
| `std::string` | text | `std::string frame = "map";` |
| `size_t` | non-negative index/size | `size_t i = 0;` |

### `const` — values that must not change

```cpp
// From bt_main.cpp
const std::string share = ament_index_cpp::get_package_share_directory("nav_bt_stack");
const std::string tree_id = node->declare_parameter<std::string>("tree_id", "GpsrTask");
const double tick_hz = node->declare_parameter<double>("tick_hz", 20.0);
```

`const` tells the compiler (and your teammates) "this value is set once and never changes." It prevents accidental modification and enables compiler optimizations.

---

## 6. Functions

### Free function (outside any class)

```cpp
// From tf_utils.hpp
inline double poseDistance(
  const geometry_msgs::msg::PoseStamped & a,
  const geometry_msgs::msg::PoseStamped & b)
{
  return std::hypot(
    a.pose.position.x - b.pose.position.x,
    a.pose.position.y - b.pose.position.y);
}
```

- **Return type** comes first: `double`
- **Name**: `poseDistance`
- **Parameters**: two `const` references to `PoseStamped` (more on `&` in §18)
- **Body**: computes Euclidean distance using `std::hypot` (= √(dx²+dy²))
- `inline` means "paste this code at the call site" — small helpers in headers use this to avoid "multiple definition" linker errors.

### Function with a template (generic function)

```cpp
// From bt_main.cpp
template<typename T>
static void registerRosLeaf(
  BT::BehaviorTreeFactory & factory, const std::string & tag,
  const rclcpp::Node::SharedPtr & node)
{
  BT::NodeBuilder builder =
    [node](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<T>(name, config, node);
    };
  factory.registerBuilder<T>(tag, builder);
}
```

`T` is a **placeholder type** — the caller decides what it is (see §14).

---

## 7. Classes — The Heart of Object-Oriented C++

A class bundles **data** (member variables) and **behaviour** (member functions / methods) into one unit.

Here is the skeleton of `BackUp` (simplified):

```cpp
class BackUp : public BT::StatefulActionNode
{
public:                              // visible to everyone
  // --- type aliases (§15) ---
  using ActionT = nav2_msgs::action::BackUp;

  // --- constructor ---
  BackUp(const std::string & name,
         const BT::NodeConfiguration & config,
         const rclcpp::Node::SharedPtr & node)
  : BT::StatefulActionNode(name, config),   // call parent constructor
    node_(node)                             // initialise member
  {
    client_ = rclcpp_action::create_client<ActionT>(node_, "backup");
  }

  // --- static method (§11) ---
  static BT::PortsList providedPorts() { /* ... */ }

  // --- overridden virtual methods (§12) ---
  BT::NodeStatus onStart()   override { /* ... */ }
  BT::NodeStatus onRunning() override { /* ... */ }
  void           onHalted()  override { /* ... */ }

private:                             // hidden from the outside
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<ActionT>::SharedPtr client_;
  // ... more members ...
};
```

**Mental model:** The class is a blueprint. Each time you create an *instance* of the class, you get an independent object with its own copy of the member variables.

---

## 8. Inheritance — Reusing and Extending Classes

All action nodes in the project **inherit** from BehaviorTree.CPP base classes:

```
BT::StatefulActionNode         BT::SyncActionNode    BT::ConditionNode  BT::DecoratorNode
        ▲                              ▲                    ▲                  ▲
        │                              │                    │                  │
   NavToPose                  MockGpsrCommands         VoxelCheck         ForEachPose
   BackUp                     CommandField             CommandKindIs
   InterleavedPlanner         GetViewpoints            CommandLocated
   WaitForDoorOpen
```

Syntax: `class Child : public Parent { ... };`

The child:
- **Gets** all the parent's methods and data for free.
- **Can override** (`override` keyword) specific methods to give them new behaviour.
- **Must not** duplicate things the parent already handles.

This is why you never write a `tick()` loop yourself — `StatefulActionNode` handles that; you just fill in `onStart()`, `onRunning()`, and `onHalted()`.

### The four BT node types

| Base class | When to use | Example in project |
|---|---|---|
| `StatefulActionNode` | Long-running async action (sends a goal, polls result) | `NavToPose`, `BackUp`, `WaitForDoorOpen` |
| `SyncActionNode` | Completes in one tick (no waiting) | `MockGpsrCommands`, `GetViewpoints`, `CommandField` |
| `ConditionNode` | Returns SUCCESS/FAILURE instantly | `VoxelCheck`, `CommandKindIs`, `CommandLocated` |
| `DecoratorNode` | Wraps a single child, modifies its behaviour | `ForEachPose` |

---

## 9. Constructors and Member Initializer Lists

The constructor is the method called **when an object is created**. C++ constructors have a special syntax: the **member initializer list**, written after a colon before the `{`:

```cpp
// From nav_to_pose.hpp
NavToPose(
  const std::string & name,
  const BT::NodeConfiguration & config,
  const rclcpp::Node::SharedPtr & node)
: BT::StatefulActionNode(name, config),    // 1. call parent constructor
  node_(node)                              // 2. initialise member node_
{
  // 3. extra setup that needs a fully constructed object
  client_ = rclcpp_action::create_client<ActionT>(node_, "navigate_to_pose");
}
```

**Why use the initializer list instead of assigning inside `{}`?**  
- Some members (references, `const` members, parent classes) *must* be initialized in the list — you cannot assign to them later.
- It is also faster: assignment inside `{}` would default-construct then overwrite; the list constructs directly.

**Brace-initialized defaults** (a shorthand for simple members):

```cpp
// From nav_to_pose.hpp — inside the class, at the member declaration:
bool done_{false};
bool accepted_{false};
rclcpp_action::ResultCode result_code_{rclcpp_action::ResultCode::UNKNOWN};
```

The `{false}` sets the initial value right where the member is declared. This is equivalent to writing `done_ = false;` in every constructor — much less repetition.

---

## 10. Access Specifiers: `public`, `private`, `protected`

```cpp
class VoxelCheck : public BT::ConditionNode
{
public:   // ← anyone can call these
  VoxelCheck(/* ... */);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:  // ← only methods of VoxelCheck itself can touch these
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<Costmap>::SharedPtr sub_;
  Costmap::ConstSharedPtr last_map_;
  // ...
};
```

| Specifier | Accessible from |
|---|---|
| `public` | Everywhere |
| `private` | Only inside this class |
| `protected` | Inside this class AND derived classes |

**Why hide things?**  
If `last_map_` were public, any piece of code could corrupt it. Making it private means only `VoxelCheck`'s own methods can touch it. This is called **encapsulation** — it makes bugs easier to find.

---

## 11. `static` Members and Methods

`static` on a class method means it belongs to the **class itself**, not to any particular object. You can call it without creating an instance.

```cpp
// Every BT node must have this:
static BT::PortsList providedPorts()
{
  return {
    BT::InputPort<double>("distance", 0.30, "reverse distance [m]"),
    BT::InputPort<double>("speed",    0.15, "reverse speed [m/s]"),
  };
}
```

The BehaviorTree.CPP framework calls `BackUp::providedPorts()` **at registration time** to discover what XML attributes a node accepts — before any `BackUp` object exists. That's why it must be `static`.

---

## 12. `override` — Safe Virtual Method Overriding

When a parent class marks a method `virtual`, child classes can replace its behaviour. The `override` keyword tells the compiler "I mean to override a virtual method — check this is actually valid":

```cpp
BT::NodeStatus onStart()   override { /* ... */ }
BT::NodeStatus onRunning() override { /* ... */ }
void           onHalted()  override { /* ... */ }
```

If you mistype the name (e.g., `onstart()`) the compiler will give you an error because no such virtual method exists in the parent. Without `override`, you would silently create a *new* method that is never called — a hard-to-find bug.

**Always write `override`** when you intend to override a virtual method.

---

## 13. Smart Pointers — Memory Without `delete`

Raw pointers (`NavToPose* p = new NavToPose(...);`) require you to call `delete p;` when done. Forget to do it → memory leak. Call it twice → crash.

C++ solves this with **smart pointers** that automatically delete the object when nobody needs it.

### `std::shared_ptr<T>` — shared ownership

```cpp
// ROS 2 nodes are almost always shared_ptr
rclcpp::Node::SharedPtr node_;  // = std::shared_ptr<rclcpp::Node>

// Create one:
auto node = std::make_shared<rclcpp::Node>("nav_bt_stack");

// It's reference-counted: when the last shared_ptr goes away, the Node is deleted.
```

You'll also see `XXX::SharedPtr` typedefs throughout ROS 2 — they are all `shared_ptr` aliases.

### `std::make_shared<T>(args...)` — the safe way to create

```cpp
auto node = std::make_shared<rclcpp::Node>("nav_bt_stack");
// ↑ allocates + constructs in one step; no bare `new`
```

### `std::unique_ptr<T>` — sole ownership

```cpp
// From the registerRosLeaf helper:
return std::make_unique<T>(name, config, node);
// ↑ only one unique_ptr can own this object; when it goes away, the object is deleted
```

`unique_ptr` is used for BT nodes because each node has exactly one owner (the tree).

### `std::shared_future<T>` — async result handle

```cpp
std::shared_future<GoalHandle::SharedPtr> future_goal_handle_;
// ...
future_goal_handle_ = client_->async_send_goal(goal, opts);
```

A `future` is a "promise of a result later". `wait_for(0s)` checks if it's ready without blocking (non-blocking poll).

---

## 14. Templates — Generic Code

Templates let you write one function or class that works with many types.

### Function template

```cpp
// bt_main.cpp
template<typename T>
static void registerRosLeaf(
  BT::BehaviorTreeFactory & factory,
  const std::string & tag,
  const rclcpp::Node::SharedPtr & node)
{
  BT::NodeBuilder builder =
    [node](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<T>(name, config, node);  // T is the concrete class
    };
  factory.registerBuilder<T>(tag, builder);
}

// Called as:
registerRosLeaf<nav_bt_stack::NavToPose>(factory, "NavToPose", node);
registerRosLeaf<nav_bt_stack::BackUp>   (factory, "BackUp",    node);
// The same function body handles both — T is substituted at compile time
```

### Class template (from STL)

```cpp
std::vector<Pose>           // T = Pose
std::map<std::pair<int,int>, double>  // key=pair, value=double
std::shared_ptr<rclcpp::Node>         // T = rclcpp::Node
```

You don't write these template classes — you just use them by supplying the type in `<>`.

### Template specialisation in BT ports

```cpp
BT::InputPort<double>("distance", 0.30, "reverse distance [m]")
BT::InputPort<Pose>("goal_pose", "ready PoseStamped (preferred)")
BT::OutputPort<std::vector<Pose>>("ordered_goals", "goals in optimal order")
```

`InputPort<T>` is a template — the `T` tells the BT framework how to serialize/deserialize the value on the blackboard.

---

## 15. Type Aliases with `using`

`using` creates a shorter, more readable name for a type:

```cpp
// Inside class NavToPose:
using ActionT    = nav2_msgs::action::NavigateToPose;
using GoalHandle = rclcpp_action::ClientGoalHandle<ActionT>;
using Pose       = geometry_msgs::msg::PoseStamped;
```

Now instead of writing `rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>` everywhere, you write `GoalHandle`. This is purely a compile-time rename — no runtime cost.

`using` at namespace scope (outside a class) works the same way and is the modern replacement for `typedef`.

---

## 16. STL Containers: `std::vector`, `std::map`, `std::string`

These are the workhorses of C++ data storage.

### `std::vector<T>` — a resizable array

```cpp
// From interleaved_planner.hpp
std::vector<Pose> goals_;      // list of goal poses
std::vector<std::pair<int,int>> pairs_;  // list of (from, to) pairs

// Add an element:
pairs_.emplace_back(-1, i);   // constructs a pair in-place (faster than push_back)

// Access by index:
const Pose & p = goals_[2];

// Size:
const int n = static_cast<int>(goals_.size());

// Reserve memory upfront:
ordered.reserve(n);   // avoids repeated reallocations

// Iterate (see §20):
for (int i : best) { ordered.push_back(goals_[i]); }
```

### `std::map<Key, Value>` — a sorted key→value store

```cpp
// From interleaved_planner.hpp
std::map<std::pair<int,int>, double> costs_;

// Insert / update:
costs_[{-1, 0}] = 2.5;    // key={-1,0}, value=2.5

// Lookup (throws if missing):
double c = costs_.at({-1, order.front()});

// Check if key exists:
auto it = fake_targets_.find("cup");
if (it == fake_targets_.end()) { /* not found */ }
else { Pose p = it->second; }   // it->second = value
```

### `std::string` — a text object

```cpp
std::string frame = "map";
frame += "_extra";           // concatenation
frame.c_str();               // get a raw C string (for printf-style logging)
frame.empty();               // check if ""
```

---

## 17. Lambda Functions — Inline Anonymous Functions

A lambda is a function defined inline, without a name. They're used heavily for callbacks.

### Basic syntax

```cpp
[capture_list](parameter_list) -> return_type {
  // body
}
```

### In the project: ROS 2 subscription callback

```cpp
// From voxel_check.hpp
sub_ = node_->create_subscription<Costmap>(
  "/local_costmap/costmap", rclcpp::QoS(1).transient_local(),
  [this](Costmap::ConstSharedPtr msg) {   // ← lambda
    last_map_ = msg;                      // stores the latest costmap
  });
```

- `[this]` — **capture**: the lambda can access the object's members (`last_map_`).
- `(Costmap::ConstSharedPtr msg)` — **parameter**: called by ROS 2 every time a message arrives.
- `{last_map_ = msg;}` — **body**: saves the message.

### In the project: action result callback

```cpp
// From nav_to_pose.hpp
opts.result_callback =
  [this](const GoalHandle::WrappedResult & result) {
    done_ = true;
    result_code_ = result.code;
  };
```

- Captures `this` (needs to set `done_` and `result_code_`).
- Called asynchronously when the action server finishes.

### In the project: node builder

```cpp
// From bt_main.cpp — registerRosLeaf
BT::NodeBuilder builder =
  [node](const std::string & name, const BT::NodeConfiguration & config) {
    return std::make_unique<T>(name, config, node);
  };
```

- `[node]` captures the node shared pointer by value (a copy of the `shared_ptr`, which increments the reference count).

---

## 18. References (`&`) and `const` References

A **reference** is an alias — another name for the same object, not a copy.

```cpp
void foo(std::string & s) {   // non-const reference: can modify s
  s += " modified";
}

void bar(const std::string & s) {  // const reference: cannot modify, avoids copying
  std::cout << s;
}
```

### Why does the project pass almost everything by `const &`?

```cpp
NavToPose(
  const std::string & name,          // don't copy the string
  const BT::NodeConfiguration & config,
  const rclcpp::Node::SharedPtr & node)  // don't copy the shared_ptr (just alias it)
```

Copying a `std::string` allocates heap memory. Passing by `const &` gives you the same string without any allocation. This is the idiomatic C++ way: **pass big objects by `const &`, small primitives by value**.

### The `->` operator

When you have a pointer (or smart pointer) to an object, use `->` instead of `.`:

```cpp
node_->get_logger()    // node_ is a shared_ptr<Node>, -> dereferences it
node_->now()
client_->async_send_goal(goal, opts)
```

`.` is for direct objects; `->` is for pointers.

---

## 19. `auto` — Let the Compiler Deduce the Type

```cpp
auto node = std::make_shared<rclcpp::Node>("nav_bt_stack");
// compiler sees: make_shared<Node>(...) returns shared_ptr<Node>
// so `node` has type shared_ptr<Node>
```

`auto` doesn't mean "no type" — it means "figure out the type from the right-hand side." Use it when the type is obvious from context and writing it out would be verbose.

```cpp
// From interleaved_planner.hpp
const auto & a = path.poses[i - 1].pose.position;  // auto deduced as PoseStamped::position type
const auto & b = path.poses[i].pose.position;
```

---

## 20. Range-Based `for` Loops

```cpp
// Iterate over every element in a container:
for (const auto & p : pairs_) {
  costs_[p] = euclid(p);
}

// Equivalent to:
for (size_t i = 0; i < pairs_.size(); ++i) {
  costs_[pairs_[i]] = euclid(pairs_[i]);
}
```

The range-based form is shorter and less error-prone (no off-by-one).

- `const auto &` — don't copy the element, and don't modify it.
- Drop `const` if you need to modify the element.
- Drop `&` only for small types like `int` or `double`.

---

## 21. `static_cast` — Explicit Type Conversion

C++ will not silently convert between types that might lose information. Use `static_cast` to be explicit:

```cpp
// From interleaved_planner.hpp
const int n = static_cast<int>(goals_.size());
// goals_.size() returns size_t (unsigned). Casting to int is intentional.

// From voxel_check.hpp
goal.speed = static_cast<float>(speed);  // double → float, intentional precision loss
const int rad_cells = static_cast<int>(std::ceil(radius / res));
```

If you see a warning about "narrowing conversion" (e.g., `double` → `int`), add `static_cast`. Never use C-style casts `(int)x` in C++ — they're less safe and harder to grep.

---

## 22. Exception Handling: `try` / `catch`

TF2 (the transform library) throws exceptions when a coordinate transform is not available. The project handles them gracefully:

```cpp
// From tf_utils.hpp
try {
  tf = buffer_->lookupTransform(global_frame_, base_frame_, tf2::TimePointZero);
} catch (const tf2::TransformException & e) {
  RCLCPP_WARN_THROTTLE(
    node_->get_logger(), *node_->get_clock(), 2000,
    "[RobotPose] TF %s->%s unavailable: %s",
    global_frame_.c_str(), base_frame_.c_str(), e.what());
  return false;   // signal failure to the caller
}
```

- `try { ... }` — code that might throw.
- `catch (const ExceptionType & e) { ... }` — handle the specific exception.
- `e.what()` — get a human-readable description of the error.
- Always catch by `const &` to avoid copying the exception object.

---

## 23. Brace Initialization of Member Variables

```cpp
// From nav_to_pose.hpp private section:
bool done_{false};
bool accepted_{false};
rclcpp_action::ResultCode result_code_{rclcpp_action::ResultCode::UNKNOWN};
```

These are **in-class member initializers**. The `{value}` sets the default value directly at the declaration. Without this, members would contain garbage until the constructor runs.

This C++11 feature means you don't need to repeat the initialization in every constructor — very useful when a class has many constructors.

---

## 24. The Build System: CMakeLists.txt and package.xml

### `package.xml` — declares dependencies

```xml
<package format="3">
  <name>nav_bt_stack</name>
  <depend>rclcpp</depend>
  <depend>rclcpp_action</depend>
  <depend>behaviortree_cpp_v3</depend>
  <depend>nav2_msgs</depend>
  <!-- ... -->
  <buildtool_depend>ament_cmake</buildtool_depend>
</package>
```

This is metadata: which ROS 2 packages does this package need? `colcon` reads this to determine build order (if package A depends on B, build B first).

### `CMakeLists.txt` — tells the compiler what to build

```cmake
cmake_minimum_required(VERSION 3.8)
project(nav_bt_stack)

# Find each dependency (makes headers + libs available):
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
# ...

# Build the executable:
add_executable(bt_main src/bt_main.cpp)

# Add our own include/ folder to the search path:
target_include_directories(bt_main PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

# Link all ROS 2 dependencies:
ament_target_dependencies(bt_main ${dependencies})

# Install the binary so ROS 2 can find it:
install(TARGETS bt_main DESTINATION lib/${PROJECT_NAME})

# Install shared files (XML trees, configs):
install(DIRECTORY bt_xml DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config  DESTINATION share/${PROJECT_NAME})
```

**Workflow:**
```bash
cd ~/nav-bt-stack/ros2_ws
colcon build --packages-select nav_bt_stack
source install/setup.bash
ros2 run nav_bt_stack bt_main
```

---

## 25. ROS 2 Concepts in C++

### 25.1 Node

A **node** is the basic unit of ROS 2. It has a name, can communicate with other nodes, and has a clock.

```cpp
// Create a node:
auto node = std::make_shared<rclcpp::Node>("nav_bt_stack");

// Read a parameter (with default):
std::string tree_id = node->declare_parameter<std::string>("tree_id", "GpsrTask");

// Get the logger:
RCLCPP_INFO(node->get_logger(), "Message: %s", some_string.c_str());
```

### 25.2 Logging macros

| Macro | Level | Notes |
|---|---|---|
| `RCLCPP_INFO(logger, fmt, ...)` | Informational | Normal operation |
| `RCLCPP_WARN(logger, fmt, ...)` | Warning | Something unexpected |
| `RCLCPP_ERROR(logger, fmt, ...)` | Error | Operation failed |
| `RCLCPP_WARN_THROTTLE(logger, clock, ms, fmt, ...)` | Throttled warn | Max once per `ms` milliseconds |

These work like `printf` — `%s` for strings, `%d` for ints, `%.2f` for doubles.

### 25.3 Subscriptions

```cpp
// From voxel_check.hpp — subscribe to a topic:
sub_ = node_->create_subscription<Costmap>(
  "/local_costmap/costmap",      // topic name
  rclcpp::QoS(1).transient_local(),  // quality-of-service
  [this](Costmap::ConstSharedPtr msg) {  // callback (lambda)
    last_map_ = msg;
  });
```

Every time a message arrives on `/local_costmap/costmap`, the lambda is called. The subscription stays alive as long as `sub_` lives.

### 25.4 Action Clients

Actions are like service calls, but for long-running tasks. The flow:
1. **Send goal** → get a future handle
2. **Poll** the handle each tick (non-blocking)
3. **Cancel** if the tree halts

```cpp
// 1. Create the client once (in constructor):
client_ = rclcpp_action::create_client<ActionT>(node_, "navigate_to_pose");

// 2. onStart(): send the goal
if (!client_->wait_for_action_server(std::chrono::seconds(2))) {
  return BT::NodeStatus::FAILURE;  // server not up
}
future_goal_handle_ = client_->async_send_goal(goal, opts);

// 3. onRunning(): poll without blocking
if (future_goal_handle_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
  goal_handle_ = future_goal_handle_.get();
  accepted_ = (goal_handle_ != nullptr);
}
if (done_ && result_code_ == rclcpp_action::ResultCode::SUCCEEDED) {
  return BT::NodeStatus::SUCCESS;
}

// 4. onHalted(): cancel if preempted
if (accepted_ && goal_handle_) {
  client_->async_cancel_goal(goal_handle_);
}
```

### 25.5 `rclcpp::spin_some` and the main loop

```cpp
// From bt_main.cpp
rclcpp::Rate rate(tick_hz);  // e.g. 20 Hz
while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
  status = tree.tickRoot();     // advance the behavior tree
  rclcpp::spin_some(node);      // process any pending callbacks (subscriptions, etc.)
  rate.sleep();                 // wait until next tick
}
```

`rclcpp::spin_some` processes pending messages **without blocking**. It's the "non-blocking version" of `rclcpp::spin`.

### 25.6 TF2 — Coordinate Transforms

The robot has many frames: `map` (global), `odom` (drift-corrected local), `base_link` (robot center), `base_scan` (lidar). TF2 knows the transform between all of them at any time.

```cpp
// From tf_utils.hpp — the RobotPose helper class
buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);

// Look up transform: where is base_link in the map frame?
tf = buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);

// Extract position:
out.pose.position.x = tf.transform.translation.x;
out.pose.position.y = tf.transform.translation.y;
```

### 25.7 Parameters

Parameters are runtime-configurable values set in YAML files or on the command line:

```cpp
// Declare with a default (first time):
node->declare_parameter<double>("detection_range", 1.5);

// Read later:
node->get_parameter("detection_range").as_double();

// The "has_parameter?" guard (needed if multiple leaves share the same node):
double range = node->has_parameter("detection_range")
  ? node->get_parameter("detection_range").as_double()
  : node->declare_parameter<double>("detection_range", 1.5);
```

---

## 26. BehaviorTree.CPP Patterns

### 26.1 The Blackboard — shared data store

The blackboard is a key-value store that all BT nodes share. Ports are the typed interface to it.

```cpp
// Declare what ports this node uses:
static BT::PortsList providedPorts() {
  return {
    BT::InputPort<Pose>("goal_pose", "ready PoseStamped"),       // read from blackboard
    BT::OutputPort<std::vector<Pose>>("ordered_goals", "..."),   // write to blackboard
  };
}

// Read inside tick():
Pose goal_pose;
if (getInput("goal_pose", goal_pose) && !goal_pose.header.frame_id.empty()) { ... }

// Write inside tick():
setOutput("ordered_goals", ordered);
```

### 26.2 NodeStatus

Every `tick()`, `onStart()`, or `onRunning()` must return one of:

| Status | Meaning |
|---|---|
| `BT::NodeStatus::SUCCESS` | Task completed successfully |
| `BT::NodeStatus::FAILURE` | Task failed |
| `BT::NodeStatus::RUNNING` | Still working, check again next tick |

### 26.3 StatefulActionNode lifecycle

```
        tree.tickRoot()
              │
    ┌─────────▼──────────┐
    │  first call?        │─── yes ──► onStart()
    │  (not RUNNING)      │            returns RUNNING, SUCCESS, or FAILURE
    └─────────┬──────────┘
              │ (was RUNNING)
    ┌─────────▼──────────┐
    │  onRunning()        │──────────► returns RUNNING, SUCCESS, or FAILURE
    └─────────────────────┘

    tree.haltTree()  ──────────────► onHalted()  (cleanup, cancel goal)
```

### 26.4 `ForEachPose` — a Decorator that loops

```cpp
// ForEachPose::tick() (simplified)
while (index_ < poses_.size()) {
  const BT::NodeStatus child_status = child_node_->executeTick();

  if (child_status == BT::NodeStatus::RUNNING) {
    return BT::NodeStatus::RUNNING;   // wait, child is still working
  }
  haltChild();                        // reset child for next pose

  if (mode_ == "find" && child_status == BT::NodeStatus::SUCCESS) {
    reset();
    return BT::NodeStatus::SUCCESS;   // found it — stop early
  }
  ++index_;
}
reset();
return (mode_ == "visit_all") ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
```

Key concept: the decorator calls `child_node_->executeTick()` and interprets the result. The child does not loop — the decorator calls it repeatedly with different data (the current pose on the blackboard).

---

## 27. Putting It All Together — Reading `bt_main.cpp`

Let's walk through every line of the main entry point with fresh eyes:

```cpp
#include <memory>   // std::make_shared, std::make_unique
#include <string>   // std::string
#include <vector>   // std::vector
```

Standard library includes (§2).

```cpp
#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
```

Third-party and ROS 2 includes.

```cpp
#include "nav_bt_stack/nav_to_pose.hpp"
// ... all the other node headers
```

Our own node definitions — each header has its own include guard (§3).

```cpp
using namespace std::chrono_literals;
```

Allows `2s` as shorthand for `std::chrono::seconds(2)` (§4 — namespace import).

```cpp
template<typename T>
static void registerRosLeaf(
  BT::BehaviorTreeFactory & factory, const std::string & tag,
  const rclcpp::Node::SharedPtr & node)
{
  BT::NodeBuilder builder =
    [node](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<T>(name, config, node);  // T = e.g. NavToPose
    };
  factory.registerBuilder<T>(tag, builder);
}
```

A **template helper** (§14) that reduces boilerplate. The lambda (§17) captures the node and constructs any leaf type on demand.

```cpp
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);  // must be first — initialises ROS 2
  auto node = std::make_shared<rclcpp::Node>("nav_bt_stack");  // create our node (§25.1)
```

`int main(int argc, char ** argv)` is the program entry point. `argc` = argument count, `argv` = argument values (strings). ROS 2 reads its config from `argv`.

```cpp
  const std::string share = ament_index_cpp::get_package_share_directory("nav_bt_stack");
  const std::string tree_id = node->declare_parameter<std::string>("tree_id", "GpsrTask");
  const double tick_hz = node->declare_parameter<double>("tick_hz", 20.0);
```

Get the installed share path (where `bt_xml/` lives) and read ROS 2 parameters with defaults.

```cpp
  BT::BehaviorTreeFactory factory;

  registerRosLeaf<nav_bt_stack::NavToPose>(factory, "NavToPose", node);
  // ... register all ROS leaves ...
  factory.registerNodeType<nav_bt_stack::ForEachPose>("ForEachPose");  // no ROS node needed
```

A factory is a registry of all BT node types. You must register every class before the tree can use it by XML tag name.

```cpp
  factory.registerBehaviorTreeFromFile(share + "/bt_xml/subtrees/room_scan.xml");
  // ... register all XML files ...
  auto tree = factory.createTree(tree_id);
```

Load XML tree definitions (the "behavior" logic). `createTree` assembles the tree by matching XML tags to registered C++ classes.

```cpp
  rclcpp::Rate rate(tick_hz);
  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
    status = tree.tickRoot();  // advance the tree one step
    rclcpp::spin_some(node);   // handle ROS callbacks
    rate.sleep();              // throttle to tick_hz
  }
```

The main loop (§25.5): tick the tree, process callbacks, sleep.

```cpp
  RCLCPP_INFO(
    node->get_logger(), "Tree '%s' finished: %s", tree_id.c_str(),
    status == BT::NodeStatus::SUCCESS ? "SUCCESS" : "FAILURE");

  rclcpp::shutdown();  // clean up ROS 2
  return 0;
}
```

Log the outcome and shut down.

---

## 28. Quick-Reference Cheat Sheet

```cpp
// ── Include / Guard ─────────────────────────────────────────────────
#ifndef MY_PKG__MY_FILE_HPP_
#define MY_PKG__MY_FILE_HPP_
// ... code ...
#endif

// ── Namespace ───────────────────────────────────────────────────────
namespace my_pkg { class Foo {}; }
my_pkg::Foo obj;

// ── Type alias ──────────────────────────────────────────────────────
using MyType = some::very::long::TypeName;

// ── Smart pointers ──────────────────────────────────────────────────
auto sp = std::make_shared<MyClass>(arg1, arg2);   // shared ownership
auto up = std::make_unique<MyClass>(arg1, arg2);   // sole ownership
sp->method();   // dereference with ->

// ── References ──────────────────────────────────────────────────────
void f(const std::string & s);   // read-only, no copy
void g(std::string & s);         // writable, no copy

// ── Lambda ──────────────────────────────────────────────────────────
auto cb = [this](Msg::ConstSharedPtr msg) { last_ = msg; };

// ── Template function ───────────────────────────────────────────────
template<typename T>
void reg(Factory & f) { f.register<T>(); }
reg<MyNode>(factory);

// ── Class skeleton ──────────────────────────────────────────────────
class MyNode : public BT::StatefulActionNode {
public:
  MyNode(const std::string & name, const BT::NodeConfiguration & cfg,
         const rclcpp::Node::SharedPtr & node)
  : BT::StatefulActionNode(name, cfg), node_(node) {}

  static BT::PortsList providedPorts() { return { BT::InputPort<double>("x") }; }

  BT::NodeStatus onStart()   override { return BT::NodeStatus::RUNNING; }
  BT::NodeStatus onRunning() override { return BT::NodeStatus::SUCCESS; }
  void           onHalted()  override {}

private:
  rclcpp::Node::SharedPtr node_;
  bool done_{false};
};

// ── ROS 2 snippets ──────────────────────────────────────────────────
auto node = std::make_shared<rclcpp::Node>("name");
node->declare_parameter<double>("p", 1.0);
RCLCPP_INFO(node->get_logger(), "value: %.2f", val);

sub_ = node->create_subscription<MsgT>("topic", qos, callback);
client_ = rclcpp_action::create_client<ActionT>(node, "action_name");

// ── STL quick reference ─────────────────────────────────────────────
std::vector<int> v = {1, 2, 3};
v.push_back(4);
v.emplace_back(5);
v.size();   v[0];

std::map<std::string, int> m;
m["key"] = 42;
m.at("key");           // throws if missing
m.find("key");         // returns iterator; != m.end() means found

// ── Casting ─────────────────────────────────────────────────────────
int n = static_cast<int>(some_size_t_value);
float f = static_cast<float>(some_double);

// ── Exception handling ──────────────────────────────────────────────
try {
  result = might_throw();
} catch (const SomeException & e) {
  // handle e.what()
}
```

---

## Further Reading

| Resource | URL / Command |
|---|---|
| cppreference.com | https://en.cppreference.com |
| ROS 2 C++ tutorial | https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html |
| BehaviorTree.CPP docs | https://www.behaviortree.dev |
| Nav2 Architecture | https://docs.nav2.org/concepts/index.html |
| C++ Core Guidelines | https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines |

> **Pro tip for your team leadership role:**  
> When reviewing pull requests, focus on three things from this guide:
> 1. **Smart pointers used correctly?** No bare `new`/`delete`.
> 2. **`override` keyword present?** Every virtual override must have it.
> 3. **BT status returned correctly?** An `onRunning()` that never returns `SUCCESS` or `FAILURE` will hang the tree forever.
