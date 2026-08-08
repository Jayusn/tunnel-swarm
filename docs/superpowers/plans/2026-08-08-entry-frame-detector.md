# Selectable Soft Entry Frame Detector Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a launch-selectable soft 3-D entry-frame detector that tolerates incomplete mapping and thin rods while preserving the existing detector as mode A.

**Architecture:** Extract the new frame-scoring geometry into a ROS-independent helper so synthetic voxel tests can drive implementation. `CorridorSearchManager` keeps its existing state machine and mode-A `detectDoorOpening()` unchanged; mode B converts accumulated points into the helper input, validates the resulting center with the existing XY path check, and returns the same `CandidateScore` interface.

**Tech Stack:** ROS Noetic, C++14, catkin, geometry_msgs/PCL, GoogleTest.

## Global Constraints

- `search_manager/entry_detection_mode` accepts exactly `a` or `b`; C++ defaults to `a` and the indoor FUEL launch defaults to `b`.
- Mode A retains the existing `detectDoorOpening()` behavior and parameters.
- Mode B starts from the map accumulated before takeoff and adds no fixed observation delay.
- Missing/unknown voxels reduce confidence but do not reject a frame by themselves.
- The top lintel is optional positive evidence; paired side posts are required.
- Known occupied voxels that remove the configured safe opening remain a hard rejection.
- Candidate confirmation, entry lock, through-point approach, and FUEL handoff remain shared and unchanged.

---

### Task 1: Testable Soft-Frame Geometry Core

**Files:**
- Create: `src/diff_planner/plan_manage/include/plan_manage/soft_entry_frame_detector.h`
- Create: `src/diff_planner/plan_manage/src/soft_entry_frame_detector.cpp`
- Create: `src/diff_planner/plan_manage/test/test_soft_entry_frame_detector.cpp`
- Modify: `src/diff_planner/plan_manage/CMakeLists.txt`

**Interfaces:**
- Consumes: `std::vector<FrameVoxel>` in frame-local coordinates `(forward, lateral, vertical)` and `SoftFrameParams`.
- Produces: `SoftFrameResult detectSoftEntryFrame(const std::vector<FrameVoxel>&, const SoftFrameParams&)`, with `detected`, `forward`, `center_lateral`, `center_vertical`, `width`, `post_height`, `known_free_depth`, `opening_occupied_ratio`, `max_blocked_width`, `lintel_support`, `support_count`, and `score`.

- [ ] **Step 1: Write the failing complete-frame test**

Create a synthetic frame with posts at lateral `-0.6/+0.6 m`, height `1.0 m`, a lintel, and a clear center extending `1.0 m` behind it. Assert `detected`, center within `0.1 m`, width within `0.15 m`, and positive lintel support.

- [ ] **Step 2: Register and run the test to verify RED**

Add under `if(CATKIN_ENABLE_TESTING)`:

```cmake
catkin_add_gtest(test_soft_entry_frame_detector
  test/test_soft_entry_frame_detector.cpp
  src/soft_entry_frame_detector.cpp)
target_link_libraries(test_soft_entry_frame_detector ${catkin_LIBRARIES})
```

Run:

```bash
catkin_make --pkg diff_planner --only-pkg-with-deps diff_planner tests
./devel/lib/diff_planner/test_soft_entry_frame_detector --gtest_filter='SoftEntryFrameDetector.CompleteFrame'
```

Expected: compilation fails because the helper types/functions do not exist.

- [ ] **Step 3: Implement the minimum grid and paired-post detector**

Use `0.10 m` lateral/vertical bins. Group occupied samples near the same forward plane, calculate each side post's total height and occupied-bin coverage, use inner post faces for width and center, and count lintel bins only as score evidence.

- [ ] **Step 4: Run the complete-frame test to verify GREEN**

Run the filtered command above. Expected: one passing test.

- [ ] **Step 5: Commit the geometry core**

```bash
git add src/diff_planner/plan_manage/include/plan_manage/soft_entry_frame_detector.h \
  src/diff_planner/plan_manage/src/soft_entry_frame_detector.cpp \
  src/diff_planner/plan_manage/test/test_soft_entry_frame_detector.cpp \
  src/diff_planner/plan_manage/CMakeLists.txt
git commit -m "feat: add testable soft entry frame geometry"
```

### Task 2: Incomplete Map and Internal-Rod Semantics

**Files:**
- Modify: `src/diff_planner/plan_manage/include/plan_manage/soft_entry_frame_detector.h`
- Modify: `src/diff_planner/plan_manage/src/soft_entry_frame_detector.cpp`
- Modify: `src/diff_planner/plan_manage/test/test_soft_entry_frame_detector.cpp`

**Interfaces:**
- Consumes: `SoftFrameParams::min_post_height`, `min_post_vertical_coverage`, `max_opening_occupied_ratio`, `max_blocked_width`, `safe_opening_width`, `min_known_free_depth=0.6`, and `full_free_depth=1.0`.
- Produces: the Task 1 result with rejection reason `no_posts`, `opening_blocked`, or `insufficient_depth`.

- [ ] **Step 1: Add failing fragmented-frame tests**

Add separate tests proving: 30% missing post bins and no lintel are accepted; a `0.10 m` center rod is accepted when safe width remains; a `0.50 m` central block is rejected; paired isolated posts with less than `0.6 m` known-free depth are rejected; unknown depth beyond the known `0.6 m` does not reject.

- [ ] **Step 2: Run all helper tests to verify RED**

```bash
./devel/lib/diff_planner/test_soft_entry_frame_detector
```

Expected: fragmented/rod/depth tests fail while `CompleteFrame` remains green.

- [ ] **Step 3: Implement coverage, obstruction, and depth scoring**

Calculate post coverage as occupied vertical bins divided by bins across the observed span. In the opening, compute both total occupied ratio and the widest contiguous occupied lateral run. Require a remaining contiguous opening at least `safe_opening_width`; treat free evidence from `0.6 m` as sufficient and add a bounded score bonus through `1.0 m`.

- [ ] **Step 4: Run all helper tests to verify GREEN**

Run the command above. Expected: all helper tests pass.

- [ ] **Step 5: Commit incomplete-map behavior**

```bash
git add src/diff_planner/plan_manage/include/plan_manage/soft_entry_frame_detector.h \
  src/diff_planner/plan_manage/src/soft_entry_frame_detector.cpp \
  src/diff_planner/plan_manage/test/test_soft_entry_frame_detector.cpp
git commit -m "feat: tolerate fragmented entry frames and thin rods"
```

### Task 3: Corridor Manager A/B Integration and Map Readiness

**Files:**
- Modify: `src/diff_planner/plan_manage/src/corridor_search_manager.cpp`
- Modify: `src/diff_planner/plan_manage/CMakeLists.txt`
- Modify: `src/diff_planner/plan_manage/test/test_soft_entry_frame_detector.cpp`

**Interfaces:**
- Consumes: `detectSoftEntryFrame(...)` from Tasks 1–2 and existing `CandidateScore`/`pathToThroughClear` behavior.
- Produces: `detectEntryCandidateModeB()`, mode dispatch in `detectEntryCandidate()`, and startup log `entry_detection_mode=a|b`.

- [ ] **Step 1: Add failing mode-validation and coordinate-conversion tests**

Test a pure `parseEntryDetectionMode(string)` helper: `a -> A`, `b -> B`, and invalid input -> A with `valid=false`. Test conversion of a frame result at yaw `90°` into the expected world center and through-point.

- [ ] **Step 2: Run tests to verify RED**

```bash
./devel/lib/diff_planner/test_soft_entry_frame_detector --gtest_filter='EntryDetectionMode.*:SoftFrameTransform.*'
```

Expected: fail because parsing and transformation helpers are absent.

- [ ] **Step 3: Implement mode B and preserve mode A**

Read all `search_manager/frame_b_*` parameters, convert stable accumulated occupancy/free-or-unknown evidence to frame-local voxels, call the helper, transform accepted inner edges and center back to world coordinates, then apply the existing centerline path check and fill `CandidateScore`. Dispatch `a` directly to the unchanged `detectDoorOpening()` and `b` to the new function. Invalid mode logs `ROS_ERROR` once and selects A.

- [ ] **Step 4: Fix readiness without adding observation delay**

Change readiness so a sufficiently dense accumulated map can be used immediately after trigger. Do not reset readiness merely because the last map message is older than `stale_cloud_timeout_`; continue logging map age, and reject only when no accumulated map or fewer than `map_ready_min_points_` stable points exist.

- [ ] **Step 5: Add RViz and diagnostic output**

Publish mode-B markers for left/right inner edges, optional lintel, fitted center, through-point, and rejection reason. Add one throttled summary containing post coverage, width, occupied ratio, blocked width, known-free depth, lintel score, and selected mode.

- [ ] **Step 6: Build and run tests**

```bash
catkin_make --pkg diff_planner --only-pkg-with-deps diff_planner
./devel/lib/diff_planner/test_soft_entry_frame_detector
```

Expected: build succeeds and all tests pass.

- [ ] **Step 7: Commit manager integration**

```bash
git add src/diff_planner/plan_manage/src/corridor_search_manager.cpp \
  src/diff_planner/plan_manage/src/soft_entry_frame_detector.cpp \
  src/diff_planner/plan_manage/include/plan_manage/soft_entry_frame_detector.h \
  src/diff_planner/plan_manage/test/test_soft_entry_frame_detector.cpp \
  src/diff_planner/plan_manage/CMakeLists.txt
git commit -m "feat: integrate selectable soft entry detector"
```

### Task 4: Launch Selection and End-to-End Verification

**Files:**
- Modify: `src/diff_planner/plan_manage/launch/exp/run_swarm_indoor1_fuel_exploration.launch`
- Test: ROS launch parameter inspection and current recorded/runtime map.

**Interfaces:**
- Consumes: `entry_detection_mode` and `frame_b_*` parameters from Task 3.
- Produces: launch argument `entry_detection_mode` with default `b`, passed as `search_manager/entry_detection_mode`.

- [ ] **Step 1: Add a failing launch assertion**

Use a shell assertion before editing:

```bash
roslaunch --ros-args diff_planner run_swarm_indoor1_fuel_exploration.launch entry_detection_mode:=b 2>&1 | rg 'entry_detection_mode'
```

Expected: no declared launch argument or parameter.

- [ ] **Step 2: Add the A/B launch switch and B defaults**

Declare `<arg name="entry_detection_mode" default="b"/>`, pass it into the manager, and add conservative `frame_b_*` values matching the tested geometry: `min_post_height=0.70`, `min_post_vertical_coverage=0.65`, `max_opening_occupied_ratio=0.12`, `max_blocked_width=0.20`, `safe_opening_width=0.55`, `min_known_free_depth=0.60`, and `full_free_depth=1.00`.

- [ ] **Step 3: Verify both parameter selections**

Start mode A and B separately, query `/corridor_search_manager/search_manager/entry_detection_mode`, and verify values `a` and `b`. Confirm startup logs name the selected mode.

- [ ] **Step 4: Verify on the live accumulated map**

With motors disarmed or workspace motion output disabled, trigger entrance search in mode B. Confirm the detector reaches candidate evaluation without the former warmup-reset loop, RViz center lies between inner post faces, thin rods do not shift it to a sub-gap, and no PositionCommand is released before three confirmations.

- [ ] **Step 5: Run final regression checks**

```bash
catkin_make --pkg diff_planner --only-pkg-with-deps diff_planner
./devel/lib/diff_planner/test_soft_entry_frame_detector
git diff --check
```

Expected: build succeeds, all tests pass, and `git diff --check` prints nothing.

- [ ] **Step 6: Commit launch configuration**

```bash
git add src/diff_planner/plan_manage/launch/exp/run_swarm_indoor1_fuel_exploration.launch
git commit -m "feat: select entry detector mode from launch"
```
